#!/usr/bin/env python3
import argparse
import datetime
import json
import os
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import requests


@dataclass
class VlmResult:
    ok: bool
    caption: Optional[str]
    error: Optional[str]


def log(msg: str):
    print(msg, flush=True)


def pick_latest_subdir(parent: str, method: str, exclude_suffix: str = "_ann") -> str:
    if not os.path.isdir(parent):
        raise FileNotFoundError(f"Parent dir does not exist: {parent}")

    subdirs = []
    for name in os.listdir(parent):
        if exclude_suffix and name.endswith(exclude_suffix):
            continue
        p = os.path.join(parent, name)
        if os.path.isdir(p):
            subdirs.append(p)

    if not subdirs:
        raise FileNotFoundError(f"No subfolders found under: {parent} (after excluding '{exclude_suffix}')")

    if method == "mtime":
        return max(subdirs, key=lambda p: os.path.getmtime(p))

    return max(subdirs, key=lambda p: os.path.basename(p))



def remap_path(path: str, src: Optional[str], dst: Optional[str]) -> str:
    """
    If your VLM server runs in a different container / mount namespace,
    you can remap local path -> server-visible path.

    Example:
      src=/home/user/jetson-containers/data
      dst=/mnt/VLM/jetson-data

    Then:
      /home/user/jetson-containers/data/R2/... -> /mnt/VLM/jetson-data/R2/...
    """
    if not src or not dst:
        return path

    src = os.path.abspath(src)
    path_abs = os.path.abspath(path)

    try:
        rel = os.path.relpath(path_abs, src)
    except Exception:
        return path

    # If rel begins with .., the path is not under src; don't remap
    if rel.startswith(".."):
        return path

    return os.path.normpath(os.path.join(dst, rel))


def load_json(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: str, obj: dict):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)

def _try_parse_json_string(s: str) -> Optional[dict]:
    s = (s or "").strip()
    if not s:
        return None
    if not (s.startswith("{") and s.endswith("}")):
        return None
    try:
        obj = json.loads(s)
        return obj if isinstance(obj, dict) else None
    except Exception:
        return None


def extract_prompt_response(vlm_caption_or_payload: str) -> Tuple[str, str]:
    """
    Accepts either:
    - plain caption string
    - JSON string with fields like auto_prompt/response_describe/...
    Returns: (prompt, response)
    """
    payload = _try_parse_json_string(vm := (vlm_caption_or_payload or ""))

    if payload:
        prompt = payload.get("auto_prompt") or payload.get("prompt") or "Describe the objects in the image"
        # Prefer your server's describe field names
        response = (
            payload.get("response_describe")
            or payload.get("response")
            or payload.get("text")
            or payload.get("caption")
            or ""
        )
        return str(prompt), str(response)

    # Not JSON → treat as raw caption
    return ("Describe the objects in the image", vm)



def already_captioned(js: dict) -> bool:
    # Your sidecar likely uses "vlm_text"; handle a couple of variants safely.
    if js.get("vlm_text"):
        return True
    vlm = js.get("vlm", {})
    if isinstance(vlm, dict) and vlm.get("text"):
        return True
    return False


def parse_caption_from_response(resp: requests.Response) -> str:
    """
    Try common response shapes:
      - {"text": "..."}
      - {"caption": "..."}
      - {"description": "..."}
      - or plain text body
    """
    ctype = (resp.headers.get("Content-Type") or "").lower()
    if "application/json" in ctype:
        data = resp.json()
        if isinstance(data, dict):
            for k in ("text", "caption", "description", "result"):
                v = data.get(k)
                if isinstance(v, str) and v.strip():
                    return v.strip()
        # fallback: stringify
        return json.dumps(data, ensure_ascii=False)

    # plain text fallback
    return resp.text.strip()


def call_vlm(endpoint: str, image_path: str, timeout_s: float, retries: int, retry_sleep_s: float) -> VlmResult:
    payload = {"image_path": image_path}

    last_err = None
    for attempt in range(1, retries + 1):
        try:
            t0 = time.time()
            resp = requests.post(endpoint, json=payload, timeout=timeout_s)
            dt = time.time() - t0

            if resp.status_code != 200:
                last_err = f"HTTP {resp.status_code}: {resp.text[:200]}"
                log(f"[vlm] attempt {attempt}/{retries} failed in {dt:.2f}s: {last_err}")
            else:
                caption = parse_caption_from_response(resp)
                if not caption:
                    last_err = "Empty caption"
                    log(f"[vlm] attempt {attempt}/{retries} got empty caption in {dt:.2f}s")
                else:
                    return VlmResult(ok=True, caption=caption, error=None)

        except Exception as e:
            last_err = str(e)
            log(f"[vlm] attempt {attempt}/{retries} exception: {last_err}")

        if attempt < retries:
            time.sleep(retry_sleep_s)

    return VlmResult(ok=False, caption=None, error=last_err)


def process_folder(
    folder: str,
    new_folder: str,
    endpoint: str,
    path_src: Optional[str],
    path_dst: Optional[str],
    timeout_s: float,
    retries: int,
    retry_sleep_s: float,
    force: bool,
    sleep_between_s: float,
) -> Tuple[int, int, int]:
    jpgs = sorted([
        f for f in os.listdir(folder)
        if f.lower().endswith(".jpg") and not f.lower().endswith("_ann.jpg")
    ])

    if not jpgs:
        log(f"[worker] No JPGs found in: {folder}")
        return (0, 0, 0)
    print(jpgs)
    done = skipped = failed = 0
    new_folder_path = Path(new_folder)
    new_folder_path.mkdir(parents=True, exist_ok=True)

    for jpg in jpgs:
        jpg_path = Path(folder) / jpg
        shutil.copy(jpg_path, Path(new_folder))
        jpg_path = new_folder_path / jpg
        base = os.path.splitext(jpg)[0]
        json_path = os.path.join(folder, base + ".json")
        js = load_json(json_path)
        json_path = new_folder_path / f"{base}.json"


        new_js = {}
        pose = js.get("pose")
        new_js.setdefault("pose", pose)
        new_js.setdefault("image", os.path.basename(str(jpg_path)))


        # tmp = json_path + ".tmp"
        with open(json_path, "w") as f:
            print(f)
            json.dump(new_js, f, indent=2)
        # os.replace(tmp, new_json_path)
        print("finished dump")
        if (not force) and already_captioned(js):
            skipped += 1
            continue

        img_for_vlm = remap_path(str(jpg_path), path_src, path_dst)
        log(f"[vlm] POST {endpoint}  image_path={img_for_vlm}")

        t0 = time.time()
        res = call_vlm(endpoint, img_for_vlm, timeout_s=timeout_s, retries=retries, retry_sleep_s=retry_sleep_s)
        dt = time.time() - t0
        log(f"[vlm] took {dt:.2f}s  ok={res.ok}")

        # Update JSON
        js.setdefault("image", os.path.basename(jpg_path))
        # Ensure list exists
        entries = js.get("entries")
        if not isinstance(entries, list):
            entries = []
            js["entries"] = entries

        if res.ok and res.caption:
            prompt, response = extract_prompt_response(res.caption)

            entries.append({
                "timestamp": int(time.time()),
                "prompt": prompt,
                "response": response,
            })

        # # Optional: keep status block (handy for debugging)
        # js["vlm"] = {
        #     "status": "done" if res.ok else "failed",
        #     "endpoint": endpoint,
        #     "image_path_sent": img_for_vlm,
        #     "took_s": round(dt, 3),
        #     "error": None if res.ok else res.error,
        #     "updated_at_unix": time.time(),
        # }
        #
        # # Optional: remove/stop using vlm_text to avoid confusion
        # if "vlm_text" in js:
        #     js.pop("vlm_text", None)
        #
        # save_json(str(json_path), js)

        if res.ok:
            done += 1
        else:
            failed += 1

        if sleep_between_s > 0:
            time.sleep(sleep_between_s)

    return (done, skipped, failed)


def main():
    ap = argparse.ArgumentParser(description="Backfill VLM captions for the latest capture folder.")
    ap.add_argument("--base-dir", required=True, help="Base directory that contains per-drone folders, e.g. /data/R2")
    ap.add_argument("--endpoint", required=True, help="VLM endpoint, e.g. http://192.168.131.22:8080/describe")
    ap.add_argument("--latest-by", choices=["name", "mtime"], default="name", help="How to choose the latest folder")
    ap.add_argument("--path-src", default=None, help="Local root path to remap from (optional)")
    ap.add_argument("--path-dst", default=None, help="Server-visible root path to remap to (optional)")
    ap.add_argument("--timeout", type=float, default=10.0, help="HTTP timeout per attempt")
    ap.add_argument("--retries", type=int, default=3, help="Retries per image")
    ap.add_argument("--retry-sleep", type=float, default=0.2, help="Sleep between retries")
    ap.add_argument("--sleep-between", type=float, default=3.0, help="Sleep between images (throttle)")
    ap.add_argument("--force", action="store_true", help="Re-caption even if vlm_text exists")
    ap.add_argument("--watch", action="store_true", help="Keep running and process the newest folder repeatedly")
    ap.add_argument("--watch-interval", type=float, default=2.0, help="Seconds between scans in --watch mode")
    ap.add_argument("--exclude-suffix", default="_ann",
                    help="Ignore folders ending with this suffix (set '' to disable)")


    args = ap.parse_args()

    if args.path_src and not args.path_dst:
        log("ERROR: if you provide --path-src you must also provide --path-dst")
        sys.exit(2)

    def run_once():
        latest = Path(pick_latest_subdir(args.base_dir, args.latest_by))
        parent= latest.parent
        new_folder_name = datetime.datetime.now().strftime("%Y_%m_%d___%H_%M_%S")
        new_file_path = Path(parent / new_folder_name)


        log(f"[worker] Latest folder: {latest}")
        done, skipped, failed = process_folder(
            folder=latest.as_posix(),
            new_folder=new_file_path.as_posix(),
            endpoint=args.endpoint,
            path_src=args.path_src,
            path_dst=args.path_dst,
            timeout_s=args.timeout,
            retries=args.retries,
            retry_sleep_s=args.retry_sleep,
            force=args.force,
            sleep_between_s=args.sleep_between,
        )
        log(f"[worker] summary: done={done} skipped={skipped} failed={failed}")
        return latest

    if not args.watch:
        run_once()
        return

    last_folder = None
    while True:
        try:
            latest_folder = pick_latest_subdir(args.base_dir, args.latest_by, exclude_suffix=args.exclude_suffix)

            # If folder changes, announce
            if latest_folder != last_folder:
                log(f"[worker] switched to latest folder: {latest_folder}")
                last_folder = latest_folder

            done, skipped, failed = process_folder(
                folder=latest_folder,
                endpoint=args.endpoint,
                path_src=args.path_src,
                path_dst=args.path_dst,
                timeout_s=args.timeout,
                retries=args.retries,
                retry_sleep_s=args.retry_sleep,
                force=False,  # in watch mode, default to skip already-captioned
                sleep_between_s=args.sleep_between,
            )
        except Exception as e:
            log(f"[worker] error: {e}")

        time.sleep(args.watch_interval)


if __name__ == "__main__":
    main()
