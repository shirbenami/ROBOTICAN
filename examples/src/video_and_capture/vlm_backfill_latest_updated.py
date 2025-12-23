#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import requests


@dataclass
class VlmResult:
    ok: bool
    caption: Optional[str]
    error: Optional[str]


def log(msg: str):
    print(msg, flush=True)


def pick_latest_subdir(parent: str, method: str, exclude_suffix: str = "_ann", allow_none: bool = False) -> Optional[str]:
    """
    Pick the latest subdirectory under `parent`.

    - method: "name" or "mtime"
    - exclude_suffix: ignore directories that end with this suffix
    - allow_none: if True, return None instead of raising when nothing is found
    """
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
        if allow_none:
            return None
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

    done = skipped = failed = 0

    for jpg in jpgs:
        jpg_path = os.path.join(folder, jpg)
        base = os.path.splitext(jpg)[0]
        json_path = os.path.join(folder, base + ".json")

        js = load_json(json_path)

        if (not force) and already_captioned(js):
            skipped += 1
            continue

        img_for_vlm = remap_path(jpg_path, path_src, path_dst)
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

        # Optional: keep status block (handy for debugging)
        js["vlm"] = {
            "status": "done" if res.ok else "failed",
            "endpoint": endpoint,
            "image_path_sent": img_for_vlm,
            "took_s": round(dt, 3),
            "error": None if res.ok else res.error,
            "updated_at_unix": time.time(),
        }

        # Optional: remove/stop using vlm_text to avoid confusion
        if "vlm_text" in js:
            js.pop("vlm_text", None)

        save_json(json_path, js)

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

    # Watch / folder selection behavior
    ap.add_argument("--watch", action="store_true", help="Keep running and process new frames as they appear")
    ap.add_argument("--watch-interval", type=float, default=2.0, help="Seconds between scans in --watch mode")
    ap.add_argument("--recent-seconds", type=float, default=60.0,
                    help="If the latest folder was modified within this time window, start watching it immediately")
    ap.add_argument("--debug", action="store_true",
                    help="Debug mode: use the latest (or --debug-folder) immediately and do not wait for a new folder")
    ap.add_argument("--debug-folder", default=None,
                    help="Specific folder to use in --debug mode (absolute or relative to --base-dir)")
    ap.add_argument("--exclude-suffix", default="_ann",
                    help="Ignore folders ending with this suffix (set '' to disable)")

    args = ap.parse_args()

    def resolve_folder_arg(folder_arg: str) -> str:
        # Allow absolute paths or paths relative to base-dir
        if os.path.isabs(folder_arg):
            return folder_arg
        return os.path.join(args.base_dir, folder_arg)

    def folder_age_s(folder: str) -> float:
        return time.time() - os.path.getmtime(folder)

    def run_folder(folder: str, force: bool) -> Tuple[int, int, int]:
        log(f"[worker] processing folder: {folder}")
        return process_folder(
            folder=folder,
            endpoint=args.endpoint,
            path_src=args.path_src,
            path_dst=args.path_dst,
            timeout_s=args.timeout,
            retries=args.retries,
            retry_sleep_s=args.retry_sleep,
            force=force,
            sleep_between_s=args.sleep_between,
        )

    # Non-watch mode keeps the legacy behavior (process latest once and exit)
    if not args.watch:
        latest = pick_latest_subdir(args.base_dir, args.latest_by, exclude_suffix=args.exclude_suffix)
        log(f"[worker] Latest folder: {latest}")
        done, skipped, failed = run_folder(latest, force=args.force)
        log(f"[worker] summary: done={done} skipped={skipped} failed={failed}")
        return

    # ---- Watch mode ----
    start_time = time.time()

    # Debug: lock to a chosen folder and start immediately (no waiting, no auto-switch)
    if args.debug:
        if args.debug_folder:
            folder = resolve_folder_arg(args.debug_folder)
            if not os.path.isdir(folder):
                raise FileNotFoundError(f"--debug-folder does not exist: {folder}")
        else:
            folder = pick_latest_subdir(args.base_dir, args.latest_by, exclude_suffix=args.exclude_suffix)

        log(f"[worker] DEBUG mode: locked to folder: {folder}")
        while True:
            done, skipped, failed = run_folder(folder, force=args.force)
            log(f"[worker] summary: done={done} skipped={skipped} failed={failed}")
            time.sleep(args.watch_interval)

    # Normal watch mode:
    # 1) If there's a folder modified within the last N seconds, start watching immediately.
    # 2) Otherwise, wait for a NEW folder to be created/modified after this script started.
    while True:
        latest = pick_latest_subdir(args.base_dir, args.latest_by, exclude_suffix=args.exclude_suffix, allow_none=True)
        if latest is None:
            log(f"[worker] No folders under {args.base_dir}. Waiting...")
            time.sleep(args.watch_interval)
            continue

        age = folder_age_s(latest)

        if age <= args.recent_seconds:
            log(f"[worker] Found recent folder (age={age:.1f}s): {latest}")
            break

        # Latest folder is old -> wait for a new folder during this run
        log(f"[worker] Latest folder is old (age={age:.1f}s). Waiting for a new folder...")
        time.sleep(args.watch_interval)

        newest = pick_latest_subdir(args.base_dir, "mtime", exclude_suffix=args.exclude_suffix, allow_none=True)
        if newest is not None and os.path.getmtime(newest) >= start_time:
            latest = newest
            log(f"[worker] New folder detected: {latest}")
            break

    # Now keep following the newest folder (switch when a newer capture folder appears),
    # and keep re-scanning the chosen folder to process new frames as they arrive.
    last_folder = None
    while True:
        latest_folder = pick_latest_subdir(args.base_dir, "mtime", exclude_suffix=args.exclude_suffix, allow_none=True)
        if latest_folder is None:
            log(f"[worker] No folders under {args.base_dir}. Waiting...")
            time.sleep(args.watch_interval)
            continue

        # Only switch to a new folder if it is "recent-ish" or created after our start time.
        # This prevents jumping back to an old folder.
        if os.path.getmtime(latest_folder) >= start_time or folder_age_s(latest_folder) <= args.recent_seconds:
            if latest_folder != last_folder:
                log(f"[worker] switched to folder: {latest_folder}")
                last_folder = latest_folder

        # Process whatever folder we're currently on (skip already-captioned by default)
        target = last_folder or latest_folder
        done, skipped, failed = run_folder(target, force=False)
        log(f"[worker] summary: done={done} skipped={skipped} failed={failed}")

        time.sleep(args.watch_interval)

