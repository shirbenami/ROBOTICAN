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


def process_folder(folder, path_src, path_dst, endpoint, ):
    jpgs = sorted([
        f for f in os.listdir(folder)
        if f.lower().endswith(".jpg") and not f.lower().endswith("_ann.jpg")
    ])

    for jpg in jpgs:
        jpg_path = os.path.join(folder, jpg)
        base = os.path.splitext(jpg)[0]
        img_for_vlm = remap_path(jpg_path, path_src, path_dst)
        log(f"[vlm] POST {endpoint}  image_path={img_for_vlm}")

        t0 = time.time()
        res = call_vlm(endpoint, img_for_vlm, timeout_s=60, retries=2, retry_sleep_s=5)
        dt = time.time() - t0
        log(f"[vlm] took {dt:.2f}s  ok={res.ok}")



def loop_over_folder(args):
    frames = list_frames(args.frames_dir)
    if not frames:
        print(f"[loop] no images under {args.frames_dir}")
        return
    angles_map = load_angles_map(args.angles_json) if args.angles_json else {}
    idx, counter = 0, 1
    print(f"[loop] iterating {len(frames)} frames, sleep={args.loop_sleep}s")
    while True:
        path = frames[idx]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            print(f"[loop] WARN unreadable: {path}")
        else:
            pose = get_pose_for_frame(path, angles_map=angles_map, from_name=args.angles_from_name)
            work = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if len(img.shape)==2 else img
            base = os.path.join(args.out, pose_to_name(pose))
            jpg_path, json_path = (base + ".jpg", base + ".json")
            if args.suffix:
                jpg_path, json_path = _unique_name(base, True, counter); counter += 1
            try:
                if args.save_full:
                    try: _save_jpg(jpg_path.replace(".jpg","_full.jpg"), img)
                    except Exception as e: print(f"[loop] WARN saving full: {e}")
                _save_jpg(jpg_path, work)
                _update_sidecar_json(json_path, pose, os.path.basename(jpg_path), vlm_text=None)
                print(f"[loop] saved {jpg_path} (src={os.path.basename(path)})")
            except Exception as e:
                print(f"[loop] ERROR: {e}")

            if args.vlm:
                prep_vlm(args, jpg_path, pose, json_path)
        time.sleep(max(0.0, args.loop_sleep))
        idx = (idx + 1) % len(frames)
        if not args.loop_frames and idx == 0:
            break
