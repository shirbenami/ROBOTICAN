#!/usr/bin/env python3
import argparse
import json
import os
import time
import requests


def log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _update_json_robust(json_path: str, new_data: dict):
    """ Reads existing JSON and merges new_data into it to prevent data loss. """
    for attempt in range(5):
        try:
            current_data = {}
            if os.path.exists(json_path):
                with open(json_path, 'r') as f:
                    current_data = json.load(f)

            # Deep merge the results
            for key, value in new_data.items():
                if isinstance(value, dict) and key in current_data:
                    current_data[key].update(value)
                else:
                    current_data[key] = value

            tmp_path = json_path + f".{os.getpid()}.tmp"
            with open(tmp_path, 'w') as f:
                json.dump(current_data, f, indent=2)
            os.replace(tmp_path, json_path)
            return True
        except Exception as e:
            time.sleep(0.1)
    return False


def call_vlm(url: str, img_path: str, timeout: float = 45.0):
    """ Sends absolute image path to VILA. """
    try:
        payload = {"image_path": os.path.abspath(img_path)}
        r = requests.post(url, json=payload, timeout=timeout)
        if r.status_code == 200:
            return r.json().get("caption", "").strip()
    except Exception as e:
        log(f"VLM Error: {e}")
    return None


def process_folder(folder_path, vlm_url, notify_url, frame_delay):
    """ Processes images in folder with a forced delay between frames. """
    # Get all .jpg files that aren't annotated
    images = sorted([f for f in os.listdir(folder_path) if f.endswith(".jpg") and "_ann" not in f])

    total = len(images)
    for i, img_name in enumerate(images):
        img_path = os.path.join(folder_path, img_name)
        json_path = img_path.replace(".jpg", ".json")

        # Skip if valid VLM result already exists
        if os.path.exists(json_path):
            try:
                with open(json_path, 'r') as f:
                    meta = json.load(f)
                if "vlm_caption" in meta or "vlm" in meta:
                    continue
            except:
                pass

        log(f"VLM -> {img_name} ({i + 1}/{total})")
        caption = call_vlm(vlm_url, img_path)

        if caption:
            # 1. Update JSON (Robustly)
            vlm_payload = {
                "vlm": {
                    "status": "success",
                    "caption": caption,
                    "ts": time.time(),
                    "endpoint": vlm_url
                }
            }
            _update_json_robust(json_path, vlm_payload)

            # 2. Notify Comm Manager with ID Anchor
            if notify_url:
                try:
                    requests.post(notify_url, json={"caption": caption, "image_name": img_name}, timeout=5.0)
                except Exception as e:
                    log(f"Notify failed: {e}")

            # 3. Forced delay between frames (if not the last frame)
            if i < total - 1:
                log(f"Waiting {frame_delay}s for next frame...")
                time.sleep(frame_delay)
        else:
            log(f"VLM Failed for {img_name}.")
            # # vlm_payload = {
            # #     "vlm": {
            # #         "status": "failed",
            # #         "ts": time.time(),
            # #         "error": "VLM returned no caption"
            # #     }
            # # }
            # _update_json_robust(json_path, vlm_payload)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-dir", required=True, help="Root folder for captures")
    parser.add_argument("--vlm-url", default="http://192.168.131.22:8080/describe")
    parser.add_argument("--notify-url", default="http://127.0.0.1:5001/from_vila")
    parser.add_argument("--wait-threshold", type=int, default=90, help="Wait if no new folder in X sec")
    parser.add_argument("--frame-delay", type=float, default=15.0, help="Seconds to wait between images in a folder")
    args = parser.parse_args()

    last_processed_dir = None
    log(f"VLM Worker started. Watching {args.base_dir}")

    while True:
        subdirs = [os.path.join(args.base_dir, d) for d in os.listdir(args.base_dir)
                   if os.path.isdir(os.path.join(args.base_dir, d)) and not d.endswith("_ann")]

        if not subdirs:
            time.sleep(5)
            continue

        latest_dir = max(subdirs, key=os.path.getmtime)
        folder_mtime = os.path.getmtime(latest_dir)
        time_since_creation = time.time() - folder_mtime

        # Check if we should process this folder or wait for activity
        if latest_dir != last_processed_dir or time_since_creation < args.wait_threshold:
            log(f"Processing folder: {os.path.basename(latest_dir)}")
            process_folder(latest_dir, args.vlm_url, args.notify_url, args.frame_delay)
            last_processed_dir = latest_dir
        else:
            log(f"Idle. No new activity in >{args.wait_threshold}s.")
            time.sleep(5)


if __name__ == "__main__":
    main()