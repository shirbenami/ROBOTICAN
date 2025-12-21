#!/usr/bin/env python3
import os
import time
import queue, threading
import argparse
from typing import Optional

import requests
import json


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from concurrent.futures import ThreadPoolExecutor

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from message_filters import Subscriber, TimeSynchronizer

from fcu_driver_interfaces.msg import UAVState
from txt_and_image_utils import _update_sidecar_json


def _remap_path(local_path: str, src_root: Optional[str], dst_root: Optional[str]) -> str:
    if not src_root or not dst_root:
        return os.path.abspath(local_path)
    local_path = os.path.abspath(local_path)
    src_root = os.path.abspath(src_root)
    try:
        rel = os.path.relpath(local_path, src_root)
    except ValueError:
        return local_path
    return os.path.join(dst_root, rel)


def _call_vlm(endpoint: Optional[str], image_path_for_vlm: str,
              timeout: float, retries: int) -> Optional[str]:
    if not endpoint:
        return None
    payload = {"image_path": image_path_for_vlm}
    last_err = None
    for _ in range(max(1, retries)):
        try:
            r = requests.post(endpoint, json=payload, timeout=timeout)
            r.raise_for_status()
            try:
                data = r.json()
                if isinstance(data, dict):
                    return (
                            data.get("response")
                            or data.get("caption")
                            or json.dumps(data, ensure_ascii=False)
                    )
                return json.dumps(data, ensure_ascii=False)
            except ValueError:
                return r.text.strip()
        except Exception as e:
            last_err = e
            time.sleep(0.2)
    print(f"[vlm] WARN: describe failed for {image_path_for_vlm}: {last_err}")
    return None


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ImageStateBuffer(Node):
    """
    Node layout:

    - When idle (default): no subscription to image/state -> no image traffic
      from the other machine.

    """

    def __init__(self, args):
        super().__init__("image_state_buffer")

        self.out_dir = None
        self.drone_id = args.drone_id
        self.pose_mode = getattr(args, "pose_mode", "state")
        self.base_dir = os.path.abspath(args.out_dir)

        self.bridge = CvBridge()
        self.reentrant_cb_group = ReentrantCallbackGroup()

        # VLM config (optional)
        self.vlm_endpoint: Optional[str] = args.vlm
        self.vlm_timeout: float = args.vlm_timeout
        self.vlm_retries: int = args.vlm_retries
        self.vlm_path_src: Optional[str] = args.vlm_path_src
        self.vlm_path_dst: Optional[str] = args.vlm_path_dst

        # Topics
        self.image_topic_name =  f"/{self.drone_id}/selected_frame"

        self.state_topic_name = f"/{self.drone_id}/fcu/state"

        # Latest messages
        self.last_img_msg: Optional[Image] = None
        self.last_state_msg: Optional[UAVState] = None
        self._last_img_stamp = None  # tuple(sec, nanosec)
        self._same_frame_hits = 0
        self._freeze_warn_every = 30  # log every N repeats

        self.executor_pool = ThreadPoolExecutor(max_workers=2)
        self._vlm_q = queue.Queue(maxsize=1)
        self._vlm_stop = threading.Event()
        self._vlm_thread = threading.Thread(target=self._vlm_worker_loop, daemon=True)
        self._vlm_thread.start()

        # Approx sync tolerance (seconds)
        self.sync_slop = 0.10  # 100 ms

        # Subscriptions (created/destroyed on demand)
        self.image_sub = self.create_subscription(
            Image, self.image_topic_name, self.image_cb, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(
            UAVState, self.state_topic_name, self.state_cb, 10)
        self.get_logger().info(
            f"ImageStateBuffer started for {self.drone_id}\n"
            f"  image topic: {self.image_topic_name}\n"
            f"  state topic: {self.state_topic_name}\n"
            f"  base_dir:     {self.base_dir}\n"
            f"  VLM:         {self.vlm_endpoint or '(disabled)'}\n"
            f"  capture:     initially OFF (no image/state subscriptions)"
        )

    # ------------------------------------------------------------------
    # Sub callbacks: keep latest messages in memory
    # ------------------------------------------------------------------
    def state_cb(self, state_msg: UAVState):
        # Only keep the latest message
        self.last_state_msg = state_msg
        # self.get_logger().info("State msg received (buffered)") # Removed verbose logging


    # ------------------------------------------------------------------
    # Helper: extract pose dict from UAVState
    # ------------------------------------------------------------------
    @staticmethod
    def extract_pose_from_state(state_msg: UAVState) -> dict:
        """
        Best-effort extraction of {x,y,z,yaw} from UAVState.

        Adjust this mapping to match your exact UAVState definition.
        """
        pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

        # Position-like fields
        if hasattr(state_msg, "position"):
            p = state_msg.position
            pose["x"] = p.x  # float(getattr(p, "x", 0.0))
            pose["y"] = p.y  # float(getattr(p, "y", 0.0))
            pose["z"] = p.z  # float(getattr(p, "z", 0.0))

        # Heading-like field
        if hasattr(state_msg, "azimuth"):
            pose["yaw"] = float(state_msg.azimuth)

        for k, v in pose.items():
            pose[k] = round(float(v), 5)

        return pose

    @staticmethod
    def parse_frame_id( frame_id: str):
        """
        Expected format:
          "<out_dir>_____ <azimuth>"
        Example:
          "2025_12_21___15_30_49_____95.18695746993248"
        Returns: (out_dir: str, azimuth: float) or (frame_id, None) if not parseable
        """
        if not frame_id:
            return "", None

        sep = "_____"
        if sep not in frame_id:
            return frame_id, None

        left, right = frame_id.split(sep, 1)
        out_dir = left.strip()
        az_str = right.strip()

        try:
            az = float(az_str)
        except ValueError:
            az = None

        return out_dir, az

    def _vlm_job(self, img_for_vlm: str, json_path: str, pose: dict, img_basename: str):
        try:
            self.get_logger().info(f"[vlm] POST {self.vlm_endpoint} image_path={img_for_vlm}")
            caption = _call_vlm(
                self.vlm_endpoint,
                img_for_vlm,
                timeout=self.vlm_timeout,
                retries=self.vlm_retries,
            )
            _update_sidecar_json(
                json_path,
                pose,
                img_basename,
                vlm_text=caption,
            )
            if caption:
                short = caption[:120] + ("…" if len(caption) > 120 else "")
                self.get_logger().info(f"[vlm] done: {os.path.basename(json_path)} caption: {short}")
            else:
                self.get_logger().warn(f"[vlm] done: {os.path.basename(json_path)} no caption")
        except Exception as e:
            self.get_logger().error(f"[vlm] job failed for {json_path}: {e}")

    def _vlm_worker_loop(self):
        while not self._vlm_stop.is_set():
            try:
                job = self._vlm_q.get(timeout=0.2)
            except Exception:
                continue

            img_for_vlm, json_path, pose, img_basename = job
            try:
                t0 = time.time()
                self.get_logger().info(f"[vlm] POST {self.vlm_endpoint}  image_path={img_for_vlm}")
                caption = _call_vlm(self.vlm_endpoint, img_for_vlm, timeout=self.vlm_timeout, retries=self.vlm_retries)
                self.get_logger().warn(f"[vlm] took {time.time() - t0:.2f}s")

                _update_sidecar_json(json_path, pose, img_basename, vlm_text=caption)
                if not caption:
                    self.get_logger().warn("[vlm] no caption returned")
            except Exception as e:
                self.get_logger().error(f"[vlm] worker failed: {e}")
            finally:
                try:
                    self._vlm_q.task_done()
                except Exception:
                    pass


    # ------------------------------------------------------------------
    # Capture service: save one JPG+JSON (and VLM) from latest messages
    # ------------------------------------------------------------------
    def image_cb(self, img_msg: Image):
        try:

            # Check approximate sync
            t_img = _stamp_to_sec(img_msg.header.stamp)
            t_state = _stamp_to_sec(self.last_state_msg.header.stamp)
            stamp_tup = (img_msg.header.stamp.sec, img_msg.header.stamp.nanosec)

            if stamp_tup == self._last_img_stamp:
                self._same_frame_hits += 1
                if (self._same_frame_hits % self._freeze_warn_every) == 0:
                    self.get_logger().warn(
                        f"Frozen/repeated frame detected (same stamp) hits={self._same_frame_hits}. Skipping.")
                return

            dt = abs(t_img - t_state)
            if dt > self.sync_slop:
                # self.get_logger().info(
                #     f"Capture with unsynced pair: |t_img - t_state| = {dt:.3f}s "
                #     f"(slop={self.sync_slop:.3f}s)"
                # )
                pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
            else:
                state_msg = self.last_state_msg
                pose = ImageStateBuffer.extract_pose_from_state(state_msg)

            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")



        except Exception as e:
            msg = f"Failed to convert image: {e}"
            self.get_logger().error(msg)
            return
        unique_out_dir, azimuth = ImageStateBuffer.parse_frame_id(img_msg.header.frame_id)
        # Optionally override yaw with AprilTag azimuth if available
        if azimuth is not None:
            pose["yaw"] = round(azimuth, 5)

        # Use image ROS time for filename
        stamp = img_msg.header.stamp
        t_sec = _stamp_to_sec(stamp)
        ts_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(t_sec))
        base_name = f"{self.drone_id}_{ts_str}"
        self.out_dir = os.path.join(self.base_dir, str(unique_out_dir))
        jpg_path = os.path.join(self.out_dir, base_name + ".jpg")
        json_path = os.path.join(self.out_dir, base_name + ".json")
        img_basename = os.path.basename(jpg_path)

        os.makedirs(self.out_dir, exist_ok=True)
        import cv2
        try:
            # txt = f"t={stamp.sec}.{stamp.nanosec:09d}"
            # cv2.putText(cv_img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imwrite(jpg_path, cv_img)
        except Exception as e:
            msg = f"Failed to save image {jpg_path}: {e}"
            self.get_logger().error(msg)

        # First create/update JSON with pose and image name, no VLM yet
        _update_sidecar_json(json_path, pose, img_basename, vlm_text=None)
        self._last_img_stamp = stamp_tup
        self._same_frame_hits = 0
        if self.vlm_endpoint:
            img_for_vlm = _remap_path(jpg_path, self.vlm_path_src or None, self.vlm_path_dst or None)
            job = (img_for_vlm, json_path, dict(pose), img_basename)

            # keep only the latest job
            try:
                self._vlm_q.put_nowait(job)
            except queue.Full:
                try:
                    _ = self._vlm_q.get_nowait()  # drop the older pending job
                except queue.Empty:
                    pass
                self._vlm_q.put_nowait(job)

            self.get_logger().info(
                f"[vlm] POST {self.vlm_endpoint}  image_path={img_for_vlm}"
            )

    # ------------------------------------------------------------------
    # Cleanup (Subscriptions are destroyed in destroy_node, which is fine)
    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
        if self.state_sub is not None:
            self.destroy_subscription(self.state_sub)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description="On-demand capture: /<id>/capture_frame using continuous subscriptions."
    )
    parser.add_argument(
        "--drone-id",
        default="R2",
        help="Drone ID prefix (topics: /<id>/camera/image_raw, /<id>/fcu/state)",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        default="/tmp/captures/",
        help="Output directory for jpg+json pairs",
    )

    parser.add_argument(
        "--vlm",
        default="http://172.16.17.12:8080/describe",
        help="VLM HTTP endpoint (e.g. http://host:port/describe). If not set, no VLM call.",
    )
    parser.add_argument(
        "--vlm-timeout",
        type=float,
        default=10.0,
        help="VLM HTTP timeout in seconds",
    )
    parser.add_argument(
        "--vlm-retries",
        type=int,
        default=3,
        help="Number of retries for VLM calls",
    )
    parser.add_argument(
        "--vlm-path-src",
        default="/mnt/VLM/jetson-data",
        help="Optional local path root to remap FROM when sending image_path to VLM",
    )
    parser.add_argument(
        "--vlm-path-dst",
        default="/mnt/VLM/jetson-data",
        help="Optional path root to remap TO for VLM",
    )

    parser.add_argument(
        "--pose-mode",
        choices=["state", "april", "both"],
        default="state",
        help=(
            "How to compute pose for JSON: "
            "'state' = from /fcu/state (current behavior), "
            "'april' = from AprilTag-based function, "
            "'both' = merge AprilTag pose into state pose."
        ),
    )

    args = parser.parse_args()

    rclpy.init()
    node = ImageStateBuffer(args)

    # Use MultiThreadedExecutor, although SingleThreadedExecutor would also work now.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()