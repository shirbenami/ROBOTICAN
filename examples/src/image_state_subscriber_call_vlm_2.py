#!/usr/bin/env python3
import os
import time
import argparse
from logging import exception
from typing import Optional
import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from fcu_driver_interfaces.msg import UAVState
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
from txt_and_image_utils import _update_sidecar_json

import requests
import json


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

    - Service: /<drone_id>/start_capture (Trigger)
        * Creates subscriptions to:
          /<drone_id>/camera/image_raw
          /<drone_id>/fcu/state
        * Starts buffering the latest messages.

    - Service: /<drone_id>/stop_capture (Trigger)
        * Destroys those subscriptions, stops the traffic.

    - Service: /<drone_id>/capture_frame (Trigger)
        * Uses the latest image + state in memory.
        * Saves <id>_YYYYmmdd_HHMMSS.jpg + .json
        * Optionally calls VLM and stores caption in json.
    """

    def __init__(self, args):
        super().__init__("image_state_buffer")

        self.drone_id = args.drone_id
        out_dir = os.path.abspath(args.out_dir)
        unique_out_dir = datetime.datetime.now().strftime("%Y_%m_%d___%H_%M_%S")
        self.out_dir = os.path.join(out_dir, unique_out_dir)
        try:
            print(f"out_dir: {self.out_dir}")
            os.makedirs(self.out_dir, exist_ok=True)
        except Exception as exp:
            self.logger.error(exp)

        self.bridge = CvBridge()

        # VLM config (optional)
        self.vlm_endpoint: Optional[str] = args.vlm
        self.vlm_timeout: float = args.vlm_timeout
        self.vlm_retries: int = args.vlm_retries
        self.vlm_path_src: Optional[str] = args.vlm_path_src
        self.vlm_path_dst: Optional[str] = args.vlm_path_dst

        # Topics
        self.image_topic = f"/{self.drone_id}/camera/image_raw"
        self.state_topic = f"/{self.drone_id}/fcu/state"

        # Latest messages
        self.last_img_msg: Optional[Image] = None
        self.last_state_msg: Optional[UAVState] = None

        # Approx sync tolerance (seconds)
        self.sync_slop = 0.10  # 100 ms

        # Subscriptions (created/destroyed on demand)
        self.image_sub = None
        self.state_sub = None
        self.capture_enabled = False
        try:
            # Services
            self.start_capture_srv = self.create_service(
                Trigger,
                f"/{self.drone_id}/start_capture",
                self.handle_start_capture,
            )
            self.stop_capture_srv = self.create_service(
                Trigger,
                f"/{self.drone_id}/stop_capture",
                self.handle_stop_capture,
            )
            self.capture_srv = self.create_service(
                Trigger,
                f"/{self.drone_id}/capture_frame",
                self.handle_capture,
            )
        except Exception as exp:
            self.get_logger().error(exp)
            raise exp

        self.get_logger().info(
            f"ImageStateBuffer started for {self.drone_id}\n"
            f"  image topic: {self.image_topic}\n"
            f"  state topic: {self.state_topic}\n"
            f"  out_dir:     {self.out_dir}\n"
            f"  VLM:         {self.vlm_endpoint or '(disabled)'}\n"
            f"  capture:     initially OFF (no image/state subscriptions)"
        )

    # ------------------------------------------------------------------
    # Start/stop capture: create/destroy subscriptions
    # ------------------------------------------------------------------

    def handle_start_capture(self, request: Trigger.Request, context) -> Trigger.Response:
        resp = Trigger.Response()

        if self.capture_enabled:
            resp.success = True
            resp.message = "Capture already enabled"
            return resp

        # Reset buffers
        self.last_img_msg = None
        self.last_state_msg = None

        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_cb, 10
        )
        self.state_sub = self.create_subscription(
            UAVState, self.state_topic, self.state_cb, 10
        )

        self.capture_enabled = True
        msg = "Capture enabled: image/state subscriptions created"
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def handle_stop_capture(self, request: Trigger.Request, context) -> Trigger.Response:
        resp = Trigger.Response()

        if not self.capture_enabled:
            resp.success = True
            resp.message = "Capture already disabled"
            return resp

        # Destroy subscriptions (this is what really stops DDS traffic)
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.state_sub is not None:
            self.destroy_subscription(self.state_sub)
            self.state_sub = None

        self.capture_enabled = False

        # Optionally keep last_* or clear them; here we keep them
        msg = "Capture disabled: image/state subscriptions destroyed"
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp


    # ------------------------------------------------------------------
    # Sub callbacks: keep latest messages in memory
    # ------------------------------------------------------------------

    def image_cb(self, img_msg: Image):
        self.last_img_msg = img_msg
        msg = "New frame received"
        self.get_logger().info(msg)

    def state_cb(self, state_msg: UAVState):
        self.last_state_msg = state_msg
        msg = "State msg received"
        self.get_logger().info(msg)

    # ------------------------------------------------------------------
    # Helper: extract pose dict from UAVState
    # ------------------------------------------------------------------
    def extract_pose_from_state(self, state_msg: UAVState) -> dict:
        """
        Best-effort extraction of {x,y,z,yaw} from UAVState.

        Adjust this mapping to match your exact UAVState definition.
        """
        pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

        # Position-like fields
        if hasattr(state_msg, "position"):
            p = state_msg.position
            pose["x"] = float(getattr(p, "x", 0.0))
            pose["y"] = float(getattr(p, "y", 0.0))
            pose["z"] = float(getattr(p, "z", 0.0))
        elif hasattr(state_msg, "location"):
            p = state_msg.location
            pose["x"] = float(getattr(p, "x", 0.0))
            pose["y"] = float(getattr(p, "y", 0.0))
            pose["z"] = float(getattr(p, "z", 0.0))

        # Heading-like field
        if hasattr(state_msg, "yaw"):
            pose["yaw"] = float(state_msg.yaw)
        elif hasattr(state_msg, "heading"):
            pose["yaw"] = float(state_msg.heading)
        elif hasattr(state_msg, "azimuth"):
            pose["yaw"] = float(state_msg.azimuth)

        # Round to 5 digits like you asked
        for k, v in pose.items():
            pose[k] = round(float(v), 5)

        return pose

    # ------------------------------------------------------------------
    # Capture service: save one JPG+JSON (and VLM) from latest messages
    # ------------------------------------------------------------------
    def handle_capture(self, request: Trigger.Request, context) -> Trigger.Response:
        resp = Trigger.Response()

        if not self.capture_enabled:
            resp.success = False
            resp.message = "Capture is disabled; call start_capture first"
            return resp

        if self.last_img_msg is None or self.last_state_msg is None:
            resp.success = False
            resp.message = "No image/state received yet"
            return resp

        img_msg = self.last_img_msg
        state_msg = self.last_state_msg

        # Check approximate sync
        t_img = _stamp_to_sec(img_msg.header.stamp)
        t_state = _stamp_to_sec(state_msg.header.stamp)
        dt = abs(t_img - t_state)
        if dt > self.sync_slop:
            self.get_logger().warn(
                f"Capture with unsynced pair: |t_img - t_state| = {dt:.3f}s "
                f"(slop={self.sync_slop:.3f}s)"
            )

        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            msg = f"Failed to convert image: {e}"
            self.get_logger().error(msg)
            resp.success = False
            resp.message = msg
            return resp

        pose = self.extract_pose_from_state(state_msg)

        # Use image ROS time for filename
        stamp = img_msg.header.stamp
        t_sec = _stamp_to_sec(stamp)
        ts_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(t_sec))
        base_name = f"{self.drone_id}_{ts_str}"

        jpg_path = os.path.join(self.out_dir, base_name + ".jpg")
        json_path = os.path.join(self.out_dir, base_name + ".json")
        img_basename = os.path.basename(jpg_path)

        os.makedirs(self.out_dir, exist_ok=True)
        import cv2
        try:
            cv2.imwrite(jpg_path, cv_img)
        except Exception as e:
            msg = f"Failed to save image {jpg_path}: {e}"
            self.get_logger().error(msg)
            resp.success = False
            resp.message = msg
            return resp

        # First create/update JSON with pose and image name, no VLM yet
        _update_sidecar_json(json_path, pose, img_basename, vlm_text=None)

        vlm_caption = None
        if self.vlm_endpoint:
            # Remap path if needed for the VLM server
            img_for_vlm = _remap_path(
                jpg_path,
                self.vlm_path_src or None,
                self.vlm_path_dst or None,
            )
            self.get_logger().info(
                f"[vlm] POST {self.vlm_endpoint}  image_path={img_for_vlm}"
            )
            vlm_caption = _call_vlm(
                self.vlm_endpoint,
                img_for_vlm,
                timeout=self.vlm_timeout,
                retries=self.vlm_retries,
            )
            if vlm_caption:
                short = vlm_caption[:120] + ("…" if len(vlm_caption) > 120 else "")
                self.get_logger().info(f"[vlm] caption: {short}")
            else:
                self.get_logger().warn("[vlm] no caption returned")

            _update_sidecar_json(
                json_path,
                pose,
                img_basename,
                vlm_text=vlm_caption,
            )

        msg = f"Captured {img_basename} with pose={pose}  dt={dt:.3f}s"
        if vlm_caption:
            msg += " (with VLM caption)"
        self.get_logger().info(msg)

        resp.success = True
        resp.message = msg
        return resp

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
        if self.state_sub is not None:
            self.destroy_subscription(self.state_sub)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description="On-demand capture: /<id>/start_capture, /<id>/stop_capture, /<id>/capture_frame."
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

    args = parser.parse_args()

    rclpy.init()
    node = ImageStateBuffer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
