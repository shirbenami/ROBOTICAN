#!/usr/bin/env python3
import argparse
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from rooster_handler_interfaces.msg import KeepAlive
from fcu_driver_interfaces.msg import ManualControl
from std_srvs.srv import Trigger

from manual_core import ManualCommandModel  # uses same axis logic as GUI


class PathRunnerNode(Node):
    """
    Minimal path runner:

    - Publishes /<ROOSTER_ID>/manual_control at ~40 Hz during segments.
    - Publishes /<ROOSTER_ID>/keep_alive at ~1 Hz while running.
    - Reads path segments from a text file or from a hard-coded list.
    - While the path is running, periodically calls /<ROOSTER_ID>/capture_frame_one_shot
      via a timer (every capture_period seconds).
    """

    def __init__(
        self,
        rooster_id: str,
        flight_mode: int,
        turtle: bool = False,
        capture_period: float = 5.0,
    ):
        super().__init__("path_runner")

        self.rooster_id = rooster_id
        self.flight_mode = int(flight_mode)
        self.capture_period = float(capture_period)

        # Track when we should actually capture (only while path is running)
        self.capture_enabled = False

        # Use same core model as GUI, but no logging.
        self.command_model = ManualCommandModel(step=10.0, turtle_scale=0.5)
        if turtle:
            self.command_model.toggle_turtle()

        manual_topic = f"/{self.rooster_id}/manual_control"
        keep_alive_topic = f"/{self.rooster_id}/keep_alive"
        capture_service_name = f"/{self.rooster_id}/capture_frame_one_shot"

        self.manual_pub = self.create_publisher(ManualControl, manual_topic, 10)
        self.keep_alive_pub = self.create_publisher(KeepAlive, keep_alive_topic, 10)

        # Service client for image capture
        self.capture_client = self.create_client(Trigger, capture_service_name)

        # Timer to call capture service periodically (disabled logically by flag)
        self.capture_timer = self.create_timer(
            self.capture_period, self._capture_timer_cb
        )

        # Optional: to avoid log spam when service isn't ready
        self._last_capture_warn_time = 0.0
        self._capture_warn_interval = 5.0  # seconds

        self.get_logger().info(
            f"PathRunnerNode for {self.rooster_id} (flight_mode={self.flight_mode})"
        )
        self.get_logger().info(
            f"Publishing to: {manual_topic}, {keep_alive_topic}"
        )
        self.get_logger().info(
            f"Image capture service (timer): {capture_service_name}, period={self.capture_period}s"
        )

    # ---------- low-level send functions ----------

    def _send_keep_alive(self):
        msg = KeepAlive()
        msg.is_active = True
        msg.requested_flight_mode = self.flight_mode
        msg.command_reboot = False
        self.keep_alive_pub.publish(msg)

    def _send_manual(self):
        axes = self.command_model.get_scaled_axes()
        msg = ManualControl()
        msg.x = axes.x
        msg.y = axes.y
        msg.z = axes.z
        msg.r = axes.r
        msg.buttons = 0
        self.manual_pub.publish(msg)

    # ---------- image capture via timer ----------

    def _capture_timer_cb(self):
        """
        Timer callback that calls capture_frame_one_shot if capture is enabled.
        """
        if not self.capture_enabled:
            return

        if not self.capture_client.service_is_ready():
            now = time.time()
            if now - self._last_capture_warn_time > self._capture_warn_interval:
                self.get_logger().warn(
                    f"{self.rooster_id}: capture_frame_one_shot not ready yet."
                )
                self._last_capture_warn_time = now
            return

        req = Trigger.Request()
        future = self.capture_client.call_async(req)

        def _done_cb(fut: rclpy.client.Future):
            try:
                resp = fut.result()
                self.get_logger().info(
                    f"{self.rooster_id}: capture_frame_one_shot (timer) "
                    f"success={resp.success}, msg='{resp.message}'"
                )
            except Exception as e:
                self.get_logger().error(
                    f"{self.rooster_id}: capture_frame_one_shot (timer) failed: {e}"
                )

        future.add_done_callback(_done_cb)

    # ---------- high-level path execution ----------

    def run_path(
        self,
        segments: List[Tuple[str, float, float, float, float, float]],
        path_name: str = "path",
    ):
        """
        segments: list of (name, x, y, z, r, duration_sec)
        """
        if not segments:
            self.get_logger().warn("Empty path, nothing to do.")
            return

        self.get_logger().info(
            f"Starting path '{path_name}' with {len(segments)} segments."
        )

        # Enable capture while the path is running
        self.capture_enabled = True

        # Start with zero axes
        self.command_model.reset_axes()
        self._send_manual()
        self._send_keep_alive()

        last_keep_alive = time.time()
        last_manual = time.time()

        for name, x, y, z, r, dur in segments:
            if not rclpy.ok():
                break

            self.get_logger().info(
                f"Segment '{name}': x={x}, y={y}, z={z}, r={r}, dur={dur}s"
            )

            # Set axes for this segment
            self.command_model.set_axes(x, y, z, r)

            start = time.time()
            end = start + dur

            while time.time() < end and rclpy.ok():
                now = time.time()

                # ~40 Hz manual control
                if now - last_manual >= 1.0 / 40.0:
                    self._send_manual()
                    last_manual = now

                # ~1 Hz keep-alive
                if now - last_keep_alive >= 1.0:
                    self._send_keep_alive()
                    last_keep_alive = now

                # Let ROS process timers, service responses, etc.
                rclpy.spin_once(self, timeout_sec=0.01)

            # small pause between segments (optional)
            self.command_model.reset_axes()
            for _ in range(5):
                self._send_manual()
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.02)

        # Disable capture when done
        self.capture_enabled = False

        # Zero at the end
        self.get_logger().info("Path finished, zeroing axes.")
        self.command_model.reset_axes()
        for _ in range(20):
            self._send_manual()
            if time.time() - last_keep_alive > 1.0:
                self._send_keep_alive()
                last_keep_alive = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)


# ---------- helpers ----------

def parse_path_file(path_file: str):
    """
    Parse a text path file.

    Each non-empty, non-comment line can be:
        name x y z r duration
      or:
        x y z r duration   (name auto-generated)
    """
    segments = []
    auto_idx = 1

    with open(path_file, "r") as f:
        for idx, line in enumerate(f, start=1):
            raw = line.strip()
            if not raw or raw.startswith("#"):
                continue
            parts = raw.split()
            if len(parts) == 5:
                # x y z r dur
                name = f"S{auto_idx}"
                auto_idx += 1
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    r = float(parts[3])
                    dur = float(parts[4])
                except ValueError:
                    raise ValueError(f"Line {idx}: cannot parse numbers: {raw!r}")
            elif len(parts) == 6:
                # name x y z r dur
                name = parts[0]
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    r = float(parts[4])
                    dur = float(parts[5])
                except ValueError:
                    raise ValueError(f"Line {idx}: cannot parse numbers: {raw!r}")
            else:
                raise ValueError(
                    f"Line {idx}: expected 5 or 6 values, got {len(parts)}: {raw!r}"
                )

            if dur <= 0:
                raise ValueError(f"Line {idx}: duration must be > 0")

            segments.append((name, x, y, z, r, dur))

    if not segments:
        raise ValueError("No valid segments in path file")

    return segments


def main():
    parser = argparse.ArgumentParser(
        description="Run a manual-control path on a Rooster via ROS 2."
    )
    parser.add_argument(
        "--rooster-id", "-r", default="R2", help="Rooster ID (R1/R2/R3...)"
    )
    parser.add_argument(
        "--flight-mode",
        "-m",
        type=int,
        default=1,
        help="Requested flight mode in KeepAlive (e.g. 1=GROUND_ROLL, 2=MANUAL)",
    )
    parser.add_argument(
        "--path", "-p", required=True, help="Path file (text) with segments"
    )
    parser.add_argument(
        "--turtle", action="store_true", help="Enable turtle (slow) mode scaling"
    )
    parser.add_argument(
        "--capture-period",
        "-c",
        type=float,
        default=5.0,
        help="Seconds between capture_frame_one_shot calls while path is running.",
    )
    args = parser.parse_args()

    segments = parse_path_file(args.path)

    rclpy.init()
    node = PathRunnerNode(
        rooster_id=args.rooster_id,
        flight_mode=args.flight_mode,
        turtle=args.turtle,
        capture_period=args.capture_period,
    )
    try:
        node.run_path(segments, path_name=args.path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
