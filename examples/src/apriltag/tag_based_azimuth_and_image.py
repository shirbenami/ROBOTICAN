#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo

import tf2_ros
from tf2_ros import TransformException

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class TagBasedAzimuth(Node):
    """
    Computes the continuous camera azimuth (yaw) relative to a world frame (0-360 degrees)
    by leveraging the known absolute azimuth of AprilTags placed on the walls.

    It selects the most reliable tag by finding the one closest to the center of the camera frame.

    Additionally, it keeps a time-stamped history of recent azimuth estimates and exposes
    a service `get_azimuth_at_time` that returns the azimuth closest to a requested timestamp.

    NEW:
    - Subscribes to the same camera image topic as apriltag_node.
    - Captures exactly 4 images around 0/90/180/270 (±5 degrees).
    - On each capture, publishes BOTH azimuth and the image.
    """

    def __init__(self):
        super().__init__("tag_based_azimuth")

        # --- Configuration Parameters ---
        self.declare_parameter("robot_ns", "R1")    # "R1" / "R2" / "R3"
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("camera_frame", "")

        # NEW: listen to same topics as apriltag_node (remappable)
        self.declare_parameter("image_topic", "")        # default: /<robot_ns>/camera/image_raw
        self.declare_parameter("camera_info_topic", "")  # default: /<robot_ns>/camera/camera_info

        # NEW: capture config
        self.declare_parameter("tolerance_deg", 30.0)

        self.robot_ns = (
            self.get_parameter("robot_ns")
            .get_parameter_value()
            .string_value
            .strip()
            .strip("/")
        )

        self.tag_family = (
            self.get_parameter("tag_family")
            .get_parameter_value()
            .string_value
        )

        self.camera_frame = (
            self.get_parameter("camera_frame")
            .get_parameter_value()
            .string_value
            .strip()
        )

        self.image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
            .strip()
        )

        self.camera_info_topic = (
            self.get_parameter("camera_info_topic")
            .get_parameter_value()
            .string_value
            .strip()
        )

        self.tolerance_deg = float(
            self.get_parameter("tolerance_deg").get_parameter_value().double_value
        )

        # If camera_frame is not provided, derive it from robot_ns
        if not self.camera_frame:
            self.camera_frame = f"{self.robot_ns}_camera"

        # Clean up camera frame name (remove leading "/")
        if self.camera_frame.startswith("/"):
            self.camera_frame = self.camera_frame[1:]

        # Default topics if not provided
        if not self.image_topic:
            self.image_topic = f"/{self.robot_ns}/camera/image_raw"
        if not self.camera_info_topic:
            self.camera_info_topic = f"/{self.robot_ns}/camera/camera_info"

        # --- Known Tag Configuration ---
        self.tag_config = {
            15: 0.0,    # North
            15: 90.0,   # East
            15: 180.0,  # South
            15: 270.0,  # West
        }

        self.known_tag_ids = list(self.tag_config.keys())

        # --- TF Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publisher ---
        azimuth_topic = f"/{self.robot_ns}/camera_azimuth"
        self.azimuth_pub = self.create_publisher(Float32, azimuth_topic, 10)

        # NEW: publish selected/captured image
        self.image_pub = self.create_publisher(Image, f"/{self.robot_ns}/camera_image_used", 10)

        # --- Subscriptions (image + camera_info) ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.last_image = None
        self.last_image_stamp = None

        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, sensor_qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, sensor_qos)

        # --- Azimuth History Buffer ---
        self.azimuth_history = deque(maxlen=20)

        self.max_time_diff_sec = 1.0

        # --- Capture state (exactly 4 targets) ---
        self.targets_deg = [0.0, 90.0, 180.0, 270.0]
        self.captured = {t: False for t in self.targets_deg}

        # Edge trigger so we don't capture repeatedly while staying in the window
        self.was_in_window = {t: False for t in self.targets_deg}

        # --- Timer (5 Hz) ---
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(f"TagBasedAzimuth Node Started. Camera Frame: '{self.camera_frame}'")
        self.get_logger().info(f"Image topic: '{self.image_topic}' | CameraInfo topic: '{self.camera_info_topic}'")
        self.get_logger().info(f"Tracking Tag IDs: {self.known_tag_ids}")
        self.get_logger().info(f"Capture targets: {self.targets_deg} with tolerance ±{self.tolerance_deg} deg")
        self.get_logger().info("Service 'get_azimuth_at_time' is ready (currently commented).")

    # -------------------------------------------------------------------------
    # Subscribers
    # -------------------------------------------------------------------------
    def on_image(self, msg: Image):
        self.last_image = msg
        self.last_image_stamp = Time.from_msg(msg.header.stamp)

    def on_camera_info(self, msg: CameraInfo):
        # Not used, but subscribed to match apriltag_node inputs
        pass

    # -------------------------------------------------------------------------
    # Angle helpers
    # -------------------------------------------------------------------------
    def angle_diff_deg(self, a: float, b: float) -> float:
        """
        Minimal signed difference a-b in degrees, in range [-180, 180).
        """
        return (a - b + 180.0) % 360.0 - 180.0

    def is_in_window(self, yaw_deg: float, target_deg: float) -> bool:
        return abs(self.angle_diff_deg(yaw_deg, target_deg)) <= self.tolerance_deg

    def all_captured(self) -> bool:
        return all(self.captured.values())

    # -------------------------------------------------------------------------
    # Core azimuth computation
    # -------------------------------------------------------------------------
    def get_camera_yaw(self):
        """
        Same API/name as your original code.

        NEW:
        - If we have an image stamp, we try lookup_transform using that stamp
          to align azimuth to the latest frame.
        - If no image received yet, falls back to latest TF (Time()).
        """
        last_error = None
        best_tag_yaw_deg = None
        best_tag_id = None

        min_abs_relative_yaw = float("inf")

        # Use image stamp if available; otherwise fallback to "latest"
        query_time = self.last_image_stamp if self.last_image_stamp is not None else rclpy.time.Time()

        for tid in self.known_tag_ids:
            # SAME candidate frame naming as your old code (no changes)
            candidate_frames = [
                f"tag{self.tag_family}:{tid}",
                f"tag_{tid}",
                f"tag{tid}",
            ]

            transform = None

            for tag_frame in candidate_frames:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.camera_frame,  # target frame
                        tag_frame,          # source frame
                        query_time,
                    )
                    break
                except TransformException as e:
                    last_error = e
                    continue

            if transform:
                t = transform.transform.translation

                relative_yaw_rad = math.atan2(-t.x, t.z)
                relative_yaw_deg = math.degrees(relative_yaw_rad)

                abs_relative_yaw = abs(relative_yaw_deg)

                if abs_relative_yaw < min_abs_relative_yaw:
                    min_abs_relative_yaw = abs_relative_yaw

                    wall_azimuth_deg = self.tag_config[tid]

                    camera_yaw = wall_azimuth_deg + relative_yaw_deg
                    camera_yaw = camera_yaw % 360.0

                    best_tag_yaw_deg = camera_yaw
                    best_tag_id = tid

        if best_tag_yaw_deg is not None:
            return best_tag_yaw_deg, best_tag_id

        return None, last_error

    # -------------------------------------------------------------------------
    # Timer callback: compute yaw, publish, capture 4 images
    # -------------------------------------------------------------------------
    def timer_callback(self):
        """
        Same timer structure as your original code.

        NEW:
        - Captures exactly 4 images at yaw near 0/90/180/270 (±tolerance).
        - On capture publishes both azimuth + image.
        """
        if self.all_captured():
            # Optional: stop spamming logs
            self.get_logger().info("All 4 targets captured. (0/90/180/270)")
            return

        yaw_deg, info = self.get_camera_yaw()

        if yaw_deg is None:
            self.get_logger().warn(
                f"No tags visible. Last TF error: {info}",
                throttle_duration_sec=2.0,
            )
            return

        # Keep history (like before)
        now_time = self.get_clock().now()
        self.azimuth_history.append((now_time, yaw_deg))

        # Update per-target in-window state and capture on entry
        for target in self.targets_deg:
            in_win = self.is_in_window(yaw_deg, target)

            # Capture only on "entering" the window, and only if not captured yet
            if in_win and (not self.was_in_window[target]) and (not self.captured[target]):
                # Must have a real image to publish
                if self.last_image is None:
                    self.get_logger().warn("Azimuth in target window but no image received yet.")
                    break

                # Publish azimuth
                msg = Float32()
                msg.data = float(yaw_deg)
                self.azimuth_pub.publish(msg)

                # Publish image
                self.image_pub.publish(self.last_image)

                # Mark captured
                self.captured[target] = True

                remaining = sum(1 for v in self.captured.values() if not v)
                self.get_logger().info(
                    f"CAPTURED target={target:.0f}° | yaw={yaw_deg:.1f}° (Tag {info}) | remaining={remaining}"
                )

                # Only one capture per timer tick
                break

            self.was_in_window[target] = in_win

        # Optional regular log (less spam)
        self.get_logger().info(
            f"Azimuth: {yaw_deg:.1f}° (Based on Tag {info})",
            throttle_duration_sec=1.0,
        )

    # -------------------------------------------------------------------------
    # Service callback: unchanged (kept for compatibility)
    # -------------------------------------------------------------------------
    def handle_get_azimuth_at_time(self, request, response):
        if not self.azimuth_history:
            self.get_logger().warn(
                "GetAzimuthAtTime: history is empty, no azimuth samples available."
            )
            response.success = False
            response.azimuth = 0.0
            response.time_diff = 0.0
            return response

        req_time = Time.from_msg(request.stamp)

        def abs_time_diff_seconds(sample):
            sample_time, _ = sample
            dt = (sample_time - req_time).nanoseconds / 1e9
            return abs(dt)

        closest_sample = min(self.azimuth_history, key=abs_time_diff_seconds)
        closest_time, closest_yaw = closest_sample

        dt_sec = (closest_time - req_time).nanoseconds / 1e9
        abs_dt_sec = abs(dt_sec)

        if abs_dt_sec > self.max_time_diff_sec:
            self.get_logger().warn(
                f"GetAzimuthAtTime: no close sample found "
                f"(min diff={abs_dt_sec:.3f}s > {self.max_time_diff_sec:.3f}s)."
            )
            response.success = False
            response.azimuth = 0.0
            response.time_diff = float(abs_dt_sec)
            return response

        response.success = True
        response.azimuth = float(closest_yaw)
        response.time_diff = float(dt_sec)

        self.get_logger().info(
            f"GetAzimuthAtTime: requested_time={req_time.nanoseconds / 1e9:.3f}s, "
            f"matched_time={closest_time.nanoseconds / 1e9:.3f}s, "
            f"yaw={closest_yaw:.1f}°, dt={dt_sec:.3f}s"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TagBasedAzimuth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
