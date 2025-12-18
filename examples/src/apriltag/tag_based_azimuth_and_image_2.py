#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo

import tf2_ros
from tf2_ros import TransformException

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class TagBasedAzimuth(Node):
    """
    Snapshot-friendly azimuth estimation.

    Key changes vs. the timer-based version:
    1) Computes azimuth ONLY when a new Image arrives (event-based).
    2) TF lookup uses the Image header stamp (Time.from_msg(msg.header.stamp)),
       so azimuth matches the specific frame (critical for sparse snapshots).
    3) Publishes:
       - /<robot_ns>/camera_azimuth (Float32)
       - /<robot_ns>/camera_image_used (Image)  [same behavior you had]
    """

    def __init__(self):
        super().__init__("tag_based_azimuth_snapshot")

        # ---------------- Parameters ----------------
        self.declare_parameter("robot_ns", "R1")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("camera_frame", "")

        # Remappable, but defaults match your previous behavior
        self.declare_parameter("image_topic", "")        # default: /<robot_ns>/camera/image_raw
        self.declare_parameter("camera_info_topic", "")  # default: /<robot_ns>/camera/camera_info

        # TF lookup behavior
        self.declare_parameter("tf_timeout_sec", 0.15)        # how long to wait for TF
        self.declare_parameter("allow_latest_fallback", True) # if stamped lookup fails, fallback to latest TF

        # Optional: capture windows (keep if you want the “4 headings” behavior)
        self.declare_parameter("enable_targets_capture", False)
        self.declare_parameter("tolerance_deg", 30.0)

        self.robot_ns = (
            self.get_parameter("robot_ns")
            .get_parameter_value()
            .string_value
            .strip()
            .strip("/")
        )

        self.tag_family = self.get_parameter("tag_family").get_parameter_value().string_value

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value.strip()
        if not self.camera_frame:
            self.camera_frame = f"{self.robot_ns}_camera"
        if self.camera_frame.startswith("/"):
            self.camera_frame = self.camera_frame[1:]

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value.strip()
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value.strip()

        if not self.image_topic:
            self.image_topic = f"/{self.robot_ns}/camera/image_raw"
        if not self.camera_info_topic:
            self.camera_info_topic = f"/{self.robot_ns}/camera/camera_info"

        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))
        self.allow_latest_fallback = bool(self.get_parameter("allow_latest_fallback").value)

        self.enable_targets_capture = bool(self.get_parameter("enable_targets_capture").value)
        self.tolerance_deg = float(self.get_parameter("tolerance_deg").value)

        # ---------------- Tag config ----------------
        # IMPORTANT: must be unique tag IDs. Example mapping:
        # 15=N, 16=E, 17=S, 18=W (change to your real IDs!)
        self.tag_config = {
            10: 0.0,    # North wall tag
            11: 90.0,   # East wall tag
            12: 180.0,  # South wall tag
            13: 270.0,  # West wall tag
        }
        self.known_tag_ids = list(self.tag_config.keys())

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Publishers ----------------
        self.azimuth_pub = self.create_publisher(Float32, f"/{self.robot_ns}/camera_azimuth", 10)
        self.image_pub = self.create_publisher(Image, f"/{self.robot_ns}/camera_image_used", 10)

        # ---------------- Subscriptions ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, sensor_qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, sensor_qos)

        # ---------------- Optional capture state ----------------
        self.targets_deg = [0.0, 90.0, 180.0, 270.0]
        self.captured = {t: False for t in self.targets_deg}

        self.get_logger().info(f"TagBasedAzimuth(snapshot) started.")
        self.get_logger().info(f"camera_frame='{self.camera_frame}'")
        self.get_logger().info(f"image_topic='{self.image_topic}' | camera_info_topic='{self.camera_info_topic}'")
        self.get_logger().info(f"known_tag_ids={self.known_tag_ids}")
        self.get_logger().info(f"TF timeout={self.tf_timeout.nanoseconds/1e9:.3f}s | latest_fallback={self.allow_latest_fallback}")
        self.get_logger().info(f"targets_capture={self.enable_targets_capture} | tol=±{self.tolerance_deg}°")

    def on_camera_info(self, msg: CameraInfo):
        # Keep subscription for compatibility with your pipeline; not required for azimuth math here.
        pass

    # ---------------- Angle helpers ----------------
    @staticmethod
    def angle_diff_deg(a: float, b: float) -> float:
        """Minimal signed difference a-b in degrees, range [-180, 180)."""
        return (a - b + 180.0) % 360.0 - 180.0

    def is_in_window(self, yaw_deg: float, target_deg: float) -> bool:
        return abs(self.angle_diff_deg(yaw_deg, target_deg)) <= self.tolerance_deg

    # ---------------- Core azimuth computation ----------------
    def get_camera_yaw_for_stamp(self, stamp_time: Time):
        """
        Try to compute yaw using TF at the given timestamp.
        Returns: (yaw_deg, best_tag_id, used_time_mode)
          - used_time_mode: "stamped" or "latest"
        """
        last_error = None
        best_yaw = None
        best_tag_id = None
        min_abs_relative_yaw = float("inf")

        def try_lookup(tag_frame: str, query_time: Time):
            return self.tf_buffer.lookup_transform(
                self.camera_frame,  # target
                tag_frame,          # source
                query_time,
                timeout=self.tf_timeout,
            )

        # 1) Stamped lookup first
        query_time = stamp_time
        used_mode = "stamped"

        for tid in self.known_tag_ids:
            candidate_frames = [
                f"tag{self.tag_family}:{tid}",
                f"tag_{tid}",
                f"tag{tid}",
            ]

            transform = None
            for tag_frame in candidate_frames:
                try:
                    transform = try_lookup(tag_frame, query_time)
                    break
                except TransformException as e:
                    last_error = e
                    continue

            # Optional fallback to latest if stamped failed
            if (transform is None) and self.allow_latest_fallback:
                used_mode = "latest"
                for tag_frame in candidate_frames:
                    try:
                        transform = try_lookup(tag_frame, Time())  # latest
                        break
                    except TransformException as e:
                        last_error = e
                        continue

            if transform is None:
                continue

            t = transform.transform.translation

            # Same geometry you had
            relative_yaw_rad = math.atan2(-t.x, t.z)
            relative_yaw_deg = math.degrees(relative_yaw_rad)

            abs_relative_yaw = abs(relative_yaw_deg)
            if abs_relative_yaw < min_abs_relative_yaw:
                min_abs_relative_yaw = abs_relative_yaw

                wall_azimuth_deg = self.tag_config[tid]
                camera_yaw = (wall_azimuth_deg + relative_yaw_deg) % 360.0

                best_yaw = camera_yaw
                best_tag_id = tid

        if best_yaw is not None:
            return best_yaw, best_tag_id, used_mode

        return None, last_error, None

    # ---------------- Event-based callback ----------------
    def on_image(self, msg: Image):
        """
        Called once per snapshot image. Computes yaw for this frame and publishes immediately.
        """
        stamp_time = Time.from_msg(msg.header.stamp)

        yaw_deg, info, used_mode = self.get_camera_yaw_for_stamp(stamp_time)

        if yaw_deg is None:
            self.get_logger().warn(
                f"[snapshot] No tags visible for image stamp={stamp_time.nanoseconds/1e9:.3f}. Last TF error: {info}",
                throttle_duration_sec=2.0,
            )
            return

        # Publish azimuth for THIS image
        az = Float32()
        az.data = float(yaw_deg)
        self.azimuth_pub.publish(az)

        # Publish the same image on camera_image_used (exactly like your pipeline expects)
        self.image_pub.publish(msg)

        # Optional: capture only certain headings (if you enable it)
        if self.enable_targets_capture:
            for target in self.targets_deg:
                if (not self.captured[target]) and self.is_in_window(yaw_deg, target):
                    self.captured[target] = True
                    remaining = sum(1 for v in self.captured.values() if not v)
                    self.get_logger().info(
                        f"[CAPTURE] target={target:.0f}° | yaw={yaw_deg:.1f}° | tag={info} | mode={used_mode} | remaining={remaining}"
                    )
                    break
