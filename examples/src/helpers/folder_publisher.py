#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from builtin_interfaces.msg import Time

from typing import Iterable, Optional

class FolderImagePublisher(Node):
    def __init__(self):
        super().__init__("folder_image_publisher")
        self.id = "R2"
        # Params
        self.declare_parameter("folder", "")
        self.declare_parameter("topic", f"/{self.id}/camera/image_raw")
        self.declare_parameter("fps", 1.0)          # 1 frame per second
        self.declare_parameter("loop", False)

        folder = self.get_parameter("folder").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        fps = float(self.get_parameter("fps").get_parameter_value().double_value)
        self.loop = bool(self.get_parameter("loop").get_parameter_value().bool_value)

        if not folder:
            raise RuntimeError("Parameter 'folder' is empty. Provide --ros-args -p folder:=/path/to/images")

        self.image_paths = sorted(
            glob.glob(os.path.join(folder, "*.png")) +
            glob.glob(os.path.join(folder, "*.jpg")) +
            glob.glob(os.path.join(folder, "*.jpeg")) +
            glob.glob(os.path.join(folder, "*.bmp"))
        )

        if not self.image_paths:
            raise RuntimeError(f"No images found in folder: {folder}")

        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(CameraInfo,
                                                     f"/{self.id}/camera/camera_info", qos_profile_sensor_data)
        self.bridge = CvBridge()

        self.idx = 0

        period_s = 1.0 / max(fps, 1e-6)
        self.timer = self.create_timer(period_s, self.on_timer)

        self.get_logger().info(f"Publishing {len(self.image_paths)} images from '{folder}' to '{topic}' every {period_s:.3f}s")

    def on_timer(self):
        if self.idx >= len(self.image_paths):
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info("Done publishing all images (loop=false). Shutting down.")
                rclpy.shutdown()
                return

        path = self.image_paths[self.idx]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f"Failed to read image: {path} (skipping)")
            self.idx += 1
            return

        self.height = img.shape[0]
        self.width = img.shape[1]
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        ci = self.make_camera_info(
            frame_id=f"{self.id}_camera"
        )
        ci.header.stamp = msg.header.stamp
        ci.header.frame_id = msg.header.frame_id


        self.pub.publish(msg)
        self.camera_info_pub.publish(ci)

        self.get_logger().info(f"[{self.idx+1}/{len(self.image_paths)}] Published {os.path.basename(path)}")
        self.idx += 1

    # CameraInfo creation:
    def intrinsic_from_fov(self, hfov_deg=130, vfov_deg=90, half_pixel=True):
        theta_x = np.deg2rad(hfov_deg)
        theta_y = np.deg2rad(vfov_deg)

        fx = self.width / (2.0 * np.tan(theta_x / 2.0))
        fy = self.height / (2.0 * np.tan(theta_y / 2.0))

        if half_pixel:
            cx = (self.width - 1) / 2.0
            cy = (self.height - 1) / 2.0
        else:
            cx = self.width / 2.0
            cy = self.height / 2.0

        K = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]

        return K

    def make_camera_info(self, frame_id: str = "camera", stamp: Optional[Time] = None,
            distortion_model: str = "plumb_bob",
    ) -> CameraInfo:
        """
         Build a CameraInfo message for an ideal pinhole camera.

         Args:
             width, height: image size in pixels.
             K: 3x3 intrinsic matrix (row-major, length 9 or 3x3 nested iterable).
             frame_id: TF frame for this camera.
             stamp: optional ROS2 time; if None, leave default.
             distortion_model: usually 'plumb_bob' for pinhole.

         Returns:
             sensor_msgs.msg.CameraInfo
         """
        K = self.intrinsic_from_fov()
        K_list = list(K)
        if len(K_list) == 3 and hasattr(K_list[0], "__iter__"):
            K_list = [float(v) for row in K_list for v in row]

        if len(K_list) != 9:
            raise ValueError("K must contain 9 elements (3x3 matrix)")

        fx = K_list[0]
        fy = K_list[4]
        cx = K_list[2]
        cy = K_list[5]

        msg = CameraInfo()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.width = self.width
        msg.height = self.height

        # Intrinsic matrix
        msg.k = K_list

        # Ideal camera: no distortion
        msg.distortion_model = distortion_model
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # 5 coeffs is common, can also use 0-length

        # Rectification matrix: identity (no stereo/rectification)
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # Projection matrix P (3x4), for monocular camera: K with Tx=0
        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        return msg


def main():
    rclpy.init()
    node = FolderImagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

