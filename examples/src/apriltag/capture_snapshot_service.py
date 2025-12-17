#!/usr/bin/env python3
import uuid
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger  # Simple service: returns image_id in response.message

import tf2_ros
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import TransformException

@dataclass
class Snapshot:
    image_id: str
    stamp: Time
    camera_frame: str
    fx: float
    fy: float
    cx: float
    cy: float
    tag_frame_used: str
    T_cam_tag: object  # geometry_msgs/TransformStamped


class CaptureSnapshotService(Node):
    """
    Service-driven snapshot creator:
    - On service call, waits for the next image frame
    - Creates a Snapshot tied to that frame timestamp
    - Stores the Snapshot in memory by image_id
    - Returns image_id to the caller
    """

    def __init__(self):
        super().__init__("capture_snapshot_service")

        # ---- Parameters ----
        self.declare_parameter("image_topic", "/R1/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/R1/camera/camera_info")
        self.declare_parameter("tag_id", 15)
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("capture_timeout_sec", 2.0)
        self.declare_parameter("snapshot_ttl_sec", 30.0)  # auto cleanup

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.tag_family = self.get_parameter("tag_family").value
        self.capture_timeout_sec = float(self.get_parameter("capture_timeout_sec").value)
        self.snapshot_ttl_sec = float(self.get_parameter("snapshot_ttl_sec").value)

        self.cb_group = ReentrantCallbackGroup()
        self._capture_event = threading.Event()
        self._capture_prev_stamp = None
        self._capture_new_image = None
        
        # ---- Intrinsics ----
        self.have_K = False
        self.fx = self.fy = self.cx = self.cy = 0.0

        # ---- Latest image tracking ----
        self._last_image_msg: Optional[Image] = None
        self._last_image_stamp: Optional[Time] = None

        # ---- TF buffer ----
        # Keep TF history long enough so lookup_transform(camera, tag, t_img) can succeed.
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Snapshot store ----
        self.snapshots: Dict[str, Snapshot] = {}

             # ---- Subscribers ----
        self.sub_caminfo = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_caminfo,
            qos_profile_sensor_data, callback_group=self.cb_group
        )
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self._on_image,
            qos_profile_sensor_data, callback_group=self.cb_group
        )

        # ---- Service ----
        self.srv = self.create_service(
            Trigger, "capture_snapshot", self._on_capture_snapshot,
            callback_group=self.cb_group
        )

        # ---- Cleanup timer ----
        self.cleanup_timer = self.create_timer(1.0, self._cleanup_old_snapshots)


        self.get_logger().info(
            "CaptureSnapshotService started.\n"
            f"  image_topic:       {self.image_topic}\n"
            f"  camera_info_topic: {self.camera_info_topic}\n"
            f"  tag_family:        {self.tag_family}\n"
            f"  tag_id:            {self.tag_id}\n"
            f"  timeout_sec:       {self.capture_timeout_sec}\n"
            f"  snapshot_ttl_sec:  {self.snapshot_ttl_sec}"
        )

    # ---------------- Subscriptions ---------------- #

    def _on_caminfo(self, msg: CameraInfo):
        # Camera intrinsics from K matrix
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.have_K = True

    def _on_image(self, msg: Image):
        self._last_image_msg = msg
        self._last_image_stamp = Time.from_msg(msg.header.stamp)

        if self._capture_prev_stamp is not None:
            if self._last_image_stamp != self._capture_prev_stamp:
                self._capture_new_image = msg
                self._capture_event.set()


    # ---------------- TF helpers ---------------- #

    def _candidate_tag_frames(self):
        # Only the actual frame that exists in your TF tree
        return [f"tag{self.tag_family}:{self.tag_id}"]  # e.g. "tag36h11:15"

    def _lookup_cam_to_tag_at_time(self, camera_frame: str, t_img: Time):
        tag_frame = f"tag{self.tag_family}:{self.tag_id}"  # "tag36h11:15"

        # 1) Try exact time
        try:
            transform = self.tf_buffer.lookup_transform(camera_frame, tag_frame, t_img)
            return transform, tag_frame, "exact"
        except TransformException as ex_exact:
            # 2) Fallback: latest available
            try:
                transform = self.tf_buffer.lookup_transform(camera_frame, tag_frame, Time())
                return transform, tag_frame, "latest"
            except TransformException as ex_latest:
                return None, f"exact_error={ex_exact}; latest_error={ex_latest}", None

    # ---------------- Snapshot logic ---------------- #



    def _on_capture_snapshot(self, request, response):
        if not self.have_K:
            response.success = False
            response.message = "No CameraInfo received yet (intrinsics missing)."
            return response

        self._capture_event.clear()
        self._capture_new_image = None
        self._capture_prev_stamp = self._last_image_stamp  # can be None

        ok = self._capture_event.wait(timeout=self.capture_timeout_sec)

        self._capture_prev_stamp = None  # stop capture mode

        if not ok or self._capture_new_image is None:
            response.success = False
            response.message = "Timeout waiting for next image frame."
            return response

        img = self._capture_new_image

        # Build snapshot tied to this exact image timestamp
        image_id = str(uuid.uuid4())
        t_img = Time.from_msg(img.header.stamp)

        camera_frame = img.header.frame_id.lstrip("/")
        if camera_frame == "":
            # If frame_id is empty, fallback to a common default (you can change it).
            camera_frame = "camera"

        transform, tag_frame_used, mode = self._lookup_cam_to_tag_at_time(camera_frame, t_img)
        if transform is None:
            response.success = False
            response.message = f"Could not find TF {camera_frame} -> {tag_frame_used}: {mode}"
            return response

        self.get_logger().info(f"TF lookup mode={mode}")


        snap = Snapshot(
            image_id=image_id,
            stamp=t_img,
            camera_frame=camera_frame,
            fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
            tag_frame_used=tag_frame_used,
            T_cam_tag=transform
        )

        self.snapshots[image_id] = snap

        # Return image_id to caller so it can tag NanoOWL request/results
        response.success = True
        response.message = image_id

        self.get_logger().info(
            f"Snapshot stored: image_id={image_id} "
            f"stamp_ns={t_img.nanoseconds} camera_frame={camera_frame} tag_frame={tag_frame_used}"
        )
        return response

    def _cleanup_old_snapshots(self):
        """
        Remove snapshots older than snapshot_ttl_sec.
        Uses node clock time; snapshots store image stamp, which is typically close enough.
        """
        if not self.snapshots:
            return

        now = self.get_clock().now()
        ttl_ns = int(self.snapshot_ttl_sec * 1e9)

        to_delete = []
        for image_id, snap in self.snapshots.items():
            age_ns = (now - snap.stamp).nanoseconds
            if age_ns > ttl_ns:
                to_delete.append(image_id)

        for image_id in to_delete:
            del self.snapshots[image_id]


def main(args=None):
    rclpy.init(args=args)
    node = CaptureSnapshotService()
    executor = MultiThreadedExecutor(num_threads=4)
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