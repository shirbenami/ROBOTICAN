#!/usr/bin/env python3
import argparse
import datetime
import time

import rclpy
from rclpy.node import Node
from rclpy.client import Client

from video_handler_interfaces.srv import SetVideoMode
from std_msgs.msg import Bool, Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import tf2_ros
from tf2_ros import TransformException

import numpy as np
import cv2
from cv_bridge import CvBridge
import copy
import math
from typing import Iterable, Optional

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import Gst, GstVideo

Gst.init(None)


class VideoStreamManager(Node):
    def __init__(self, drone_id="R2", high_resolution=640, host_ip="192.168.131.24", port=5001):
        super().__init__("video_stream_example")
        self.id = drone_id
        self.i = 0
        self.width = high_resolution
        self.height = int(self.width * 9 / 16)
        self.host = host_ip    # host IP "192.168.131.24" Laptop
        self.port = port

        self.last_pub_time = 0  # in seconds, wall-clock time
        self.pub_period = 0.5  # seconds between publishes

        self.awaiting_detection = False
        self.capturing_enabled = False
        self.last_frame_msg = None
        self.camera_info_template = None
        self.dir_name = None


        # cv_bridge for publishing
        self.bridge = CvBridge()

        # ---- ROS2: topic names ----
        azimuth_topic = f"/{self.id}/camera_azimuth"
        image_raw_topic = f"/{self.id}/camera/image_raw"
        camera_info_topic = f"/{self.id}/camera/camera_info"
        image_used_topic = f"/{self.id}/selected_frame"
        self.camera_frame = f"{self.id}_camera"

        # ---- ROS2: service + keep-alive ----
        self.set_video_mode_srv: Client = self.create_client(
            srv_type=SetVideoMode,
            srv_name=f"/{self.id}/video_handler/set_video_mode",
        )
        # Keep Alive
        self.gcs_keep_alive_publisher = self.create_publisher(
            Bool, f"/{self.id}/gcs_keep_alive", 10
        )
        self.gcs_keep_alive_timer = self.create_timer(
            1.0, self.gcs_keep_alive_timer_callback
        )
        # ROS2 Timer
        self.video_on_timer = self.create_timer(
            3.0, self.video_on_timer_callback
        )

        # --- TF Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publisher ---
        self.image_pub = self.create_publisher(
            Image, image_raw_topic, qos_profile_sensor_data
        )
        self.camera_info_pub = self.create_publisher(CameraInfo,
                                                     camera_info_topic, qos_profile_sensor_data)
        self.azimuth_pub = self.create_publisher(PointStamped, azimuth_topic, qos_profile_sensor_data)
        self.image_used_pub = self.create_publisher(Image, image_used_topic, qos_profile_sensor_data)

        # --- Services
        self.create_service(Trigger, f"/{self.id}/start_capture", self.handle_start_capture)
        self.create_service(Trigger, f"/{self.id}/stop_capture", self.handle_stop_capture)




        # Azimuth Parameters
        self.tag_config = {
            10: 0.0,  # North
            11: 90.0,  # East
            12: 180.0,  # South
            13: 270.0,  # West
        }
        self.tag_family = "36h11"
        self.known_tag_ids = list(self.tag_config.keys())

        # ---- GStreamer pipeline with appsink ----
        gst_pipeline = (
            f"udpsrc port={self.port} "
            "caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,"
            "encoding-name=(string)H264,payload=(int)96 ! "
            "rtph264depay ! "
            "decodebin ! "
            "videoconvert ! "
           # "videocrop name=cropper top=42 left=1 right=4 bottom=0 ! "
            "video/x-raw,format=BGR ! "
            "appsink name=mysink emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self.get_logger().info(f"Creating GStreamer pipeline:\n{gst_pipeline}")

        self.pipeline = Gst.parse_launch(gst_pipeline)
        self.appsink = self.pipeline.get_by_name("mysink")
        if self.appsink is None:
            self.get_logger().error("Failed to get appsink from pipeline")
            raise RuntimeError("appsink not found")

        # be explicit about properties
        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("sync", False)
        self.appsink.set_property("max-buffers", 1)
        self.appsink.set_property("drop", True)

        # connect callback that will be called on every new frame
        self.appsink.connect("new-sample", self.on_new_sample)

        # optional: listen to bus errors
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_gst_error)

        # start the pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Failed to set GStreamer pipeline to PLAYING")
        else:
            self.get_logger().info("GStreamer pipeline set to PLAYING")

        # --- Subscriptions to Azimuth node outputs ---
        self.last_azimuth_deg = None
        self.captured_count = 0
        self.max_captures = 20

    # ---------- ROS2 timers ----------

    def gcs_keep_alive_timer_callback(self):
        msg = Bool()
        msg.data = True
        self.gcs_keep_alive_publisher.publish(msg)

    def video_on_timer_callback(self):
        # run only once
        self.video_on_timer.cancel()

        if not self.set_video_mode_srv.service_is_ready():
            self.get_logger().warn("set_video_mode service not ready yet, retrying...")
            self.video_on_timer.reset()
            return

        req = SetVideoMode.Request()
        req.camera_id = 0
        req.playing = True
        req.port = self.port
        req.host = self.host   # host IP
        req.resolution_width = self.width
        req.resolution_height = self.height
        req.recording = False
        req.bitrate = SetVideoMode.Request.BITRATE_1500000
        req.fps = 0  # default

        self.get_logger().info(
            f"send {self.set_video_mode_srv.srv_name} [{req}]"
        )

        future = self.set_video_mode_srv.call_async(req)

        def set_video_mode_cb(fut: rclpy.client.Future):
            try:
                result = fut.result()
                self.get_logger().info(
                    f"{self.id}: set_video_mode_cb: success={result.success}, msg='{result.message}'"
                )
            except Exception as e:
                self.get_logger().error(f"{self.id}: set_video_mode failed: {e}")

        future.add_done_callback(set_video_mode_cb)

    def handle_start_capture(self, request, context):
        self.capturing_enabled = True
        self.dir_name = datetime.datetime.now().strftime("%Y_%m_%d___%H_%M_%S")
        self.get_logger().info("Capture ENABLED by service call")
        resp = Trigger.Response()
        resp.success = True
        resp.message = "Started capturing"
        return resp

    def handle_stop_capture(self, request, context):
        self.capturing_enabled = False
        self.get_logger().info("Capture DISABLED by service call")
        resp = Trigger.Response()
        resp.success = True
        resp.message = "Stopped capturing"
        return resp

    # ---------- GStreamer callbacks ----------

    def on_gst_error(self, bus, msg):
        err, debug = msg.parse_error()
        self.get_logger().error(f"GStreamer error: {err} (debug: {debug})")

    def on_new_sample(self, sink):
        """
        Called by GStreamer streaming thread whenever a new frame arrives.
        Rate-limits publishing to at most 1 every self.pub_period seconds.
        """
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()

        # Get video info (width/height)
        info = GstVideo.VideoInfo()
        try:
            info.from_caps(caps)
        except Exception as e:
            self.get_logger().error(f"VideoInfo.from_caps failed: {e}")
            return Gst.FlowReturn.ERROR

        width, height = info.width, info.height

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().warn("Failed to map buffer")
            return Gst.FlowReturn.ERROR

        try:
            data = map_info.data
            expected = width * height * 3
            if len(data) < expected:
                self.get_logger().warn(
                    f"Mapped buffer too small: len={len(data)}, expected={expected}"
                )
                return Gst.FlowReturn.ERROR

            # Only publish if capturing enabled!
            if not self.capturing_enabled:
                return Gst.FlowReturn.OK
            # --- Rate limiting (wall time) ---
            now = time.time()
            if now - self.last_pub_time < self.pub_period:
                return Gst.FlowReturn.OK  # skip this frame

            frame = np.frombuffer(data, dtype=np.uint8)
            try:
                frame = frame.reshape((height, width, 3))
            except ValueError as e:
                self.get_logger().error(
                    f"Reshape failed for frame {height}x{width}: {e}"
                )
                return Gst.FlowReturn.ERROR

            frame = frame.copy()  # detach from Gst buffer

        finally:
            buf.unmap(map_info)

        # Only update last_pub_time **if** we publish!
        self.last_pub_time = now
        self.process_frame(frame)

        return Gst.FlowReturn.OK

    # ----------  logic on each frame ----------

    def process_frame(self, frame: np.ndarray):
        """
        frame is a numpy array, shape (H, W, 3), BGR.
        """
        h, w, _ = frame.shape
        self.i += 1
        
        # Build CameraInfo template once, when we see the first frame
        if self.camera_info_template is None:
            # Use the camera frame id you actually publish on images
            self.camera_info_template = self.make_camera_info(
                frame_id=f"{self.id}_camera"
            )
        # occasional debug
        if self.i % 500 == 1:
            self.get_logger().info(f"Frame #{self.i}: {w}x{h}")

        # Optional: save a frame every 100 frames
        # if self.i % 100 == 0:
        #     cv2.imwrite(f"{self.id}_{self.i}.png", frame)

        # ROS2 publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.last_pub_time
        msg.header.frame_id = f"{self.id}_camera"

        ci = copy.deepcopy(self.camera_info_template)
        # Update header + size (in case the actual frame size differs)
        ci.header.stamp = msg.header.stamp
        ci.header.frame_id = msg.header.frame_id
        ci.width = w
        ci.height = h

        self.last_frame_msg = msg
        self.image_pub.publish(msg)
        self.camera_info_pub.publish(ci)
        self.awaiting_detection = True

        last_yaw_deg, tag_id = self.get_camera_yaw(msg.header.stamp)
        if last_yaw_deg is None:
            self.get_logger().warn(
                f"No tags visible. Last TF error: {tag_id}")
            azimuth_msg = PointStamped()
            azimuth_msg.header.stamp = None
        else:
            azimuth_value_msg = Point()
            azimuth_msg = PointStamped()
            azimuth_value_msg.x = float(last_yaw_deg)
            azimuth_value_msg.y = float(tag_id)
            azimuth_msg.header.stamp = msg.header.stamp
            azimuth_msg.header.frame_id = self.dir_name
            azimuth_msg.point = azimuth_value_msg
            self.get_logger().info(
                f"Azimuth: {last_yaw_deg:.1f}° (Based on Tag {tag_id})")

        self.frame_and_azimuth_publisher(azimuth_msg)

        # CameraInfo creation:
    def intrinsic_from_fov(self,  hfov_deg=130, vfov_deg=90, half_pixel=True):
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

    def frame_and_azimuth_publisher(self, azimuth_msg):
        if not self.awaiting_detection:
            return
        if not self.last_pub_time or azimuth_msg.header.stamp != self.last_pub_time:
            return  # Not the detection for our last_frame_msg


        self.image_used_pub.publish(self.last_frame_msg)
        self.azimuth_pub.publish(azimuth_msg)
        self.awaiting_detection = False  # Allow next frame to be published
        self.last_frame_msg = None

    # -------------------------------------------------------------------------
    # Core azimuth computation
    # -------------------------------------------------------------------------
    def get_camera_yaw(self, frame_timestamp):
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

        query_time = frame_timestamp
        if query_time is None:
            query_time = rclpy.time.Time()
            self.get_logger().warn("Got frame without timestamp.")

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
                        self.camera_frame,
                        tag_frame,
                        query_time,
                        timeout=rclpy.duration.Duration(seconds=0.05)
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

    # ---------- cleanup ----------

    def destroy_node(self):
        # stop pipeline
        if hasattr(self, "pipeline") and self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone-id", default="R2", help="Drone ID (R1/R2/R3...)")
    parser.add_argument("--host-ip", default="192.168.131.22", help="Host IP for UDP video sink")
    parser.add_argument("--port", type=int, default=5001, help="UDP port for video stream")
    parser.add_argument("--width", type=int, default=640, help="Image width in pixels")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = VideoStreamManager(
        drone_id=parsed.drone_id,
        high_resolution=parsed.width,
        host_ip=parsed.host_ip,
        port=parsed.port,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()