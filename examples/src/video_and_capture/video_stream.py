#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.node import Node
from rclpy.client import Client

from video_handler_interfaces.srv import SetVideoMode
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time

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


class VideoStreamExample(Node):
    def __init__(self, drone_id="R2", high_resolution=640, host_ip="192.168.131.24", port=5001):
        super().__init__("video_stream_example")
        self.id = drone_id
        self.i = 0
        self.width = high_resolution
        self.height = int(self.width * 9 / 16)
        self.host = host_ip    # host IP "192.168.131.24" Laptop
        self.port = port

        # cv_bridge for publishing
        self.bridge = CvBridge()

        # ---- ROS2: service + keep-alive ----
        self.set_video_mode_srv: Client = self.create_client(
            srv_type=SetVideoMode,
            srv_name=f"/{self.id}/video_handler/set_video_mode",
        )

        self.gcs_keep_alive_publisher = self.create_publisher(
            Bool, f"/{self.id}/gcs_keep_alive", 10
        )
        self.gcs_keep_alive_timer = self.create_timer(
            1.0, self.gcs_keep_alive_timer_callback
        )

        self.video_on_timer = self.create_timer(
            3.0, self.video_on_timer_callback
        )

        self.image_pub = self.create_publisher(
            Image, f"/{self.id}/camera/image_raw", 10
        )
        self.camera_info_pub = self.create_publisher(CameraInfo,
                                                     f"/{self.id}/camera/camera_info", 10)

        self.camera_info_template = None
        self.camera_info_pub = self.create_publisher(
            CameraInfo, f"/{self.id}/camera/camera_info", 10
        )


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

    # ---------- GStreamer callbacks ----------

    def on_gst_error(self, bus, msg):
        err, debug = msg.parse_error()
        self.get_logger().error(f"GStreamer error: {err} (debug: {debug})")

    def on_new_sample(self, sink):
        """
        Called by GStreamer streaming thread whenever a new frame arrives.
        We convert GstSample → numpy array (BGR) and hand it to process_frame().
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

        # now we can use the frame in our own code
        self.process_frame(frame)

        return Gst.FlowReturn.OK

    # ---------- your logic on each frame ----------

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
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.id}_camera"

        ci = copy.deepcopy(self.camera_info_template)
        # Update header + size (in case the actual frame size differs)
        ci.header.stamp = msg.header.stamp
        ci.header.frame_id = msg.header.frame_id
        ci.width = w
        ci.height = h


        self.image_pub.publish(msg)
        self.camera_info_pub.publish(ci)


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
    node = VideoStreamExample(
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