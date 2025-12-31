import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from sensor_msgs.msg import Image, CameraInfo, Imu
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
from video_handler_interfaces.srv import SetVideoMode
from fcu_driver_interfaces.msg import UAVState
from cv_bridge import CvBridge
import numpy as np
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import Gst, GstVideo

Gst.init(None)


class DroneDataBridge(Node):
    def __init__(self, drone_id="R1", high_resolution=640, host_ip="192.168.131.5", port=5001):
        super().__init__("drone_data_bridge")

        self.appsink = None
        self.pipeline = None
        self.id = drone_id
        self.bridge = CvBridge()
        self.last_state = None
        self.width = high_resolution
        self.height = int(self.width * 9 / 16)
        self.host = host_ip  # host IP "192.168.131.24" Laptop
        self.port = port

        # 1. Camera Publishers
        self.image_pub = self.create_publisher(Image, f"/{self.id}/camera/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, f"/{self.id}/camera/camera_info", 10)

        # 2. Virtual IMU Publisher
        self.imu_pub = self.create_publisher(Imu, "/visual_slam/imu", 10)
        self.state_sub = self.create_subscription(UAVState, f"/{self.id}/fcu/state", self.state_callback, 10)

        # 3. Service Client & Keep-Alive (from your video_stream.py)
        self.set_video_mode_srv = self.create_client(SetVideoMode, f"/{self.id}/video_handler/set_video_mode")
        self.gcs_keep_alive_pub = self.create_publisher(Bool, f"/{self.id}/gcs_keep_alive", 10)
        self.create_timer(1.0, lambda: self.gcs_keep_alive_pub.publish(Bool(data=True)))
        self.video_on_timer = self.create_timer(3.0, self.video_on_timer_callback)

        # 4. GStreamer Pipeline
        self.setup_pipeline()


    def state_callback(self, msg):
        """Calculates IMU acceleration and rates from FCU state"""
        if self.last_state is None:
            self.last_state = msg
            return

        # Calculate time delta
        dt = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - \
             (self.last_state.header.stamp.sec + self.last_state.header.stamp.nanosec * 1e-9)
        if dt <= 0: return

        imu = Imu()
        imu.header = msg.header
        imu.header.frame_id = f"{self.id}_imu_frame"

        # Calculate Linear Acceleration (dv/dt)
        imu.linear_acceleration.x = (msg.velocity.x - self.last_state.velocity.x) / dt
        imu.linear_acceleration.y = (msg.velocity.y - self.last_state.velocity.y) / dt
        imu.linear_acceleration.z = (msg.velocity.z - self.last_state.velocity.z) / dt + 9.81  # Add gravity

        # Calculate Angular Velocity (d_angle/dt)
        imu.angular_velocity.x = (msg.roll - self.last_state.roll) / dt
        imu.angular_velocity.y = (msg.pitch - self.last_state.pitch) / dt
        imu.angular_velocity.z = (msg.azimuth - self.last_state.azimuth) / dt

        self.imu_pub.publish(imu)
        self.last_state = msg


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

    def get_camera_info(self, frame_id: str = "camera", stamp: Optional[Time] = None,
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


    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
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
            except Exception as e:
                self.get_logger().error(f"Exception is: {e}")
                return Gst.FlowReturn.ERROR

            frame = frame.copy()  # detach from Gst buffer

        finally:
            buf.unmap(map_info)


        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = f"{self.id}_camera_optical"

        self.image_pub.publish(img_msg)
        info_msg = self.get_camera_info(frame_id=f"{self.id}_camera_optical", stamp=stamp)
        self.info_pub.publish(info_msg)
        return Gst.FlowReturn.OK


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

    def setup_pipeline(self):
        # ---- GStreamer pipeline with appsink ----
        gst_pipeline = (
            f"udpsrc port={self.port} buffer-size=5242880 do-timestamp=true "
            "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! "
            "rtpjitterbuffer latency=100 drop-on-latency=true ! "
            "rtph264depay ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "decodebin ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=mysink emit-signals=true sync=false max-buffers=1 drop=true enable-last-sample=false"
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
    
    def on_gst_error(self, bus, msg):
        err, debug = msg.parse_error()
        self.get_logger().error(f"GStreamer error: {err} (debug: {debug})")



    # ---------- cleanup ----------

    def destroy_node(self):
        # stop pipeline
        if hasattr(self, "pipeline") and self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone-id", default="R2", help="Drone ID (R1/R2/R3...)")
    parser.add_argument("--host-ip", default="192.168.131.20", help="Host IP for UDP video sink")
    parser.add_argument("--port", type=int, default=5001, help="UDP port for video stream")
    parser.add_argument("--width", type=int, default=640, help="Image width in pixels")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = DroneDataBridge(
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




