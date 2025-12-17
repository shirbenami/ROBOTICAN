#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32

import tf2_ros
from tf2_ros import TransformException

# Replace `your_pkg_name` with your actual package name
#from ros2apriltag.srv import GetAzimuthAtTime


class TagBasedAzimuth(Node):
    """
    Computes the continuous camera azimuth (yaw) relative to a world frame (0-360 degrees)
    by leveraging the known absolute azimuth of AprilTags placed on the walls.

    It selects the most reliable tag by finding the one closest to the center of the camera frame.

    Additionally, it keeps a time-stamped history of recent azimuth estimates and exposes
    a service `get_azimuth_at_time` that returns the azimuth closest to a requested timestamp.
    """

    def __init__(self):
        super().__init__("tag_based_azimuth")

        # --- Configuration Parameters ---
        self.declare_parameter("robot_ns", "R1")    # "R1" / "R2" / "R3"
        self.declare_parameter("tag_family", "36h11")
        # Ensure this is the correct camera frame name for your setup
        self.declare_parameter("camera_frame", "")

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

        # If camera_frame is not provided, derive it from robot_ns
        # Based on your TF naming: "R1_camera", "R2_camera", ...
        if not self.camera_frame:
            self.camera_frame = f"{self.robot_ns}_camera"
            
        # Clean up camera frame name (remove leading "/")
        if self.camera_frame.startswith("/"):
            self.camera_frame = self.camera_frame[1:]

        # --- Known Tag Configuration ---
        # Define the absolute world azimuth (in degrees, clockwise from North=0)
        # for each known tag ID on the walls.
        self.tag_config = {
            10: 0.0,   # Tag 10 on the North Wall (0 degrees)
            11: 90.0,  # Tag 11 on the East Wall (90 degrees)
            12: 180.0,    # Tag 13 on the West Wall (180 degrees)
            13: 270.0,  # Tag 12 on the South Wall (270 degrees)

        }
            #10: 90.0,   # Tag 10 on the East Wall (90 degrees)
            #11: 270.0,  # Tag 11 on the West Wall (270 degrees)
            #13: 0.0,    # Tag 13 on the North Wall (0 degrees)
            #12: 180.0,  # Tag 12 on the South Wall (180 degrees)

        self.known_tag_ids = list(self.tag_config.keys())

        # --- TF Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publisher ---
        # Publish to /R<i>/camera_azimuth
        azimuth_topic = f"/{self.robot_ns}/camera_azimuth"
        self.azimuth_pub = self.create_publisher(Float32, azimuth_topic, 10)

        # --- Azimuth History Buffer ---
        # Stores tuples of (Time, yaw_deg) for the last N estimates
        self.azimuth_history = deque(maxlen=20)  # You can adjust this length

        # Maximum allowed time difference (in seconds) between requested time
        # and stored azimuth sample for the service to consider it valid
        self.max_time_diff_sec = 1.0

        # --- Service: Get Azimuth At Time ---
        #self.azimuth_service = self.create_service(
         #   GetAzimuthAtTime,
          #  "get_azimuth_at_time",
           # self.handle_get_azimuth_at_time,
        #)

        # --- Timer (5 Hz) ---
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(
            f"TagBasedAzimuth Node Started. Camera Frame: '{self.camera_frame}'"
        )
        self.get_logger().info(f"Tracking Tag IDs: {self.known_tag_ids}")
        self.get_logger().info("Service 'get_azimuth_at_time' is ready.")

    # -------------------------------------------------------------------------
    # Core azimuth computation
    # -------------------------------------------------------------------------
    def get_camera_yaw(self):
        """
        Scans all known tags, calculates the azimuth from each visible one,
        and returns the result from the tag closest to the center of the frame.

        Returns:
            (yaw_deg, tag_id) on success
            (None, last_error) if no tags are visible
        """
        last_error = None
        best_tag_yaw_deg = None
        best_tag_id = None

        # Use infinity to find the smallest absolute relative yaw
        min_abs_relative_yaw = float("inf")

        for tid in self.known_tag_ids:
            # --- Step 1: Find Transform (Camera -> Tag) ---
            candidate_frames = [
                f"tag{self.tag_family}:{tid}",
                f"tag_{tid}",
                f"tag{tid}",
            ]
            transform = None

            for tag_frame in candidate_frames:
                try:
                    # Lookup transform from camera frame to tag frame
                    transform = self.tf_buffer.lookup_transform(
                        self.camera_frame,  # target frame
                        tag_frame,          # source frame
                        rclpy.time.Time(),
                    )
                    break
                except TransformException as e:
                    last_error = e
                    continue

            # --- Step 2: If Transform Found, Calculate Yaw and Quality ---
            if transform:
                t = transform.transform.translation

                # Calculate relative yaw (angle deviation of tag from camera center)
                # Assumes standard optical frame (Z forward, X right).
                # The sign might need adjustment (t.x vs -t.x) depending on TF definition.
                relative_yaw_rad = math.atan2(-t.x, t.z)
                relative_yaw_deg = math.degrees(relative_yaw_rad)

                # The absolute relative yaw indicates how close the tag is to the center
                abs_relative_yaw = abs(relative_yaw_deg)

                # --- Step 3: Select the Best Tag (Closest to Center) ---
                if abs_relative_yaw < min_abs_relative_yaw:
                    min_abs_relative_yaw = abs_relative_yaw

                    wall_azimuth_deg = self.tag_config[tid]

                    # Compute the absolute world azimuth of the camera
                    camera_yaw = wall_azimuth_deg + relative_yaw_deg

                    # Normalize to 0-360 range
                    camera_yaw = camera_yaw % 360.0

                    # Store data for the current best tag
                    best_tag_yaw_deg = camera_yaw
                    best_tag_id = tid

        # --- Step 4: Return the Best Result ---
        if best_tag_yaw_deg is not None:
            return best_tag_yaw_deg, best_tag_id

        # No tags visible
        return None, last_error

    # -------------------------------------------------------------------------
    # Timer callback: compute yaw, publish, and store in history
    # -------------------------------------------------------------------------
    def timer_callback(self):
        """
        Periodically calls the yaw calculation and publishes the result.
        Also stores (time, azimuth) in a history buffer for later querying.
        """
        yaw_deg, info = self.get_camera_yaw()

        if yaw_deg is not None:
            # --- Success ---
            # 1) Store in history with current ROS time
            now_time = self.get_clock().now()
            self.azimuth_history.append((now_time, yaw_deg))

            # 2) Publish Float32 message for other consumers
            msg = Float32()
            msg.data = float(yaw_deg)
            self.azimuth_pub.publish(msg)

            # 3) Logging
            self.get_logger().info(
                f"Azimuth: {yaw_deg:.1f}° (Based on Tag {info})"
            )
        else:
            # --- Failure (No tags visible) ---
            self.get_logger().warn(
                f"No tags visible. Last TF error: {info}",
                throttle_duration_sec=2.0,
            )

    # -------------------------------------------------------------------------
    # Service callback: get azimuth closest to a requested timestamp
    # -------------------------------------------------------------------------
    def handle_get_azimuth_at_time(self, request, response):
        """
        Service callback for GetAzimuthAtTime.

        Input (request):
            request.stamp (builtin_interfaces/Time):
                The timestamp for which the client wants the camera azimuth.

        Output (response):
            response.success (bool):
                True if a suitable azimuth sample was found, False otherwise.
            response.azimuth (float32):
                The azimuth corresponding to the closest sample in time.
            response.time_diff (float32):
                The signed time difference (in seconds) between the matched sample
                and the requested timestamp (sample_time - requested_time).
        """
        if not self.azimuth_history:
            self.get_logger().warn(
                "GetAzimuthAtTime: history is empty, no azimuth samples available."
            )
            response.success = False
            response.azimuth = 0.0
            response.time_diff = 0.0
            return response

        # Convert the requested stamp to rclpy.time.Time
        req_time = Time.from_msg(request.stamp)

        # Helper: return absolute time difference between a sample and the requested time
        def abs_time_diff_seconds(sample):
            sample_time, _ = sample
            dt = (sample_time - req_time).nanoseconds / 1e9
            return abs(dt)

        # Find the sample in history closest in time to req_time
        closest_sample = min(self.azimuth_history, key=abs_time_diff_seconds)
        closest_time, closest_yaw = closest_sample

        dt_sec = (closest_time - req_time).nanoseconds / 1e9
        abs_dt_sec = abs(dt_sec)

        # Check if the closest sample is within the allowed time window
        if abs_dt_sec > self.max_time_diff_sec:
            self.get_logger().warn(
                f"GetAzimuthAtTime: no close sample found "
                f"(min diff={abs_dt_sec:.3f}s > {self.max_time_diff_sec:.3f}s)."
            )
            response.success = False
            response.azimuth = 0.0
            response.time_diff = float(abs_dt_sec)
            return response

        # Return the matched azimuth
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