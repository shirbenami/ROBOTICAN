#!/usr/bin/env python3
"""
state_to_tf_and_pose_unified.py

A single node that can consume either:
  - fcu_driver_interfaces/msg/UAVState   (topic like /R1/fcu/state)
  - nav_msgs/msg/Odometry               (topic like /simple_drone/odom)

and publishes:
  1) Dynamic TF:  <map_frame> -> <base_frame>
  2) PoseStamped: <pose_topic> in <map_frame>

Optional:
  3) Static TF:   <camera_link_frame> -> <camera_optical_frame> (REP-103 optical)

Key idea:
- UAVState provides position + azimuth -> convert azimuth to ROS yaw (configurable).
- Odometry already provides full pose quaternion -> use it directly (no azimuth field needed).

Run examples (everything as ROS args / params):

UAVState:
  python3 state_to_tf_and_pose_unified.py --ros-args \
    -p use_sim_time:=true \
    -p mode:=uavstate \
    -p uav_state_topic:=/R1/fcu/state \
    -p map_frame:=map \
    -p base_frame:=base_link \
    -p pose_topic:=/R1/robot_pose_slam \
    -p yaw_mode:=pi2_minus_azimuth \
    -p publish_optical_tf:=true \
    -p camera_link_frame:=main_camera_link \
    -p camera_optical_frame:=main_camera_optical

Odometry:
  python3 state_to_tf_and_pose_unified.py --ros-args \
    -p use_sim_time:=true \
    -p mode:=odom \
    -p odom_topic:=/simple_drone/odom \
    -p map_frame:=map \
    -p base_frame:=base_link \
    -p pose_topic:=/simple_drone/robot_pose_slam \
    -p use_msg_frames:=true

Notes about frames:
- If use_msg_frames=true and mode=odom, we'll use:
    parent = odom.header.frame_id
    child  = odom.child_frame_id
  and ignore map_frame/base_frame params for TF publishing (PoseStamped will still use parent).
- If use_msg_frames=false, we'll force TF to map_frame->base_frame regardless of the message frames.

"""

import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterAlreadyDeclaredException

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from fcu_driver_interfaces.msg import UAVState


def _normalize_frame(frame: str) -> str:
    """TF frames should not start with '/'. Normalize to be safe."""
    if not frame:
        return frame
    return frame[1:] if frame.startswith("/") else frame


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # Standard ROS yaw about +Z.
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class StateToTFAndPose(Node):
    def __init__(self):
        super().__init__("state_to_tf_and_pose_unified")

        # Robust sim time set (avoid ParameterAlreadyDeclaredException)
        try:
            self.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # Mode selection
        #   - "uavstate" : subscribe UAVState
        #   - "odom"     : subscribe Odometry
        self.mode = str(self.declare_parameter("mode", "uavstate").value).lower()

        # Topics
        self.uav_state_topic = self.declare_parameter("uav_state_topic", "/R1/fcu/state").value
        self.odom_topic = self.declare_parameter("odom_topic", "/simple_drone/odom").value

        # Frames (defaults)
        self.map_frame = _normalize_frame(self.declare_parameter("map_frame", "map").value)
        self.base_frame = _normalize_frame(self.declare_parameter("base_frame", "base_link").value)

        # For odom: optionally use msg.header.frame_id / msg.child_frame_id
        self.use_msg_frames = bool(self.declare_parameter("use_msg_frames", False).value)

        # Pose output topic
        self.pose_topic = self.declare_parameter("pose_topic", "/R1/robot_pose_slam").value

        # For UAVState: azimuth -> yaw conversion
        # yaw_mode options:
        #   - "pi2_minus_azimuth" (default, common if x=East, y=North)
        #   - "azimuth"
        #   - "neg_azimuth"
        self.yaw_mode = str(self.declare_parameter("yaw_mode", "pi2_minus_azimuth").value)

        # Optional: camera optical TF
        self.publish_optical_tf = bool(self.declare_parameter("publish_optical_tf", False).value)
        self.camera_link_frame = _normalize_frame(self.declare_parameter("camera_link_frame", "main_camera_link").value)
        self.camera_optical_frame = _normalize_frame(self.declare_parameter("camera_optical_frame", "main_camera_optical").value)

        # Default quaternion for camera_link -> camera_optical (REP-103)
        # Many URDFs use: (x, y, z, w) = (0.5, -0.5, 0.5, 0.5)
        self.optical_qx = float(self.declare_parameter("optical_qx", 0.5).value)
        self.optical_qy = float(self.declare_parameter("optical_qy", -0.5).value)
        self.optical_qz = float(self.declare_parameter("optical_qz", 0.5).value)
        self.optical_qw = float(self.declare_parameter("optical_qw", 0.5).value)

        # TF publishers
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # Pose publisher
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        # Subscriptions
        if self.mode == "odom":
            self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 50)
            self.get_logger().info(f"Mode=odom. Subscribing to {self.odom_topic}")
        else:
            self.sub = self.create_subscription(UAVState, self.uav_state_topic, self.cb_uavstate, 50)
            self.get_logger().info(f"Mode=uavstate. Subscribing to {self.uav_state_topic}")

        # Optional static optical TF
        if self.publish_optical_tf:
            self._publish_optical_tf_once()

        self.get_logger().info(
            f"Publishing TF + PoseStamped. pose_topic={self.pose_topic}, "
            f"default frames={self.map_frame}->{self.base_frame}, use_msg_frames={self.use_msg_frames}"
        )

    def _publish_optical_tf_once(self):
        t = TransformStamped()
        # Use current time; static TF doesn't need exact stamps.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_link_frame
        t.child_frame_id = self.camera_optical_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.optical_qx
        t.transform.rotation.y = self.optical_qy
        t.transform.rotation.z = self.optical_qz
        t.transform.rotation.w = self.optical_qw
        self.static_br.sendTransform(t)
        self.get_logger().info(f"Published static optical TF: {self.camera_link_frame}->{self.camera_optical_frame}")

    def azimuth_to_yaw(self, az: float) -> float:
        if self.yaw_mode == "pi2_minus_azimuth":
            return (math.pi / 2.0) - az
        if self.yaw_mode == "neg_azimuth":
            return -az
        return az

    def _publish_tf_and_pose(
        self,
        stamp_msg,
        parent_frame: str,
        child_frame: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ):
        parent_frame = _normalize_frame(parent_frame)
        child_frame = _normalize_frame(child_frame)

        # 1) TF
        t = TransformStamped()
        t.header.stamp = stamp_msg
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.br.sendTransform(t)

        # 2) PoseStamped (in parent_frame)
        p = PoseStamped()
        p.header.stamp = stamp_msg
        p.header.frame_id = parent_frame
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation.x = float(qx)
        p.pose.orientation.y = float(qy)
        p.pose.orientation.z = float(qz)
        p.pose.orientation.w = float(qw)
        self.pose_pub.publish(p)

    def cb_uavstate(self, msg: UAVState):
        # UAVState has position (ENU) + azimuth (0 North, CW positive).
        yaw = self.azimuth_to_yaw(float(msg.azimuth))
        qx, qy, qz, qw = quat_from_yaw(yaw)

        parent = self.map_frame
        child = self.base_frame

        self._publish_tf_and_pose(
            msg.header.stamp,
            parent,
            child,
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
            qx, qy, qz, qw,
        )

    def cb_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        if self.use_msg_frames:
            parent = msg.header.frame_id or self.map_frame
            child = msg.child_frame_id or self.base_frame
        else:
            parent = self.map_frame
            child = self.base_frame

        self._publish_tf_and_pose(
            msg.header.stamp,
            parent,
            child,
            float(pos.x),
            float(pos.y),
            float(pos.z),
            float(ori.x),
            float(ori.y),
            float(ori.z),
            float(ori.w),
        )


def main(args=None):
    rclpy.init(args=args)
    node = StateToTFAndPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
