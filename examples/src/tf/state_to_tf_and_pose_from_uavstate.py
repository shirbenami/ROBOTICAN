#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterAlreadyDeclaredException

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

from fcu_driver_interfaces.msg import UAVState


def quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


class StateToTFAndPose(Node):
    '''
    Publishes:
      1) TF: <map_frame> -> <base_frame>  (dynamic)
      2) PoseStamped on <pose_topic>       (dynamic)

    Input:
      - /R1/fcu/state (fcu_driver_interfaces/msg/UAVState)

    Notes:
      - UAVState azimuth: 0=North, positive clockwise (East=pi/2)
      - If yaw looks rotated, change yaw_mode.
    '''

    def __init__(self):
        super().__init__("state_to_tf_and_pose")

        # Robust sim time set (avoid ParameterAlreadyDeclaredException)
        try:
            self.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # Frames
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        # Pose topic that your map builder subscribes to
        self.pose_topic = self.declare_parameter("pose_topic", "/R1/robot_pose_slam").value

        # yaw_mode options:
        #   - "pi2_minus_azimuth" (default, common if x=East, y=North)
        #   - "azimuth"
        #   - "neg_azimuth"
        self.yaw_mode = self.declare_parameter("yaw_mode", "pi2_minus_azimuth").value

        # IO
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.sub = self.create_subscription(UAVState, "/R1/fcu/state", self.cb, 20)

        self.get_logger().info(
            f"Publishing TF {self.map_frame}->{self.base_frame} and PoseStamped on {self.pose_topic} from /R1/fcu/state"
        )

    def azimuth_to_yaw(self, az: float) -> float:
        if self.yaw_mode == "pi2_minus_azimuth":
            return (math.pi / 2.0) - az
        if self.yaw_mode == "neg_azimuth":
            return -az
        return az

    def cb(self, msg: UAVState):
        # Convert state -> yaw quaternion
        az = float(msg.azimuth)
        yaw = self.azimuth_to_yaw(az)
        qx, qy, qz, qw = quat_from_yaw(yaw)

        # 1) TF: map -> base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(msg.position.x)
        t.transform.translation.y = float(msg.position.y)
        t.transform.translation.z = float(msg.position.z)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

        # 2) PoseStamped in map frame (same pose as base_link)
        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = self.map_frame
        p.pose.position.x = float(msg.position.x)
        p.pose.position.y = float(msg.position.y)
        p.pose.position.z = float(msg.position.z)
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        self.pose_pub.publish(p)


def main():
    rclpy.init()
    node = StateToTFAndPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
