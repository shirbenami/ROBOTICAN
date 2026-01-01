#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from fcu_driver_interfaces.msg import UAVState


def quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


class StateToTF(Node):
    def __init__(self):
        super().__init__("state_to_tf")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.child_frame = self.declare_parameter("child_frame", "fcu_1").value

        # yaw_mode: "pi2_minus_azimuth" (default) or "azimuth"
        self.yaw_mode = self.declare_parameter("yaw_mode", "pi2_minus_azimuth").value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(UAVState, "/R1/fcu/state", self.cb, 10)

        self.get_logger().info(
            f"Publishing TF {self.map_frame} -> {self.child_frame} from /R1/fcu/state (yaw_mode={self.yaw_mode})"
        )

    def cb(self, msg: UAVState):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = float(msg.position.x)
        t.transform.translation.y = float(msg.position.y)
        t.transform.translation.z = float(msg.position.z)

        az = float(msg.azimuth)
        if self.yaw_mode == "pi2_minus_azimuth":
            yaw = (math.pi / 2.0) - az
        else:
            yaw = az

        qx, qy, qz, qw = quat_from_yaw(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = StateToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
