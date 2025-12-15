#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from fcu_driver_interfaces.msg import UAVState  # adjust package name if different


class R2UavStateListener(Node):
    def __init__(self):
        super().__init__("r2_uav_state_listener")

        # Subscribe to /R2/fcu/state
        self.sub = self.create_subscription(
            UAVState,
            "/R2/fcu/state",
            self.state_callback,
            10,
        )

        self.get_logger().info("R2UavStateListener started, listening to /R2/fcu/state")

    def state_callback(self, msg: UAVState):
        # Position in ENU frame relative to origin
        pos = msg.position

        # Azimuth in radians from the message
        azimuth_rad = msg.azimuth
        azimuth_deg = math.degrees(azimuth_rad)

        if msg.valid_position:
            self.get_logger().info(
                f"R2: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) m, "
                f"azimuth={azimuth_rad:.3f} rad ({azimuth_deg:.1f} deg)"
            )
        else:
            self.get_logger().warn(
                f"R2: position NOT valid yet, azimuth={azimuth_rad:.3f} rad "
                f"({azimuth_deg:.1f} deg)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = R2UavStateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
