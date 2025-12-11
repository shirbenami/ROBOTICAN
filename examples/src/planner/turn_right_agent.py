#!/usr/bin/env python3
"""
Turn Right Agent
---------------------------------
Rotates drone clockwise (default 90°).
Supports time-based or yaw-based controller.

Usage:
  python turn_right_agent.py --controller time --degrees 90
  python turn_right_agent.py --controller yaw --degrees 45
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Trigger, SetBool


class TurnRightAgent(Node):
    def __init__(self, rooster_id: str = "R1", hover_throttle: float = 200.0,
                 controller_type: str = "time", rotation_degrees: float = 90.0):
        super().__init__(f"turn_right_agent_{rooster_id}")

        self.rooster_id = rooster_id
        self.hover_throttle = hover_throttle
        self.rotation_degrees = rotation_degrees

        # Create controller based on type
        if controller_type == "time":
            from controller.rooster_time_controller import TimeYawController
            self.yaw_ctrl = TimeYawController(
                rooster_id=rooster_id,
                name=f"turn_right_time_{rooster_id}"
            )
            self.get_logger().info("Using TIME-based controller")
        else:
            from controller.rooster_yaw_controller import YawController
            self.yaw_ctrl = YawController(
                rooster_id=rooster_id,
                name=f"turn_right_yaw_{rooster_id}"
            )
            self.get_logger().info("Using YAW-based controller")

        # Create service
        self.srv = self.create_service(
            Trigger,
            f"/{rooster_id}/turn_right",
            self.handle_request,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_logger().info(f"TurnRightAgent ready for {rooster_id} ({rotation_degrees}° CW)")

    def ensure_armed_and_airborne(self) -> bool:
        """Ensure drone is armed and airborne."""
        if not self.yaw_ctrl.is_armed:
            self.get_logger().info(f"[{self.rooster_id}] Arming...")

            if not self.yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("force_arm service not available!")
                return False

            req = SetBool.Request()
            req.data = True
            future = self.yaw_ctrl.force_arm_client.call_async(req)

            start = time.time()
            while not future.done() and time.time() - start < 3.0:
                time.sleep(0.05)

            time.sleep(1.0)

        if not self.yaw_ctrl.is_airborne:
            self.get_logger().info(f"[{self.rooster_id}] Applying throttle...")
            self.yaw_ctrl.current_z = self.hover_throttle

            start = time.time()
            while not self.yaw_ctrl.is_airborne and time.time() - start < 5.0:
                time.sleep(0.1)

            if not self.yaw_ctrl.is_airborne:
                self.yaw_ctrl.current_z = 400.0
                time.sleep(2.0)

        return self.yaw_ctrl.is_armed

    def _disarm(self):
        """Disarm the drone."""
        self.yaw_ctrl.current_x = 0.0
        self.yaw_ctrl.current_y = 0.0
        self.yaw_ctrl.current_z = 0.0
        self.yaw_ctrl.current_r = 0.0
        time.sleep(0.1)

        if not self.yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=1.0):
            return False

        req = SetBool.Request()
        req.data = False
        future = self.yaw_ctrl.force_arm_client.call_async(req)

        start = time.time()
        while not future.done() and time.time() - start < 2.0:
            time.sleep(0.05)

        self.get_logger().info(f"[{self.rooster_id}] Disarmed")
        return True

    def handle_request(self, request, response):
        self.get_logger().info(f"[{self.rooster_id}] Executing {self.rotation_degrees}° right turn...")

        if not self.ensure_armed_and_airborne():
            response.success = False
            response.message = "Failed to arm drone"
            return response

        success = self.yaw_ctrl.rotate(self.rotation_degrees, hover_throttle=self.yaw_ctrl.current_z)
        self._disarm()

        response.success = success
        response.message = "Turn right complete" if success else "Turn failed"
        return response


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Turn Right Agent")
    parser.add_argument("--rooster", "-r", type=str, default="R1",
                        help="Rooster ID (R1, R2, R3)")
    parser.add_argument("--throttle", "-t", type=float, default=200.0,
                        help="Hover throttle (-1000 to 1000)")
    parser.add_argument("--controller", "-c", type=str, default="time",
                        choices=["time", "yaw"],
                        help="Controller type: 'time' or 'yaw'")
    parser.add_argument("--degrees", "-d", type=float, default=90.0,
                        help="Rotation degrees")
    args = parser.parse_args()

    rclpy.init()

    node = TurnRightAgent(
        rooster_id=args.rooster,
        hover_throttle=args.throttle,
        controller_type=args.controller,
        rotation_degrees=args.degrees
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(node.yaw_ctrl)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.yaw_ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()