#!/usr/bin/env python3
"""
Turn Left Agent
---------------------------------
Rotates drone 90° counterclockwise.
Service: /{rooster_id}/turn_left

Automatically arms the drone if not armed.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Trigger, SetBool

from rooster_yaw_controller import YawController


class TurnLeftAgent(Node):
    def __init__(self, rooster_id: str = "R1", hover_throttle: float = 200.0):
        super().__init__(f"turn_left_agent_{rooster_id}")

        self.rooster_id = rooster_id
        self.hover_throttle = hover_throttle

        # Create yaw controller for this drone
        self.yaw_ctrl = YawController(
            rooster_id=rooster_id,
            name=f"turn_left_yaw_{rooster_id}"
        )

        # Create service
        service_name = f"/{rooster_id}/turn_left"
        self.srv = self.create_service(
            Trigger,
            service_name,
            self.handle_request,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_logger().info(f"TurnLeftAgent ready for {rooster_id} (90° counterclockwise)")

    def ensure_armed_and_airborne(self) -> bool:
        """Ensure the drone is armed and airborne. Returns True if ready."""
        # Check if already armed
        if not self.yaw_ctrl.is_armed:
            self.get_logger().info(f"[{self.rooster_id}] Drone not armed, arming...")

            if not self.yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("force_arm service not available!")
                return False

            req = SetBool.Request()
            req.data = True
            future = self.yaw_ctrl.force_arm_client.call_async(req)

            # Wait for arm response
            start = time.time()
            while not future.done() and time.time() - start < 3.0:
                time.sleep(0.05)

            if future.done():
                try:
                    result = future.result()
                    self.get_logger().info(f"Arm result: {result.message}")
                except Exception as e:
                    self.get_logger().error(f"Arm exception: {e}")
                    return False

            # Wait for arming to take effect
            time.sleep(1.0)

        # Apply throttle to get airborne
        if not self.yaw_ctrl.is_airborne:
            self.get_logger().info(f"[{self.rooster_id}] Applying throttle to get airborne...")
            self.yaw_ctrl.current_z = self.hover_throttle

            # Wait to become airborne
            start = time.time()
            while not self.yaw_ctrl.is_airborne and time.time() - start < 5.0:
                time.sleep(0.1)

            if not self.yaw_ctrl.is_airborne:
                self.get_logger().warn("Failed to become airborne, trying higher throttle...")
                self.yaw_ctrl.current_z = 400.0
                time.sleep(2.0)

        self.get_logger().info(
            f"[{self.rooster_id}] Armed: {self.yaw_ctrl.is_armed}, Airborne: {self.yaw_ctrl.is_airborne}")
        return self.yaw_ctrl.is_armed

    def _disarm(self):
        """Disarm the drone."""
        self.get_logger().info(f"[{self.rooster_id}] Disarming...")

        # Stop all movement first
        self.yaw_ctrl.current_x = 0.0
        self.yaw_ctrl.current_y = 0.0
        self.yaw_ctrl.current_z = 0.0
        self.yaw_ctrl.current_r = 0.0
        time.sleep(0.1)

        if not self.yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("force_arm service not available for disarm!")
            return False

        req = SetBool.Request()
        req.data = False
        future = self.yaw_ctrl.force_arm_client.call_async(req)

        start = time.time()
        while not future.done() and time.time() - start < 2.0:
            time.sleep(0.05)

        if future.done():
            try:
                result = future.result()
                self.get_logger().info(f"[{self.rooster_id}] Disarm: {result.message}")
            except Exception as e:
                self.get_logger().error(f"Disarm exception: {e}")
        return True

    def handle_request(self, request, response):
        self.get_logger().info(f"[{self.rooster_id}] Executing 90° left turn...")

        # Ensure drone is armed and airborne
        if not self.ensure_armed_and_airborne():
            response.success = False
            response.message = "Failed to arm drone"
            return response

        # Perform rotation
        success = self.yaw_ctrl.rotate(-90, hover_throttle=self.yaw_ctrl.current_z)

        # Disarm after rotation
        self._disarm()

        response.success = success
        response.message = "Turn left complete" if success else "Turn failed"

        return response


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Turn Left Agent")
    parser.add_argument("--rooster", "-r", type=str, default="R1",
                        help="Rooster ID (R1, R2, or R3)")
    parser.add_argument("--throttle", "-t", type=float, default=200.0,
                        help="Hover throttle (z value, range -1000 to 1000)")
    args = parser.parse_args()

    rclpy.init()

    node = TurnLeftAgent(rooster_id=args.rooster, hover_throttle=args.throttle)

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