#!/usr/bin/env python3
"""
Turn Super Agent - Interactive
---------------------------------
Supports both yaw-based and time-based controllers via --controller argument.

Usage:
  python super_agent.py --controller time --degrees 90
  python super_agent.py --controller yaw --degrees 45
"""

import signal
import sys
import time
import threading
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_srvs.srv import Trigger, SetBool


class TurnSuperAgent(Node):
    def __init__(self, rooster_id: str = "R1", hover_throttle: float = 200.0,
                 controller_type: str = "time", rotation_degrees: float = 90.0):
        super().__init__(f"turn_super_agent_{rooster_id}")

        self.rooster_id = rooster_id
        self.hover_throttle = hover_throttle
        self.rotation_degrees = rotation_degrees
        self._shutdown_requested = False
        self._operation_in_progress = False
        self._lock = threading.Lock()

        # Create controller based on type
        if controller_type == "time":
            from controller.rooster_time_controller import TimeYawController
            self.yaw_ctrl = TimeYawController(
                rooster_id=rooster_id,
                name=f"super_agent_time_{rooster_id}"
            )
            self.get_logger().info("Using TIME-based controller")
        else:
            from controller.rooster_yaw_controller import YawController
            self.yaw_ctrl = YawController(
                rooster_id=rooster_id,
                name=f"super_agent_yaw_{rooster_id}"
            )
            self.get_logger().info("Using YAW-based controller")

        # Callback groups
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.stop_cb_group = ReentrantCallbackGroup()

        # Services
        self.srv_turn_right = self.create_service(
            Trigger, f"/{rooster_id}/turn_right", self.handle_turn_right,
            callback_group=self.service_cb_group)
        self.srv_turn_left = self.create_service(
            Trigger, f"/{rooster_id}/turn_left", self.handle_turn_left,
            callback_group=self.service_cb_group)
        self.srv_stop = self.create_service(
            Trigger, f"/{rooster_id}/stop", self.handle_stop,
            callback_group=self.stop_cb_group)

        self.get_logger().info(f"TurnSuperAgent ready for {rooster_id} ({rotation_degrees}° rotation)")

    def handle_turn_right(self, request, response):
        success, message = self.execute_turn_right()
        response.success = success
        response.message = message
        return response

    def handle_turn_left(self, request, response):
        success, message = self.execute_turn_left()
        response.success = success
        response.message = message
        return response

    def handle_stop(self, request, response):
        self.get_logger().warn(f"[{self.rooster_id}] EMERGENCY STOP!")
        self.emergency_stop()
        response.success = True
        response.message = "Emergency stop executed"
        return response

    def ensure_armed_and_airborne(self) -> bool:
        """Ensure drone is armed and airborne."""
        if not self.yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("force_arm service not available!")
            return False

        # Zero throttle before arming
        self.yaw_ctrl.current_x = 0.0
        self.yaw_ctrl.current_y = 0.0
        self.yaw_ctrl.current_z = 0.0
        self.yaw_ctrl.current_r = 0.0
        time.sleep(0.2)

        # Arm if needed
        if not self.yaw_ctrl.is_armed:
            self.get_logger().info(f"[{self.rooster_id}] Arming...")
            req = SetBool.Request()
            req.data = True
            future = self.yaw_ctrl.force_arm_client.call_async(req)

            start = time.time()
            while not future.done() and time.time() - start < 3.0:
                time.sleep(0.05)

            # Wait for armed state
            start = time.time()
            while not self.yaw_ctrl.is_armed and time.time() - start < 2.0:
                time.sleep(0.05)

            if not self.yaw_ctrl.is_armed:
                self.get_logger().error("Failed to arm!")
                return False

        # Apply throttle
        self.yaw_ctrl.current_z = self.hover_throttle

        # Wait for airborne
        start = time.time()
        while not self.yaw_ctrl.is_airborne and time.time() - start < 5.0:
            if not self.yaw_ctrl.is_armed:
                req = SetBool.Request()
                req.data = True
                self.yaw_ctrl.force_arm_client.call_async(req)
                time.sleep(0.2)
            time.sleep(0.1)

        return self.yaw_ctrl.is_armed

    def execute_turn_right(self) -> tuple:
        with self._lock:
            if self._shutdown_requested:
                return False, "Shutdown in progress"
            if self._operation_in_progress:
                return False, "Operation in progress"
            self._operation_in_progress = True

        try:
            self.get_logger().info(f"[{self.rooster_id}] Executing {self.rotation_degrees}° RIGHT...")
            if not self.ensure_armed_and_airborne():
                return False, "Failed to arm"
            success = self.yaw_ctrl.rotate(self.rotation_degrees, hover_throttle=self.yaw_ctrl.current_z)
            return success, "Turn right complete" if success else "Turn failed"
        finally:
            with self._lock:
                self._operation_in_progress = False

    def execute_turn_left(self) -> tuple:
        with self._lock:
            if self._shutdown_requested:
                return False, "Shutdown in progress"
            if self._operation_in_progress:
                return False, "Operation in progress"
            self._operation_in_progress = True

        try:
            self.get_logger().info(f"[{self.rooster_id}] Executing {self.rotation_degrees}° LEFT...")
            if not self.ensure_armed_and_airborne():
                return False, "Failed to arm"
            success = self.yaw_ctrl.rotate(-self.rotation_degrees, hover_throttle=self.yaw_ctrl.current_z)
            return success, "Turn left complete" if success else "Turn left failed"
        finally:
            with self._lock:
                self._operation_in_progress = False

    def emergency_stop(self):
        self.get_logger().warn(f"[{self.rooster_id}] === EMERGENCY STOP ===")
        self.yaw_ctrl.current_x = 0.0
        self.yaw_ctrl.current_y = 0.0
        self.yaw_ctrl.current_z = 0.0
        self.yaw_ctrl.current_r = 0.0
        time.sleep(0.1)
        self._disarm()

    def _disarm(self):
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

    def shutdown(self):
        with self._lock:
            if self._shutdown_requested:
                return
            self._shutdown_requested = True
        self.get_logger().warn(f"[{self.rooster_id}] Shutdown...")
        self.emergency_stop()


def print_menu(rooster_id: str, degrees: float):
    print("\n" + "=" * 50)
    print(f"  DRONE CONTROL - {rooster_id}")
    print("=" * 50)
    print(f"  1. Rotate RIGHT ({degrees}° clockwise)")
    print(f"  2. Rotate LEFT  ({degrees}° counterclockwise)")
    print("  3. STOP ALL & Disarm")
    print("-" * 50)
    print("  Press CTRL+C for emergency stop")
    print("=" * 50)


def interactive_menu(node: TurnSuperAgent, shutdown_event: threading.Event):
    time.sleep(1.0)
    print_menu(node.rooster_id, node.rotation_degrees)

    while not shutdown_event.is_set():
        try:
            choice = input("\nEnter choice [1-3]: ").strip()
            if shutdown_event.is_set():
                break

            if choice == "1":
                print("\n>>> Rotating RIGHT...")
                success, msg = node.execute_turn_right()
                print(f"{'✓' if success else '✗'} {msg}")
                print_menu(node.rooster_id, node.rotation_degrees)
            elif choice == "2":
                print("\n>>> Rotating LEFT...")
                success, msg = node.execute_turn_left()
                print(f"{'✓' if success else '✗'} {msg}")
                print_menu(node.rooster_id, node.rotation_degrees)
            elif choice == "3":
                print("\n>>> STOPPING...")
                node.emergency_stop()
                print("✓ Stopped and disarmed")
                print_menu(node.rooster_id, node.rotation_degrees)
            elif choice:
                print(f"✗ Invalid: '{choice}'")
        except (EOFError, KeyboardInterrupt):
            break


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Turn Super Agent")
    parser.add_argument("--rooster", "-r", type=str, default="R1",
                        help="Rooster ID (R1, R2, R3)")
    parser.add_argument("--throttle", "-t", type=float, default=250.0,
                        help="Hover throttle (-1000 to 1000)")
    parser.add_argument("--controller", "-c", type=str, default="time",
                        choices=["time", "yaw"],
                        help="Controller type: 'time' (time-based) or 'yaw' (azimuth-based)")
    parser.add_argument("--degrees", "-d", type=float, default=90.0,
                        help="Rotation degrees")
    args = parser.parse_args()

    rclpy.init()

    node = TurnSuperAgent(
        rooster_id=args.rooster,
        hover_throttle=args.throttle,
        controller_type=args.controller,
        rotation_degrees=args.degrees
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node.yaw_ctrl)

    shutdown_event = threading.Event()
    shutdown_started = [False]

    def signal_handler(signum, frame):
        if shutdown_started[0]:
            os._exit(1)
        shutdown_started[0] = True
        print("\n\n[CTRL+C] Emergency stop...")
        node.shutdown()
        shutdown_event.set()
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        interactive_menu(node, shutdown_event)
    except (KeyboardInterrupt, EOFError):
        if not shutdown_started[0]:
            node.shutdown()
            shutdown_event.set()

    print("\nCleaning up...")
    time.sleep(0.3)
    try:
        node.destroy_node()
        node.yaw_ctrl.destroy_node()
    except:
        pass
    try:
        rclpy.shutdown()
    except:
        pass
    print("Done.")
    sys.exit(0)


if __name__ == "__main__":
    main()