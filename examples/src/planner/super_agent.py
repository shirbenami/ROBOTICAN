#!/usr/bin/env python3
"""
Turn Super Agent - Interactive
---------------------------------
Orchestrates turn left/right operations for a drone with interactive menu.
Handles graceful shutdown with immediate stop and disarm on CTRL+C.

Interactive Menu:
  1. Rotate right (45° CW)
  2. Rotate left (45° CCW)
  3. Stop all & disarm

Services (also available):
  /{rooster_id}/turn_right  - Rotate 45° clockwise
  /{rooster_id}/turn_left   - Rotate 45° counterclockwise
  /{rooster_id}/stop        - Emergency stop and disarm
"""

import signal
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_srvs.srv import Trigger, SetBool

from controller.rooster_yaw_controller import YawController


class TurnSuperAgent(Node):
    def __init__(self, rooster_id: str = "R1", hover_throttle: float = 200.0):
        super().__init__(f"turn_super_agent_{rooster_id}")

        self.rooster_id = rooster_id
        self.hover_throttle = hover_throttle
        self._shutdown_requested = False
        self._operation_in_progress = False
        self._lock = threading.Lock()

        # Create yaw controller for this drone
        self.yaw_ctrl = YawController(
            rooster_id=rooster_id,
            name=f"super_agent_yaw_{rooster_id}"
        )

        # Callback groups for concurrent service handling
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.stop_cb_group = ReentrantCallbackGroup()

        # Create services
        self.srv_turn_right = self.create_service(
            Trigger,
            f"/{rooster_id}/turn_right",
            self.handle_turn_right,
            callback_group=self.service_cb_group
        )

        self.srv_turn_left = self.create_service(
            Trigger,
            f"/{rooster_id}/turn_left",
            self.handle_turn_left,
            callback_group=self.service_cb_group
        )

        self.srv_stop = self.create_service(
            Trigger,
            f"/{rooster_id}/stop",
            self.handle_stop,
            callback_group=self.stop_cb_group
        )

        self.get_logger().info(f"TurnSuperAgent ready for {rooster_id}")

    def handle_turn_right(self, request, response):
        """Handle turn right service request."""
        success, message = self.execute_turn_right()
        response.success = success
        response.message = message
        return response

    def handle_turn_left(self, request, response):
        """Handle turn left service request."""
        success, message = self.execute_turn_left()
        response.success = success
        response.message = message
        return response

    def handle_stop(self, request, response):
        """Handle emergency stop service request."""
        self.get_logger().warn(f"[{self.rooster_id}] EMERGENCY STOP requested!")
        self.emergency_stop()
        response.success = True
        response.message = "Emergency stop executed"
        return response

    def ensure_armed_and_airborne(self) -> bool:
        """Ensure the drone is armed and airborne. Returns True if ready."""

        if not self.yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("force_arm service not available!")
            return False

        # STEP 1: Ensure throttle is at ZERO before arming (required by most FCUs)
        self.get_logger().info(f"[{self.rooster_id}] Setting throttle to zero before arming...")
        self.yaw_ctrl.current_x = 0.0
        self.yaw_ctrl.current_y = 0.0
        self.yaw_ctrl.current_z = 0.0
        self.yaw_ctrl.current_r = 0.0
        time.sleep(0.2)  # Let zero-throttle command propagate

        # STEP 2: Arm the drone
        if not self.yaw_ctrl.is_armed:
            self.get_logger().info(f"[{self.rooster_id}] Arming drone...")

            req = SetBool.Request()
            req.data = True
            future = self.yaw_ctrl.force_arm_client.call_async(req)

            # Wait for arm service response
            start = time.time()
            while not future.done() and time.time() - start < 3.0:
                time.sleep(0.05)

            if future.done():
                try:
                    result = future.result()
                    self.get_logger().info(f"Arm service result: {result.message}")
                except Exception as e:
                    self.get_logger().error(f"Arm exception: {e}")
                    return False

            # STEP 3: Wait for armed STATE to be confirmed (not just service response)
            self.get_logger().info(f"[{self.rooster_id}] Waiting for armed state confirmation...")
            start = time.time()
            while not self.yaw_ctrl.is_armed and time.time() - start < 2.0:
                time.sleep(0.05)

            if not self.yaw_ctrl.is_armed:
                self.get_logger().error(f"[{self.rooster_id}] Failed to confirm armed state!")
                return False

            self.get_logger().info(f"[{self.rooster_id}] Armed state confirmed!")

        # STEP 4: NOW apply throttle (after armed state is confirmed)
        self.get_logger().info(f"[{self.rooster_id}] Applying throttle: {self.hover_throttle}")
        self.yaw_ctrl.current_z = self.hover_throttle

        # STEP 5: Wait to become airborne
        self.get_logger().info(f"[{self.rooster_id}] Waiting to become airborne...")
        start = time.time()
        while not self.yaw_ctrl.is_airborne and time.time() - start < 5.0:
            # Check if we lost armed status
            if not self.yaw_ctrl.is_armed:
                self.get_logger().warn(f"[{self.rooster_id}] Lost armed status! Throttle may be too low.")
                # Try to re-arm while keeping throttle
                req = SetBool.Request()
                req.data = True
                self.yaw_ctrl.force_arm_client.call_async(req)
                time.sleep(0.2)
            time.sleep(0.1)

        if not self.yaw_ctrl.is_airborne:
            self.get_logger().warn(f"[{self.rooster_id}] Not airborne yet, trying higher throttle (400)...")
            self.yaw_ctrl.current_z = 400.0

            start = time.time()
            while not self.yaw_ctrl.is_airborne and time.time() - start < 3.0:
                if not self.yaw_ctrl.is_armed:
                    req = SetBool.Request()
                    req.data = True
                    self.yaw_ctrl.force_arm_client.call_async(req)
                    time.sleep(0.2)
                time.sleep(0.1)

        self.get_logger().info(
            f"[{self.rooster_id}] Final status - Armed: {self.yaw_ctrl.is_armed}, Airborne: {self.yaw_ctrl.is_airborne}")
        return self.yaw_ctrl.is_armed

    def execute_turn_right(self) -> tuple:
        """Execute turn right operation. Returns (success, message)."""
        with self._lock:
            if self._shutdown_requested:
                return False, "Shutdown in progress"
            if self._operation_in_progress:
                return False, "Another operation in progress"
            self._operation_in_progress = True

        try:
            self.get_logger().info(f"[{self.rooster_id}] Executing 45° RIGHT turn...")

            # Ensure drone is armed and airborne
            if not self.ensure_armed_and_airborne():
                return False, "Failed to arm drone"

            success = self.yaw_ctrl.rotate(45, hover_throttle=self.yaw_ctrl.current_z)
            message = "Turn right complete" if success else "Turn right failed"
            return success, message
        except Exception as e:
            return False, f"Error: {str(e)}"
        finally:
            with self._lock:
                self._operation_in_progress = False

    def execute_turn_left(self) -> tuple:
        """Execute turn left operation. Returns (success, message)."""
        with self._lock:
            if self._shutdown_requested:
                return False, "Shutdown in progress"
            if self._operation_in_progress:
                return False, "Another operation in progress"
            self._operation_in_progress = True

        try:
            self.get_logger().info(f"[{self.rooster_id}] Executing 45° LEFT turn...")

            # Ensure drone is armed and airborne
            if not self.ensure_armed_and_airborne():
                return False, "Failed to arm drone"

            success = self.yaw_ctrl.rotate(-45, hover_throttle=self.yaw_ctrl.current_z)
            message = "Turn left complete" if success else "Turn left failed"
            return success, message
        except Exception as e:
            return False, f"Error: {str(e)}"
        finally:
            with self._lock:
                self._operation_in_progress = False

    def emergency_stop(self):
        """Immediately stop all movement and disarm the drone."""
        self.get_logger().warn(f"[{self.rooster_id}] === EMERGENCY STOP ===")

        # Stop all control outputs immediately
        self.yaw_ctrl.current_x = 0.0
        self.yaw_ctrl.current_y = 0.0
        self.yaw_ctrl.current_z = 0.0
        self.yaw_ctrl.current_r = 0.0

        # Give time for stop command to be published
        time.sleep(0.1)

        # Disarm the drone
        self.get_logger().warn(f"[{self.rooster_id}] Disarming drone...")
        self._disarm()

    def _disarm(self):
        """Disarm the drone."""
        if not self.yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("force_arm service not available for disarm!")
            return False

        req = SetBool.Request()
        req.data = False

        future = self.yaw_ctrl.force_arm_client.call_async(req)

        # Wait for disarm to complete (with timeout)
        start = time.time()
        while not future.done() and time.time() - start < 2.0:
            time.sleep(0.05)

        if future.done():
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info(f"[{self.rooster_id}] Disarmed successfully")
                else:
                    self.get_logger().warn(f"[{self.rooster_id}] Disarm returned: {result.message}")
            except Exception as e:
                self.get_logger().error(f"[{self.rooster_id}] Disarm exception: {e}")
        else:
            self.get_logger().warn(f"[{self.rooster_id}] Disarm timed out")

        return True

    def shutdown(self):
        """Graceful shutdown handler."""
        with self._lock:
            if self._shutdown_requested:
                return
            self._shutdown_requested = True

        self.get_logger().warn(f"[{self.rooster_id}] Shutdown initiated...")
        self.emergency_stop()


def print_menu(rooster_id: str):
    """Print the interactive menu."""
    print("\n" + "=" * 50)
    print(f"  DRONE CONTROL - {rooster_id}")
    print("=" * 50)
    print("  1. Rotate RIGHT (45° clockwise)")
    print("  2. Rotate LEFT  (45° counterclockwise)")
    print("  3. STOP ALL & Disarm")
    print("-" * 50)
    print("  Press CTRL+C for emergency stop")
    print("=" * 50)


def interactive_menu(node: TurnSuperAgent, shutdown_event: threading.Event):
    """Run interactive menu in a loop."""
    rooster_id = node.rooster_id

    # Wait a moment for ROS to initialize
    time.sleep(1.0)

    print_menu(rooster_id)

    while not shutdown_event.is_set():
        try:
            # Simple blocking input
            choice = input("\nEnter choice [1-3]: ").strip()

            if shutdown_event.is_set():
                break

            if choice == "1":
                print("\n>>> Rotating RIGHT...")
                success, message = node.execute_turn_right()
                if success:
                    print(f"✓ {message}")
                else:
                    print(f"✗ {message}")
                print_menu(rooster_id)

            elif choice == "2":
                print("\n>>> Rotating LEFT...")
                success, message = node.execute_turn_left()
                if success:
                    print(f"✓ {message}")
                else:
                    print(f"✗ {message}")
                print_menu(rooster_id)

            elif choice == "3":
                print("\n>>> STOPPING AND DISARMING...")
                node.emergency_stop()
                print("✓ Stopped and disarmed")
                print_menu(rooster_id)

            elif choice == "":
                # Empty input, just show prompt again
                continue

            else:
                print(f"✗ Invalid choice: '{choice}'. Please enter 1, 2, or 3.")

        except (EOFError, KeyboardInterrupt):
            # Exit the loop on CTRL+C or EOF
            break
        except Exception as e:
            if not shutdown_event.is_set():
                print(f"\nInput error: {e}")


def main():
    import argparse
    import os

    parser = argparse.ArgumentParser(description="Turn Super Agent - Interactive")
    parser.add_argument("--rooster", "-r", type=str, default="R1",
                        help="Rooster ID (R1, R1, or R3)")
    parser.add_argument("--throttle", "-t", type=float, default=250.0,
                        help="Hover throttle (z value, range -1000 to 1000)")
    args = parser.parse_args()

    rclpy.init()

    node = TurnSuperAgent(rooster_id=args.rooster, hover_throttle=args.throttle)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node.yaw_ctrl)

    # Setup signal handlers for graceful shutdown
    shutdown_event = threading.Event()
    shutdown_started = [False]  # Use list to allow modification in nested function

    def signal_handler(signum, frame):
        if shutdown_started[0]:
            # Already shutting down, force exit
            print("\nForce exit...")
            os._exit(1)

        shutdown_started[0] = True
        print(f"\n\n[CTRL+C] Emergency stop initiated...")
        node.shutdown()
        shutdown_event.set()

        # Restore default handler for next CTRL+C to force exit
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Spin ROS in a separate thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run interactive menu in main thread
    try:
        interactive_menu(node, shutdown_event)
    except (KeyboardInterrupt, EOFError):
        if not shutdown_started[0]:
            print("\n\n[CTRL+C] Emergency stop initiated...")
            node.shutdown()
            shutdown_event.set()

    # Cleanup
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

    print("Shutdown complete.")
    sys.exit(0)


if __name__ == "__main__":
    main()