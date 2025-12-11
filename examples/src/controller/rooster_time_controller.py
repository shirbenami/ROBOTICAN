#!/usr/bin/env python3
"""
Time-Based Yaw Controller
-------------------------
Simple rotation controller using timed force application.
Phase 1: Stabilize (5s) - Z=200, R=0
Phase 2: Rotate (4s)    - Z=220, R=±800
Phase 3: Settle (1s)    - Z=200, R=±100
"""

import time
import threading
from rclpy.node import Node

from fcu_driver_interfaces.msg import ManualControl, UAVState
from rooster_handler_interfaces.msg import KeepAlive
from std_srvs.srv import SetBool


class TimeYawController(Node):
    def __init__(self, rooster_id: str = "R1", name: str = "time_yaw_controller"):
        super().__init__(name)
        self.rooster_id = rooster_id

        # State
        self._lock = threading.Lock()
        self._armed = False
        self._airborne = False
        self._state_received = False

        # Control outputs
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_r = 0.0
        self.flight_mode = 1

        # Time-based rotation parameters (for 90 degrees)
        self.stabilize_time = 5.0
        self.stabilize_z = 200.0
        self.rotate_time = 4.0
        self.rotate_z = 220.0
        self.rotate_r = 800.0
        self.settle_time = 1.0
        self.settle_z = 200.0
        self.settle_r = 100.0

        # ROS setup
        self.state_sub = self.create_subscription(
            UAVState, f"/{rooster_id}/fcu/state", self._state_cb, 10)
        self.manual_pub = self.create_publisher(
            ManualControl, f"/{rooster_id}/manual_control", 10)
        self.keep_alive_pub = self.create_publisher(
            KeepAlive, f"/{rooster_id}/keep_alive", 10)
        self.force_arm_client = self.create_client(
            SetBool, f"/{rooster_id}/fcu/command/force_arm")

        self.create_timer(1.0 / 40.0, self._publish_control)
        self.create_timer(1.0, self._publish_keepalive)

        self.get_logger().info(f"TimeYawController ready for {rooster_id}")

    def _state_cb(self, msg: UAVState):
        with self._lock:
            self._armed = msg.armed
            self._airborne = msg.airborne
            self._state_received = True

    def _publish_control(self):
        msg = ManualControl()
        msg.x = self.current_x
        msg.y = self.current_y
        msg.z = self.current_z
        msg.r = self.current_r
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def _publish_keepalive(self):
        msg = KeepAlive()
        msg.is_active = True
        msg.requested_flight_mode = self.flight_mode
        msg.command_reboot = False
        self.keep_alive_pub.publish(msg)

    @property
    def is_armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def is_airborne(self) -> bool:
        with self._lock:
            return self._airborne

    def rotate(self, degrees: float, timeout: float = 30.0, hover_throttle: float = 200.0) -> bool:
        """
        Rotate using time-based approach.
        degrees > 0: clockwise (R positive)
        degrees < 0: counterclockwise (R negative)
        """
        direction = 1.0 if degrees > 0 else -1.0
        self.get_logger().info(f"Time-based rotate: {degrees:+.0f}° ({'CW' if direction > 0 else 'CCW'})")

        # Wait for state
        t0 = time.time()
        while not self._state_received and time.time() - t0 < 5.0:
            time.sleep(0.05)

        # Phase 1: Stabilize
        self.get_logger().info(f"[STABILIZE] {self.stabilize_time}s - Z={self.stabilize_z}")
        self.current_z = self.stabilize_z
        self.current_r = 0.0
        time.sleep(self.stabilize_time)

        # Phase 2: Rotate
        rotate_r = self.rotate_r * direction
        self.get_logger().info(f"[ROTATE] {self.rotate_time}s - Z={self.rotate_z}, R={rotate_r}")
        self.current_z = self.rotate_z
        self.current_r = rotate_r
        time.sleep(self.rotate_time)

        # Phase 3: Settle
        settle_r = self.settle_r * direction
        self.get_logger().info(f"[SETTLE] {self.settle_time}s - Z={self.settle_z}, R={settle_r}")
        self.current_z = self.settle_z
        self.current_r = settle_r
        time.sleep(self.settle_time)

        # Stop
        self._stop()
        self.get_logger().info("✓ Rotation complete")
        return True

    def _stop(self):
        """Stop all movement."""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_r = 0.0
        time.sleep(0.05)