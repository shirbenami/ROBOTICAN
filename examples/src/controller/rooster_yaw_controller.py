#!/usr/bin/env python3
"""
Two-Phase Continuous Yaw Controller
------------------------------------
Phase 1: Fast rotation (300 force) until close to target
Phase 2: Slow rotation (200 force) to precisely reach target
"""

import math
import time
import threading
from rclpy.node import Node

from fcu_driver_interfaces.msg import ManualControl, UAVState
from rooster_handler_interfaces.msg import KeepAlive
from std_srvs.srv import SetBool


def normalize_angle(a: float) -> float:
    """Normalize angle to [-π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def to_360(rad: float) -> float:
    """Convert radians to 0-360 degrees."""
    deg = math.degrees(rad)
    return deg % 360


class YawController(Node):
    def __init__(self, rooster_id: str = "R2", name: str = "yaw_controller"):
        super().__init__(name)
        self.rooster_id = rooster_id

        # State
        self._lock = threading.Lock()
        self._azimuth = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_z = 0.0
        self._state_received = False
        self._armed = False
        self._airborne = False

        # Control outputs
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_r = 0.0
        self.flight_mode = 1

        # Phase 1: Fast
        self.fast_force = 300.0
        self.fast_threshold = math.radians(15.0)  # Switch to slow when within 15°

        # Phase 2: Slow
        self.slow_force = 200.0
        self.slow_threshold = math.radians(2.0)   # Stop when within 2° and coast

        # Completion
        self.tolerance = math.radians(1.0)
        self.stable_samples = 5

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

        self.get_logger().info(f"YawController ready for {rooster_id}")

    def _state_cb(self, msg: UAVState):
        with self._lock:
            self._azimuth = msg.azimuth
            self._pos_x = msg.position.x
            self._pos_y = msg.position.y
            self._pos_z = msg.position.z
            self._armed = msg.armed
            self._airborne = msg.airborne
            self._state_received = True

    def _publish_control(self):
        msg = ManualControl()
        msg.x, msg.y, msg.z, msg.r = self.current_x, self.current_y, self.current_z, self.current_r
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def _publish_keepalive(self):
        msg = KeepAlive()
        msg.is_active = True
        msg.requested_flight_mode = self.flight_mode
        msg.command_reboot = False
        self.keep_alive_pub.publish(msg)

    @property
    def yaw(self) -> float:
        with self._lock:
            return self._azimuth

    @property
    def position(self) -> tuple:
        with self._lock:
            return (self._pos_x, self._pos_y, self._pos_z)

    @property
    def is_armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def is_airborne(self) -> bool:
        with self._lock:
            return self._airborne

    def arm(self, arm: bool = True) -> bool:
        if not self.force_arm_client.service_is_ready():
            self.get_logger().warn("force_arm service not ready")
            return False
        req = SetBool.Request()
        req.data = arm
        self.force_arm_client.call_async(req)
        return True

    def rotate(self, degrees: float, timeout: float = 30.0, hover_throttle: float = 500.0) -> bool:
        """Rotate using two-phase continuous movement."""
        self.get_logger().info(f"Rotate {degrees:+.1f}° requested")

        # Wait for state
        t0 = time.time()
        while not self._state_received and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self._state_received:
            self.get_logger().error("No UAVState received!")
            return False

        # Calculate target
        initial_yaw = self.yaw
        target_yaw = normalize_angle(initial_yaw + math.radians(degrees))

        self.get_logger().info(f"Rotating: {math.degrees(initial_yaw):.1f}° → {math.degrees(target_yaw):.1f}°")

        # Log initial state
        pos = self.position
        self.get_logger().info(
            f"[START] pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) angle={to_360(initial_yaw):.1f}°"
        )

        # Set hover
        self.current_z = hover_throttle

        stable_count = 0
        start_time = time.time()
        phase = "FAST"

        while time.time() - start_time < timeout:
            error = normalize_angle(target_yaw - self.yaw)
            abs_error = abs(error)
            direction = 1 if error > 0 else -1

            # Check completion
            if abs_error < self.tolerance:
                stable_count += 1
                if stable_count >= self.stable_samples:
                    self._stop()
                    actual = math.degrees(normalize_angle(self.yaw - initial_yaw))
                    final_err = math.degrees(abs_error)
                    pos = self.position
                    self.get_logger().info(
                        f"[END] pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) angle={to_360(self.yaw):.1f}°"
                    )
                    self.get_logger().info(f"✓ Done! Rotated {actual:.1f}° (error: {final_err:.2f}°)")
                    return True
            else:
                stable_count = 0

            # Phase logic
            if abs_error >= self.fast_threshold:
                # Phase 1: Fast
                if phase != "FAST":
                    phase = "FAST"
                    self.get_logger().info(f"[FAST] {math.degrees(abs_error):.1f}° remaining")
                self.current_r = self.fast_force * direction

            elif abs_error >= self.slow_threshold:
                # Phase 2: Slow
                if phase != "SLOW":
                    phase = "SLOW"
                    pos = self.position
                    self.get_logger().info(f"[SLOW] {math.degrees(abs_error):.1f}° remaining")
                    self.get_logger().info(
                        f"[TRANSITION] pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) angle={to_360(self.yaw):.1f}°"
                    )
                self.current_r = self.slow_force * direction

            else:
                # Coast to target
                if phase != "COAST":
                    phase = "COAST"
                    self.get_logger().info(f"[COAST] {math.degrees(abs_error):.1f}° remaining")
                self.current_r = 0.0

            time.sleep(0.025)

        self._stop()
        final_err = math.degrees(abs(normalize_angle(target_yaw - self.yaw)))
        pos = self.position
        self.get_logger().info(
            f"[END] pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) angle={to_360(self.yaw):.1f}°"
        )
        self.get_logger().warn(f"Timeout! Final error: {final_err:.2f}°")
        return False

    def _stop(self):
        """Stop all movement."""
        self.current_x = self.current_y = self.current_z = self.current_r = 0.0
        time.sleep(0.05)