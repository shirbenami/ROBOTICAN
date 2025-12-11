#!/usr/bin/env python3
"""
Two-Phase Continuous Yaw Controller with Filtering
---------------------------------------------------
Phase 1: Stabilize (5s) - Z=200, R=0
Phase 2: Fast rotation (800 force) until within 15°
Phase 3: Slow rotation (100 force) until within 2°

Includes outlier filtering and moving average for azimuth.
"""

import math
import time
import threading
from collections import deque
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
    return math.degrees(rad) % 360


class YawController(Node):
    def __init__(self, rooster_id: str = "R1", name: str = "yaw_controller"):
        super().__init__(name)
        self.rooster_id = rooster_id

        # State
        self._lock = threading.Lock()
        self._azimuth_raw = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_z = 0.0
        self._state_received = False
        self._armed = False
        self._airborne = False

        # Azimuth filtering - only last K values (small K for fast rotation)
        self._azimuth_k = 5
        self._azimuth_history = deque(maxlen=self._azimuth_k)
        self._azimuth_filtered = 0.0
        self._outlier_threshold = math.radians(45)  # Max jump to consider valid

        # Control outputs
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_r = 0.0
        self.flight_mode = 1

        # Stabilization phase
        self.stabilize_time = 5.0
        self.stabilize_z = 200.0

        # Phase 2: Fast
        self.fast_force = 800.0
        self.fast_threshold = math.radians(15.0)

        # Phase 3: Slow
        self.slow_force = 100.0
        self.slow_threshold = math.radians(2.0)

        # Completion
        self.tolerance = math.radians(1.0)
        self.stable_samples = 3  # Need 3 consecutive confirmations

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

    def _is_outlier(self, new_value: float) -> bool:
        """Check if new azimuth value is an outlier."""
        if len(self._azimuth_history) < 2:
            return False
        # Compare to recent average
        avg = sum(self._azimuth_history) / len(self._azimuth_history)
        diff = abs(normalize_angle(new_value - avg))
        return diff > self._outlier_threshold

    def _update_filtered_azimuth(self, raw: float):
        """Update filtered azimuth, rejecting outliers."""
        if self._is_outlier(raw):
            self.get_logger().debug(f"Outlier rejected: {math.degrees(raw):.1f}°")
            return

        self._azimuth_history.append(raw)

        if len(self._azimuth_history) > 0:
            # Average using circular mean for angles
            sin_sum = sum(math.sin(a) for a in self._azimuth_history)
            cos_sum = sum(math.cos(a) for a in self._azimuth_history)
            self._azimuth_filtered = math.atan2(sin_sum, cos_sum)

    def _state_cb(self, msg: UAVState):
        with self._lock:
            self._azimuth_raw = msg.azimuth
            self._update_filtered_azimuth(msg.azimuth)
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
        """Return filtered azimuth."""
        with self._lock:
            return self._azimuth_filtered

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

    def rotate(self, degrees: float, timeout: float = 30.0, hover_throttle: float = 500.0) -> bool:
        """Rotate using two-phase continuous movement with stabilization."""
        self.get_logger().info(f"Rotate {degrees:+.1f}° requested")

        # Wait for state
        t0 = time.time()
        while not self._state_received and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self._state_received:
            self.get_logger().error("No UAVState received!")
            return False

        # Phase 1: Stabilize
        self.get_logger().info(f"[STABILIZE] {self.stabilize_time}s - Z={self.stabilize_z}")
        self.current_z = self.stabilize_z
        self.current_r = 0.0
        time.sleep(self.stabilize_time)

        # Calculate target from filtered yaw
        initial_yaw = self.yaw
        target_yaw = normalize_angle(initial_yaw + math.radians(degrees))

        self.get_logger().info(f"Rotating: {to_360(initial_yaw):.1f}° → {to_360(target_yaw):.1f}°")

        # Set hover
        self.current_z = hover_throttle

        stable_count = 0
        start_time = time.time()
        phase = "FAST"

        while time.time() - start_time < timeout:
            error = normalize_angle(target_yaw - self.yaw)
            abs_error = abs(error)
            direction = 1 if error > 0 else -1

            # Check completion (need 3 consecutive confirmations)
            if abs_error < self.tolerance:
                stable_count += 1
                if stable_count >= self.stable_samples:
                    self._stop()
                    actual = math.degrees(normalize_angle(self.yaw - initial_yaw))
                    self.get_logger().info(f"✓ Done! Rotated {actual:.1f}° (error: {math.degrees(abs_error):.2f}°)")
                    return True
            else:
                stable_count = 0

            # Phase logic
            if abs_error >= self.fast_threshold:
                if phase != "FAST":
                    phase = "FAST"
                    self.get_logger().info(f"[FAST] {math.degrees(abs_error):.1f}° remaining")
                self.current_r = self.fast_force * direction

            elif abs_error >= self.slow_threshold:
                if phase != "SLOW":
                    phase = "SLOW"
                    self.get_logger().info(f"[SLOW] {math.degrees(abs_error):.1f}° remaining")
                self.current_r = self.slow_force * direction

            else:
                if phase != "COAST":
                    phase = "COAST"
                    self.get_logger().info(f"[COAST] {math.degrees(abs_error):.1f}° remaining")
                self.current_r = 0.0

            time.sleep(0.025)

        self._stop()
        final_err = math.degrees(abs(normalize_angle(target_yaw - self.yaw)))
        self.get_logger().warn(f"Timeout! Final error: {final_err:.2f}°")
        return False

    def _stop(self):
        """Stop all movement."""
        self.current_x = self.current_y = self.current_z = self.current_r = 0.0
        time.sleep(0.05)