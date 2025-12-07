#!/usr/bin/env python3
"""
Precision Yaw Controller (Simplified 2-Step)
---------------------------------------------
Rotates drone by exact degrees using impulse-based control.
Positive = clockwise, Negative = counterclockwise.

2 STEPS:
  1. FAST: Strong short bursts for main rotation
  2. FINE: Subtle adjustments to finish

Uses short powerful bursts to minimize drift.
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


class YawController(Node):
    """
    Simplified 2-step yaw controller using impulse-based control.
    """

    def __init__(self, rooster_id: str = "R1", name: str = "yaw_controller"):
        super().__init__(name)

        self.rooster_id = rooster_id

        # State from UAVState
        self._lock = threading.Lock()
        self._azimuth = 0.0
        self._state_received = False
        self._armed = False
        self._airborne = False

        # Control state
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_z: float = 0.0
        self.current_r: float = 0.0
        self.flight_mode: int = 1

        # ROS interfaces
        state_topic = f"/{rooster_id}/fcu/state"
        manual_topic = f"/{rooster_id}/manual_control"
        keep_alive_topic = f"/{rooster_id}/keep_alive"
        force_arm_service = f"/{rooster_id}/fcu/command/force_arm"

        self.state_sub = self.create_subscription(
            UAVState, state_topic, self._state_cb, 10
        )
        self.manual_pub = self.create_publisher(ManualControl, manual_topic, 10)
        self.keep_alive_pub = self.create_publisher(KeepAlive, keep_alive_topic, 10)
        self.force_arm_client = self.create_client(SetBool, force_arm_service)

        # Timers
        self.manual_timer = self.create_timer(1.0 / 40.0, self._manual_timer_cb)  # 40 Hz
        self.keep_alive_timer = self.create_timer(1.0, self._keep_alive_timer_cb)  # 1 Hz

        # === STEP 1: FAST - Strong rotation ===
        self.fast_threshold = math.radians(10.0)  # Switch to FINE when error < 10°
        self.fast_force = 1000.0  # Strong force
        self.fast_duration = 0.1  # Short burst (100ms)
        self.fast_settle = 0.05  # Brief settle (50ms)

        # === STEP 2: FINE - Subtle adjustments ===
        self.fine_force = 400.0  # Gentler force
        self.fine_duration = 0.05  # Shorter burst (40ms)
        self.fine_settle = 0.04  # Settle time (60ms)

        # Completion parameters
        self.tolerance = math.radians(0.5)  # 0.5° tolerance
        self.stable_required = 6  # Samples within tolerance

        self.get_logger().info(
            f"YawController ready for {rooster_id} (2-STEP: fast={self.fast_force}, fine={self.fine_force})"
        )

    def _state_cb(self, msg: UAVState):
        with self._lock:
            self._azimuth = msg.azimuth
            self._armed = msg.armed
            self._airborne = msg.airborne
            self._state_received = True

    def _manual_timer_cb(self):
        msg = ManualControl()
        msg.x = float(self.current_x)
        msg.y = float(self.current_y)
        msg.z = float(self.current_z)
        msg.r = float(self.current_r)
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def _keep_alive_timer_cb(self):
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

    def _impulse(self, force: float, duration: float, settle: float):
        """Apply a single impulse: force for duration, then stop and settle."""
        self.current_r = force
        time.sleep(duration)
        self.current_r = 0.0
        time.sleep(settle)

    def rotate(self, degrees: float, timeout: float = 30.0, hover_throttle: float = 500.0) -> bool:
        """
        Rotate by specified degrees using 2-step impulse control.
        """
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
        delta_rad = math.radians(degrees)
        target_yaw = normalize_angle(initial_yaw + delta_rad)

        self.get_logger().info(
            f"Rotating: {math.degrees(initial_yaw):.1f}° → {math.degrees(target_yaw):.1f}° (delta: {degrees:+.1f}°)"
        )

        # Set hover throttle
        self.current_z = hover_throttle

        stable_count = 0
        start_time = time.time()
        step = "FAST"

        while time.time() - start_time < timeout:
            # Get current error
            current_yaw = self.yaw
            error = normalize_angle(target_yaw - current_yaw)
            abs_error = abs(error)
            direction = 1 if error > 0 else -1

            # Check completion
            if abs_error < self.tolerance:
                stable_count += 1
                if stable_count >= self.stable_required:
                    self._stop()
                    final_yaw = self.yaw
                    final_err = math.degrees(abs(normalize_angle(target_yaw - final_yaw)))
                    actual = math.degrees(normalize_angle(final_yaw - initial_yaw))
                    final_angle = math.degrees(final_yaw)
                    self.get_logger().info(
                        f"✓ Done! Rotated {actual:.1f}° (error: {final_err:.2f}°) | Final angle: {final_angle:.1f}°"
                    )
                    return True
                time.sleep(0.01)
                continue
            else:
                stable_count = 0

            # Determine step
            if abs_error >= self.fast_threshold:
                # STEP 1: FAST
                if step != "FAST":
                    step = "FAST"
                    self.get_logger().info(f"[FAST] err={math.degrees(error):.1f}°")

                self._impulse(self.fast_force * direction, self.fast_duration, self.fast_settle)

            else:
                # STEP 2: FINE
                if step != "FINE":
                    step = "FINE"
                    self.get_logger().info(f"[FINE] err={math.degrees(error):.1f}°")

                # Scale force slightly based on error
                scale = 0.7 + 0.3 * (abs_error / self.fast_threshold)
                force = self.fine_force * scale

                self._impulse(force * direction, self.fine_duration, self.fine_settle)

        self._stop()
        final_yaw = self.yaw
        final_err = math.degrees(abs(normalize_angle(target_yaw - final_yaw)))
        final_angle = math.degrees(final_yaw)
        self.get_logger().warn(f"Timeout! error: {final_err:.2f}° | Final angle: {final_angle:.1f}°")
        return False

    def _stop(self):
        """Stop all movement."""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_r = 0.0
        time.sleep(0.05)