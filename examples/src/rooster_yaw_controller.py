#!/usr/bin/env python3
"""
Precision Yaw Controller
------------------------------------------
Rotates drone by exact degrees using PID with velocity profiling.
Positive = clockwise, Negative = counterclockwise.

Based on working multi_city_missions_gui.py pattern:
- Continuous 40 Hz ManualControl publishing via timer
- Continuous 1 Hz KeepAlive publishing via timer

FINE MODE: Uses impulse-based control (medium force, short duration)
to overcome simulator physics resistance.
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
    Precision yaw controller matching multi_city_missions_gui.py pattern.
    Uses timers for continuous publishing.
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

        # Control state (like DroneController in the GUI)
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_z: float = 0.0
        self.current_r: float = 0.0
        self.flight_mode: int = 1  # roll

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

        # Timers - matching multi_city_missions_gui.py
        self.manual_timer = self.create_timer(1.0 / 40.0, self._manual_timer_cb)  # 40 Hz
        self.keep_alive_timer = self.create_timer(1.0, self._keep_alive_timer_cb)  # 1 Hz

        # PID gains (tuned for -1000 to 1000 output range)
        self.kp = 400.0
        self.ki = 50.0
        self.kd = 80.0

        # Limits (ManualControl uses -1000 to 1000 scale)
        self.max_rate = 600.0
        self.min_rate = 200.0
        self.integral_limit = 300.0
        self.decel_angle = 0.3  # Start slowing at ~17°

        # Fine phase parameters
        self.fine_angle = math.radians(8.0)
        self.tolerance = math.radians(0.5)
        self.stable_required = 35

        # IMPULSE CONTROL parameters for fine mode
        self.impulse_force = 350.0  # Medium force to apply (not too weak, not too strong)
        self.impulse_duration = 0.08  # How long to apply force (seconds)
        self.settle_duration = 0.15  # How long to wait after impulse (seconds)
        self.fine_deadband = math.radians(1.0)  # Don't impulse if error is below this (but above tolerance)

        # PID state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._filtered_deriv = 0.0

        self.get_logger().info(
            f"YawController ready for {rooster_id} (tolerance=0.5°, impulse mode enabled)"
        )

    def _state_cb(self, msg: UAVState):
        """Callback for UAVState."""
        with self._lock:
            self._azimuth = msg.azimuth
            self._armed = msg.armed
            self._airborne = msg.airborne
            self._state_received = True

    def _manual_timer_cb(self):
        """Publish ManualControl at 40 Hz - always running."""
        msg = ManualControl()
        msg.x = float(self.current_x)
        msg.y = float(self.current_y)
        msg.z = float(self.current_z)
        msg.r = float(self.current_r)
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def _keep_alive_timer_cb(self):
        """Publish KeepAlive at 1 Hz - always running."""
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
        """Arm or disarm the drone."""
        if not self.force_arm_client.service_is_ready():
            self.get_logger().warn("force_arm service not ready")
            return False

        req = SetBool.Request()
        req.data = arm

        future = self.force_arm_client.call_async(req)
        return True

    def _apply_impulse(self, direction: int, hover_throttle: float) -> None:
        """
        Apply a short impulse in the given direction.
        direction: +1 for clockwise, -1 for counterclockwise
        """
        force = self.impulse_force * direction

        # Apply force
        self.current_r = force
        time.sleep(self.impulse_duration)

        # Stop rotation and let it settle
        self.current_r = 0.0
        time.sleep(self.settle_duration)

    def rotate(self, degrees: float, timeout: float = 45.0, hover_throttle: float = 500.0) -> bool:
        """
        Rotate by specified degrees.
        Positive = clockwise, Negative = counterclockwise.
        hover_throttle: Z value in -1000 to 1000 range (500 typical for hover)
        Returns True if target reached.
        """
        self.get_logger().info(f"[DEBUG] rotate() called with degrees={degrees}")

        # Wait for state
        t0 = time.time()
        while not self._state_received and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self._state_received:
            self.get_logger().error("[DEBUG] No UAVState received!")
            return False

        # Get initial yaw
        initial_yaw = self.yaw
        self.get_logger().info(f"[DEBUG] Initial yaw: {math.degrees(initial_yaw):.2f}°")

        # Compute target yaw
        delta_rad = math.radians(degrees)
        target_yaw = normalize_angle(initial_yaw + delta_rad)

        self.get_logger().info(
            f"Rotating {degrees:+.1f}° | {math.degrees(initial_yaw):.1f}° → {math.degrees(target_yaw):.1f}°"
        )

        # Reset PID
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._filtered_deriv = 0.0

        # Set hover throttle
        self.current_z = hover_throttle

        stable_count = 0
        start_time = time.time()
        loop_count = 0
        in_fine_mode = False

        while time.time() - start_time < timeout:
            loop_start = time.time()
            loop_count += 1

            current_yaw = self.yaw
            error = normalize_angle(target_yaw - current_yaw)
            abs_error = abs(error)

            # Check if we should enter fine mode
            if not in_fine_mode and abs_error < self.fine_angle:
                in_fine_mode = True
                self._integral = 0.0  # Reset integral
                self._filtered_deriv = 0.0
                self.current_r = 0.0  # Stop any ongoing rotation
                self.get_logger().info(f"[DEBUG] Entering FINE/IMPULSE mode at {math.degrees(abs_error):.2f}°")
                time.sleep(0.1)  # Brief pause before starting impulse control
                continue

            # Debug logging
            if loop_count % 25 == 0:
                mode = "FINE" if in_fine_mode else "COARSE"
                self.get_logger().info(
                    f"[DEBUG] [{mode}] yaw: {math.degrees(current_yaw):.2f}° | "
                    f"err: {math.degrees(error):.2f}° | r: {self.current_r:.1f} | "
                    f"armed: {self.is_armed} | airborne: {self.is_airborne}"
                )

            # === FINE MODE: Impulse-based control ===
            if in_fine_mode:
                # Check if within tolerance
                if abs_error < self.tolerance:
                    stable_count += 1
                    if stable_count >= self.stable_required:
                        self._stop()
                        final_yaw = self.yaw
                        final_err = math.degrees(abs(normalize_angle(target_yaw - final_yaw)))
                        actual_rotation = math.degrees(normalize_angle(final_yaw - initial_yaw))
                        self.get_logger().info(
                            f"✓ Complete | rotated: {actual_rotation:.2f}° | error: {final_err:.2f}°"
                        )
                        return True
                    time.sleep(0.02)
                    continue
                else:
                    stable_count = 0

                # If error is significant enough, apply an impulse
                if abs_error > self.fine_deadband:
                    direction = 1 if error > 0 else -1

                    # Scale impulse force based on error magnitude
                    # Larger error = stronger impulse, but capped
                    error_factor = min(abs_error / self.fine_angle, 1.0)
                    scaled_force = self.impulse_force * (0.5 + 0.5 * error_factor)

                    self.get_logger().info(
                        f"[DEBUG] IMPULSE: dir={direction}, force={scaled_force:.0f}, "
                        f"err={math.degrees(error):.2f}°"
                    )

                    # Apply impulse
                    self.current_r = scaled_force * direction
                    time.sleep(self.impulse_duration)
                    self.current_r = 0.0
                    time.sleep(self.settle_duration)

                elif abs_error > self.tolerance:
                    # Small error but above tolerance - use very gentle impulse
                    direction = 1 if error > 0 else -1
                    gentle_force = self.impulse_force * 0.4

                    self.get_logger().info(
                        f"[DEBUG] GENTLE IMPULSE: dir={direction}, force={gentle_force:.0f}"
                    )

                    self.current_r = gentle_force * direction
                    time.sleep(self.impulse_duration * 0.5)
                    self.current_r = 0.0
                    time.sleep(self.settle_duration)

                continue

            # === COARSE MODE: PID control ===
            # Velocity profiling
            if abs_error < self.decel_angle:
                rate_limit = 200.0 + (self.max_rate - 200.0) * (abs_error / self.decel_angle)
            else:
                rate_limit = self.max_rate

            # PID computation
            now = time.time()
            dt = 0.02 if self._prev_time is None else max(now - self._prev_time, 0.001)
            self._prev_time = now

            p_term = self.kp * error
            self._integral += error * dt
            self._integral = max(min(self._integral, self.integral_limit), -self.integral_limit)
            i_term = self.ki * self._integral

            raw_deriv = (error - self._prev_error) / dt
            self._filtered_deriv = 0.1 * raw_deriv + 0.9 * self._filtered_deriv
            d_term = self.kd * self._filtered_deriv
            self._prev_error = error

            output = p_term + i_term + d_term
            output = max(min(output, rate_limit), -rate_limit)

            self.current_r = output

            # Control rate ~50Hz
            sleep_time = 0.02 - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._stop()
        final_err = math.degrees(abs(normalize_angle(target_yaw - self.yaw)))
        self.get_logger().warn(f"Timeout! error: {final_err:.2f}°")
        return False

    def _stop(self):
        """Stop all movement."""
        self.get_logger().info("[DEBUG] Stopping...")
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_r = 0.0
        time.sleep(0.1)