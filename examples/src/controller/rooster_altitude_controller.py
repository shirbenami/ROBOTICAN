#!/usr/bin/env python3
"""
Altitude Controller
-------------------
Raises the drone by a specified height and maintains that altitude.
Uses PID control for smooth altitude management.

Usage:
    python3 rooster_altitude_controller.py [OPTIONS]

Examples:
    python3 rooster_altitude_controller.py --height 2.0
    python3 rooster_altitude_controller.py -r R1 -H 1.5 --descent-rate 0.3
    python3 rooster_altitude_controller.py --kp 200 --ki 10 --kd 60
"""

import argparse
import math
import time
import threading
from dataclasses import dataclass
from rclpy.node import Node
import rclpy

from fcu_driver_interfaces.msg import ManualControl, UAVState
from rooster_handler_interfaces.msg import KeepAlive
from std_srvs.srv import SetBool


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class AltitudeConfig:
    """Configuration parameters for the altitude controller."""

    # Drone identification
    rooster_id: str = "R1"

    # Target altitude
    target_height: float = 1.0  # meters

    # Throttle settings
    hover_throttle: float = 500.0  # Base throttle to hover
    takeoff_throttle: float = 600.0  # Initial takeoff throttle
    max_throttle: float = 700.0  # Maximum throttle limit
    min_throttle: float = 300.0  # Minimum throttle limit

    # PID gains
    kp: float = 150.0  # Proportional gain
    ki: float = 5.0  # Integral gain
    kd: float = 50.0  # Derivative gain

    # PID limits
    integral_limit: float = 50.0  # Anti-windup limit

    # Altitude tolerance
    tolerance: float = 0.05  # meters (5cm)
    stable_samples: int = 20  # Samples needed to confirm stable

    # Landing parameters
    descent_rate: float = 0.2  # meters per second
    landing_throttle: float = 400.0
    ground_threshold: float = 0.1  # meters (10cm)

    # Timing
    control_rate: float = 40.0  # Hz
    keepalive_rate: float = 1.0  # Hz
    log_interval: float = 2.0  # seconds

    def __str__(self) -> str:
        return f"""
╔══════════════════════════════════════════════════════════╗
║              ALTITUDE CONTROLLER CONFIG                  ║
╠══════════════════════════════════════════════════════════╣
║  Drone ID:        {self.rooster_id:<39} ║
║  Target Height:   {self.target_height:<6.2f} m                              ║
╠══════════════════════════════════════════════════════════╣
║  THROTTLE SETTINGS                                       ║
║    Hover:         {self.hover_throttle:<6.1f}                              ║
║    Takeoff:       {self.takeoff_throttle:<6.1f}                              ║
║    Max:           {self.max_throttle:<6.1f}                              ║
║    Min:           {self.min_throttle:<6.1f}                              ║
╠══════════════════════════════════════════════════════════╣
║  PID GAINS                                               ║
║    Kp:            {self.kp:<6.1f}                              ║
║    Ki:            {self.ki:<6.1f}                              ║
║    Kd:            {self.kd:<6.1f}                              ║
╠══════════════════════════════════════════════════════════╣
║  LANDING                                                 ║
║    Descent Rate:  {self.descent_rate:<6.2f} m/s                          ║
║    Land Throttle: {self.landing_throttle:<6.1f}                              ║
╚══════════════════════════════════════════════════════════╝
"""


# =============================================================================
# ALTITUDE CONTROLLER
# =============================================================================

class AltitudeController(Node):
    """ROS2 node for altitude control of a drone."""

    def __init__(self, config: AltitudeConfig):
        super().__init__("altitude_controller")
        self.config = config

        # State variables
        self._lock = threading.Lock()
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_z = 0.0
        self._altitude_relative = 0.0
        self._azimuth = 0.0
        self._state_received = False
        self._armed = False
        self._airborne = False

        # Control outputs
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0  # Throttle
        self.current_r = 0.0
        self.flight_mode = KeepAlive.FLIGHT_MODE_MANUAL

        # PID state
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

        # ROS2 setup
        self._setup_ros()

        self.get_logger().info(f"AltitudeController initialized for {config.rooster_id}")

    def _setup_ros(self):
        """Setup ROS2 publishers, subscribers, and services."""
        rid = self.config.rooster_id

        # Subscriber
        self.state_sub = self.create_subscription(
            UAVState, f"/{rid}/fcu/state", self._state_cb, 10
        )

        # Publishers
        self.manual_pub = self.create_publisher(
            ManualControl, f"/{rid}/manual_control", 10
        )
        self.keep_alive_pub = self.create_publisher(
            KeepAlive, f"/{rid}/keep_alive", 10
        )

        # Service client
        self.force_arm_client = self.create_client(
            SetBool, f"/{rid}/fcu/command/force_arm"
        )

        # Timers
        self.create_timer(1.0 / self.config.control_rate, self._publish_control)
        self.create_timer(1.0 / self.config.keepalive_rate, self._publish_keepalive)

    # -------------------------------------------------------------------------
    # ROS2 Callbacks
    # -------------------------------------------------------------------------

    def _state_cb(self, msg: UAVState):
        """Handle incoming UAV state messages."""
        with self._lock:
            self._pos_x = msg.position.x
            self._pos_y = msg.position.y
            self._pos_z = msg.position.z
            self._altitude_relative = msg.altitude_relative
            self._azimuth = msg.azimuth
            self._armed = msg.armed
            self._airborne = msg.airborne
            self._state_received = True

    def _publish_control(self):
        """Publish manual control message."""
        msg = ManualControl()
        msg.x = self.current_x
        msg.y = self.current_y
        msg.z = self.current_z
        msg.r = self.current_r
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def _publish_keepalive(self):
        """Publish keepalive message."""
        msg = KeepAlive()
        msg.is_active = True
        msg.requested_flight_mode = self.flight_mode
        msg.command_reboot = False
        self.keep_alive_pub.publish(msg)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def altitude(self) -> float:
        """Get current relative altitude."""
        with self._lock:
            return self._altitude_relative

    @property
    def position(self) -> tuple:
        """Get current position (x, y, z)."""
        with self._lock:
            return (self._pos_x, self._pos_y, self._pos_z)

    @property
    def is_armed(self) -> bool:
        """Check if drone is armed."""
        with self._lock:
            return self._armed

    @property
    def is_airborne(self) -> bool:
        """Check if drone is airborne."""
        with self._lock:
            return self._airborne

    # -------------------------------------------------------------------------
    # Arm / Disarm
    # -------------------------------------------------------------------------

    def arm(self, arm: bool = True) -> bool:
        """Arm or disarm the drone."""
        if not self.force_arm_client.service_is_ready():
            self.get_logger().warn("force_arm service not ready, waiting...")
            self.force_arm_client.wait_for_service(timeout_sec=5.0)
            if not self.force_arm_client.service_is_ready():
                self.get_logger().error("force_arm service not available!")
                return False

        req = SetBool.Request()
        req.data = arm
        future = self.force_arm_client.call_async(req)

        # Wait for result
        timeout = 5.0
        start = time.time()
        while not future.done() and time.time() - start < timeout:
            time.sleep(0.1)

        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info(f"{'Armed' if arm else 'Disarmed'} successfully")
                return True
            else:
                self.get_logger().error(f"{'Arm' if arm else 'Disarm'} failed: {result.message}")
                return False
        else:
            self.get_logger().error("Arm request timed out")
            return False

    # -------------------------------------------------------------------------
    # PID Controller
    # -------------------------------------------------------------------------

    def _compute_throttle_pid(self, target_alt: float, current_alt: float) -> float:
        """Compute throttle using PID control."""
        cfg = self.config
        current_time = time.time()
        error = target_alt - current_alt

        if self._last_time is None:
            self._last_time = current_time
            self._last_error = error
            return cfg.hover_throttle

        dt = current_time - self._last_time
        if dt <= 0:
            dt = 1.0 / cfg.control_rate

        # Proportional
        p_term = cfg.kp * error

        # Integral (with anti-windup)
        self._integral += error * dt
        self._integral = max(-cfg.integral_limit, min(cfg.integral_limit, self._integral))
        i_term = cfg.ki * self._integral

        # Derivative
        derivative = (error - self._last_error) / dt
        d_term = cfg.kd * derivative

        # Compute throttle
        throttle = cfg.hover_throttle + p_term + i_term + d_term

        # Clamp to limits
        throttle = max(cfg.min_throttle, min(cfg.max_throttle, throttle))

        # Save state
        self._last_error = error
        self._last_time = current_time

        return throttle

    def _reset_pid(self):
        """Reset PID controller state."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    # -------------------------------------------------------------------------
    # Flight Operations
    # -------------------------------------------------------------------------

    def raise_and_hold(self) -> bool:
        """
        Raise the drone to target height and hold altitude indefinitely.
        Press CTRL+C to land and disarm.

        Returns:
            True if completed successfully, False otherwise
        """
        cfg = self.config

        self.get_logger().info(f"Raise and hold at {cfg.target_height:.2f}m requested")
        self.get_logger().info("Press CTRL+C to land and disarm")

        # Wait for state
        if not self._wait_for_state():
            return False

        # Record initial altitude
        initial_alt = self.altitude
        target_alt = initial_alt + cfg.target_height

        self.get_logger().info(f"Initial altitude: {initial_alt:.3f}m")
        self.get_logger().info(f"Target altitude:  {target_alt:.3f}m")

        # Reset PID state
        self._reset_pid()

        # Arm if not armed
        if not self._ensure_armed():
            return False

        pos = self.position
        self.get_logger().info(
            f"[START] pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) alt={self.altitude:.3f}m"
        )

        stable_count = 0
        phase = "TAKEOFF"
        last_log_time = 0
        target_reached = False

        # Main control loop - runs until CTRL+C
        while True:
            current_alt = self.altitude
            error = target_alt - current_alt
            abs_error = abs(error)

            # Phase transitions
            if phase == "TAKEOFF" and current_alt > initial_alt + 0.1:
                phase = "CLIMBING"
                self.get_logger().info(f"[CLIMBING] Altitude: {current_alt:.3f}m")

            if phase == "CLIMBING" and abs_error < 0.2:
                phase = "HOLDING"
                self.get_logger().info(f"[HOLDING] Altitude: {current_alt:.3f}m")

            # Check if at target
            if abs_error < cfg.tolerance:
                stable_count += 1
                if stable_count >= cfg.stable_samples and not target_reached:
                    target_reached = True
                    pos = self.position
                    self.get_logger().info(
                        f"[STABLE] pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) alt={current_alt:.3f}m"
                    )
                    self.get_logger().info(
                        f"✓ Target altitude reached! Holding at {current_alt:.3f}m"
                    )
                    self.get_logger().info("Press CTRL+C to land and disarm")
            else:
                stable_count = 0

            # Compute throttle
            if phase == "TAKEOFF":
                self.current_z = cfg.takeoff_throttle
            else:
                self.current_z = self._compute_throttle_pid(target_alt, current_alt)

            # Log periodically
            current_time = time.time()
            if current_time - last_log_time > cfg.log_interval:
                self.get_logger().info(
                    f"[{phase}] alt={current_alt:.3f}m target={target_alt:.3f}m "
                    f"error={error:.3f}m throttle={self.current_z:.1f}"
                )
                last_log_time = current_time

            time.sleep(1.0 / cfg.control_rate)

        return True

    def land(self):
        """
        Gentle landing by slowly descending to ground.
        Uses configured descent_rate and landing_throttle.
        """
        cfg = self.config

        self.get_logger().info("=" * 50)
        self.get_logger().info("LANDING SEQUENCE INITIATED")
        self.get_logger().info("=" * 50)

        current_alt = self.altitude
        self.get_logger().info(f"Current altitude: {current_alt:.3f}m")
        self.get_logger().info(f"Descent rate: {cfg.descent_rate:.2f} m/s")

        target_alt = current_alt
        last_log_time = time.time()
        loop_period = 1.0 / cfg.control_rate

        # Phase 1: Controlled descent
        while self.altitude > cfg.ground_threshold:
            # Gradually lower target altitude
            target_alt -= cfg.descent_rate * loop_period
            target_alt = max(0.0, target_alt)

            current_alt = self.altitude
            error = target_alt - current_alt

            # Throttle control during landing
            if current_alt > 0.3:
                # Above 30cm: controlled descent
                self.current_z = cfg.landing_throttle + (error * 50.0)
                self.current_z = max(cfg.min_throttle, min(cfg.max_throttle, self.current_z))
            else:
                # Below 30cm: gradually reduce throttle
                self.current_z = max(0.0, self.current_z - 2.0)

            # Log every second
            current_time = time.time()
            if current_time - last_log_time > 1.0:
                self.get_logger().info(
                    f"[DESCENDING] alt={current_alt:.3f}m throttle={self.current_z:.1f}"
                )
                last_log_time = current_time

            time.sleep(loop_period)

        self.get_logger().info("[TOUCHDOWN] Ground detected!")

        # Phase 2: Cut throttle smoothly
        self.get_logger().info("[SETTLING] Reducing throttle...")
        for throttle in range(int(self.current_z), 0, -5):
            self.current_z = float(throttle)
            time.sleep(0.05)

        self.current_z = 0.0
        time.sleep(0.5)

        # Phase 3: Disarm
        self.get_logger().info("[DISARMING] Sending disarm command...")
        self.arm(False)

        self.get_logger().info("=" * 50)
        self.get_logger().info("LANDING COMPLETE - DISARMED")
        self.get_logger().info("=" * 50)

    # -------------------------------------------------------------------------
    # Helper Methods
    # -------------------------------------------------------------------------

    def _wait_for_state(self, timeout: float = 5.0) -> bool:
        """Wait for UAV state to be received."""
        t0 = time.time()
        while not self._state_received and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not self._state_received:
            self.get_logger().error("No UAVState received!")
            return False
        return True

    def _ensure_armed(self) -> bool:
        """Ensure drone is armed, arm if not."""
        if not self.is_armed:
            self.get_logger().info("Arming drone...")
            self.current_z = 0.0
            time.sleep(0.5)

            if not self.arm(True):
                self.get_logger().error("Failed to arm!")
                return False
            time.sleep(1.0)
        return True

    def _stop(self):
        """Stop all movement except throttle."""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_r = 0.0
        time.sleep(0.05)


# =============================================================================
# COMMAND LINE INTERFACE
# =============================================================================

def parse_args() -> AltitudeConfig:
    """Parse command line arguments and return configuration."""
    parser = argparse.ArgumentParser(
        description="Altitude Controller - Raise drone and hold altitude",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -r R1 --height 1.0
  %(prog)s --hover-throttle 520 --takeoff-throttle 620
  %(prog)s --kp 200 --ki 10 --kd 60
  %(prog)s --descent-rate 0.15 --landing-throttle 380
        """
    )

    # Drone ID
    parser.add_argument(
        "-r", "--rooster-id",
        type=str,
        default="R1",
        help="Drone ID (default: R1)"
    )

    # Target altitude
    parser.add_argument(
        "-H", "--height",
        type=float,
        default=1.0,
        help="Target height in meters (default: 1.0)"
    )

    # Throttle settings
    throttle_group = parser.add_argument_group("Throttle Settings")
    throttle_group.add_argument(
        "--hover-throttle",
        type=float,
        default=500.0,
        help="Base throttle to hover (default: 500)"
    )
    throttle_group.add_argument(
        "--takeoff-throttle",
        type=float,
        default=600.0,
        help="Throttle for takeoff (default: 600)"
    )
    throttle_group.add_argument(
        "--max-throttle",
        type=float,
        default=700.0,
        help="Maximum throttle limit (default: 700)"
    )
    throttle_group.add_argument(
        "--min-throttle",
        type=float,
        default=300.0,
        help="Minimum throttle limit (default: 300)"
    )

    # PID gains
    pid_group = parser.add_argument_group("PID Gains")
    pid_group.add_argument(
        "--kp",
        type=float,
        default=150.0,
        help="Proportional gain (default: 150)"
    )
    pid_group.add_argument(
        "--ki",
        type=float,
        default=5.0,
        help="Integral gain (default: 5)"
    )
    pid_group.add_argument(
        "--kd",
        type=float,
        default=50.0,
        help="Derivative gain (default: 50)"
    )

    # Landing parameters
    landing_group = parser.add_argument_group("Landing Settings")
    landing_group.add_argument(
        "--descent-rate",
        type=float,
        default=0.2,
        help="Descent rate in m/s (default: 0.2)"
    )
    landing_group.add_argument(
        "--landing-throttle",
        type=float,
        default=400.0,
        help="Throttle during landing descent (default: 400)"
    )
    landing_group.add_argument(
        "--ground-threshold",
        type=float,
        default=0.1,
        help="Altitude to consider landed in meters (default: 0.1)"
    )

    # Tolerance
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.05,
        help="Altitude tolerance in meters (default: 0.05)"
    )

    args = parser.parse_args()

    # Create config from args
    config = AltitudeConfig(
        rooster_id=args.rooster_id,
        target_height=args.height,
        hover_throttle=args.hover_throttle,
        takeoff_throttle=args.takeoff_throttle,
        max_throttle=args.max_throttle,
        min_throttle=args.min_throttle,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        descent_rate=args.descent_rate,
        landing_throttle=args.landing_throttle,
        ground_threshold=args.ground_threshold,
        tolerance=args.tolerance,
    )

    return config


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main entry point."""
    # Parse arguments
    config = parse_args()

    # Print configuration
    print(config)

    # Initialize ROS2
    rclpy.init()

    # Create controller
    controller = AltitudeController(config)

    # Start spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        # Wait for state data
        time.sleep(2.0)

        # Raise and hold indefinitely until CTRL+C
        controller.raise_and_hold()

    except KeyboardInterrupt:
        controller.get_logger().info("")
        controller.get_logger().info("CTRL+C detected!")
        controller.land()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()