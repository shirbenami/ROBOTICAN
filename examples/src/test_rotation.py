#!/usr/bin/env python3
"""
Test Yaw Rotation - Matches multi_city_missions_gui.py pattern
Usage: python3 test_rotation.py --rooster R1 --degrees 90
"""

import argparse
import math
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from rooster_yaw_controller import YawController


def main():
    parser = argparse.ArgumentParser(description="Test Yaw Rotation")
    parser.add_argument("--rooster", "-r", type=str, default="R1",
                        help="Rooster ID (R1, R2, or R3)")
    parser.add_argument("--degrees", "-d", type=float, default=90.0,
                        help="Degrees to rotate (positive=CW, negative=CCW)")
    parser.add_argument("--throttle", "-t", type=float, default=200.0,
                        help="Hover throttle (z value, range -1000 to 1000, typically 200)")
    args = parser.parse_args()

    rclpy.init()

    yaw_ctrl = YawController(
        rooster_id=args.rooster,
        name=f"test_yaw_{args.rooster}"
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(yaw_ctrl)

    # Start spinning in background thread - this keeps timers running!
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"Waiting for UAVState from {args.rooster}...")

    start = time.time()
    while not yaw_ctrl._state_received and time.time() - start < 5.0:
        time.sleep(0.1)

    if not yaw_ctrl._state_received:
        print("ERROR: No UAVState received!")
        rclpy.shutdown()
        return

    print(f"Current azimuth: {yaw_ctrl.yaw:.2f} rad ({math.degrees(yaw_ctrl.yaw):.1f}°)")
    print(f"Armed: {yaw_ctrl.is_armed}, Airborne: {yaw_ctrl.is_airborne}")

    # Arm the drone
    print(f"\nArming {args.rooster}...")
    if not yaw_ctrl.force_arm_client.wait_for_service(timeout_sec=5.0):
        print(f"ERROR: force_arm service not available!")
        rclpy.shutdown()
        return

    req = SetBool.Request()
    req.data = True
    future = yaw_ctrl.force_arm_client.call_async(req)

    # Wait for arm response
    while not future.done():
        time.sleep(0.1)

    try:
        resp = future.result()
        if resp.success:
            print(f"Arm call returned: {resp.message}")
        else:
            print(f"Arm call failed: {resp.message}")
    except Exception as e:
        print(f"Arm exception: {e}")

    # Wait for arming to take effect
    print("Waiting for drone to arm...")
    time.sleep(2.0)

    print(f"Armed: {yaw_ctrl.is_armed}, Airborne: {yaw_ctrl.is_airborne}")

    if not yaw_ctrl.is_armed:
        print("⚠ Drone is NOT ARMED!")
        print("Trying to continue anyway...")

    # TAKEOFF: Apply throttle to get airborne
    print(f"\nTaking off with throttle={args.throttle}...")
    yaw_ctrl.current_z = args.throttle

    # Wait to become airborne
    takeoff_start = time.time()
    while not yaw_ctrl.is_airborne and time.time() - takeoff_start < 10.0:
        time.sleep(0.1)
        if int((time.time() - takeoff_start) * 10) % 10 == 0:  # Every second
            print(f"  Waiting... Armed: {yaw_ctrl.is_armed}, Airborne: {yaw_ctrl.is_airborne}, z={yaw_ctrl.current_z}")

    if not yaw_ctrl.is_airborne:
        print("⚠ Failed to become airborne! Trying higher throttle...")
        yaw_ctrl.current_z = 700.0  # Try higher throttle
        time.sleep(3.0)
        print(f"  Armed: {yaw_ctrl.is_armed}, Airborne: {yaw_ctrl.is_airborne}")

    if yaw_ctrl.is_airborne:
        print("✓ Airborne!")
    else:
        print("⚠ Still not airborne, continuing anyway...")

    print(f"\nRotating {args.degrees}°...")

    # Run rotation (blocking) - keep the throttle we set
    success = yaw_ctrl.rotate(args.degrees, hover_throttle=yaw_ctrl.current_z)

    if success:
        print("\n✓ Rotation successful!")
    else:
        print("\n✗ Rotation failed or timed out")

    # Land - reduce throttle
    print("\nLanding...")
    yaw_ctrl.current_z = 0.0
    time.sleep(2.0)

    # Disarm
    print(f"\nDisarming {args.rooster}...")
    req = SetBool.Request()
    req.data = False
    future = yaw_ctrl.force_arm_client.call_async(req)
    while not future.done():
        time.sleep(0.1)
    print("Disarm command sent")

    time.sleep(1.0)

    print("Shutting down...")
    rclpy.shutdown()


if __name__ == "__main__":
    main()