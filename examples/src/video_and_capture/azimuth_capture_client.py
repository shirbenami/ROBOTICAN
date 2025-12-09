#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from fcu_driver_interfaces.msg import UAVState  # adjust if your state msg is different


class AzimuthCaptureClient(Node):
    """
    Listens to /R2/fcu/state, converts yaw (radians) to degrees,
    and when the azimuth enters a configured angle window, it performs
    a capture sequence exactly once per window:

        /R2/start_capture  -> (delay) ->  /R2/frame_capture  ->  /R2/stop_capture
    """

    def __init__(self):
        super().__init__("azimuth_capture_client")

        self.drone_id = "R2"

        # angle windows in degrees: [start, end)
        self.angle_windows: List[Tuple[float, float, str]] = [
            (30.0, 35.0, "30-35"),
            (70.0, 75.0, "70-75"),
            (105.0, 110.0, "105-110"),
            (175.0, 180.0, "175-180"),
            (220.0, 225.0, "220-225"),
            (260.0, 265.0, "260-265"),
            (300.0, 305.0, "300-305"),
            (355.0, 360.0, "355-360"),
            (40.0, 45.0, "40-45"),
            (15.0, 20.0, "15-20"),
            (90.0, 95.0, "90-95"),
            (130.0, 135.0, "130-135"),
            (150.0, 155.0, "150-155"),
            (280.0, 285.0, "280-285"),
            (330.0, 335.0, "330-335"),
        ]

        # trigger once per window per run
        self.already_triggered = {label: False for _, _, label in self.angle_windows}

        # flag to avoid overlapping one-shot calls
        self.capture_busy: bool = False

        # one-shot capture service client
        self.capture_one_shot_client = self.create_client(
            Trigger, f"/{self.drone_id}/capture_frame_one_shot"
        )

        # subscriber to FCU state
        self.state_sub = self.create_subscription(
            UAVState,
            f"/{self.drone_id}/fcu/state",
            self.state_callback,
            10,
        )

        self.get_logger().info(
            f"{self.drone_id}: AzimuthCaptureClient started, watching windows: "
            f"{[lbl for _, _, lbl in self.angle_windows]}"
        )

    @staticmethod
    def rad_to_deg(angle_rad: float) -> float:
        deg = math.degrees(angle_rad)
        return (deg + 360.0) % 360.0

    def services_ready(self) -> bool:
        return self.capture_one_shot_client.service_is_ready()

    def state_callback(self, msg: UAVState):
        try:
            yaw_rad = msg.azimuth
        except AttributeError:
            self.get_logger().error(
                "UAVState has no field 'azimuth'. Update state_callback() to use the correct field."
            )
            return

        az_deg = self.rad_to_deg(yaw_rad)

        if self.capture_busy:
            return

        if not self.services_ready():
            return

        for start, end, label in self.angle_windows:
            if self.already_triggered[label]:
                continue
            if start <= az_deg < end:
                self.get_logger().info(
                    f"{self.drone_id}: azimuth {az_deg:.2f} deg in window {label}, "
                    f"starting capture sequence."
                )
                self.start_capture_sequence(label, az_deg)
                break

    def start_capture_sequence(self, label: str, az_deg: float):
        if self.capture_busy:
            return

        self.capture_busy = True

        req = Trigger.Request()
        future = self.capture_one_shot_client.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                self.get_logger().info(
                    f"{self.drone_id}: capture_frame_one_shot reply for window {label} "
                    f"(az={az_deg:.5f} deg): success={resp.success}, msg='{resp.message}'"
                )
            except Exception as e:
                self.get_logger().error(
                    f"{self.drone_id}: capture_frame_one_shot call failed for window {label}: {e}"
                )

            self.already_triggered[label] = True
            self.capture_busy = False

        future.add_done_callback(_done_cb)


def main(args=None):
    rclpy.init(args=args)
    node = AzimuthCaptureClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
