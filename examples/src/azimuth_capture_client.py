#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from fcu_driver_interfaces.msg import UAVState  # adjust if your state msg is different


CAPTURE_DELAY_SEC = 1.5 # wait this long after start_capture before frame_capture


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

        # flag + timer to avoid overlapping sequences
        self.capture_busy: bool = False
        self._frame_timer: Optional[rclpy.timer.Timer] = None

        # service clients
        self.start_capture_client = self.create_client(
            Trigger, f"/{self.drone_id}/start_capture"
        )
        self.frame_capture_client = self.create_client(
            Trigger, f"/{self.drone_id}/capture_frame"
        )
        self.stop_capture_client = self.create_client(
            Trigger, f"/{self.drone_id}/stop_capture"
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
        return (
            self.start_capture_client.service_is_ready()
            and self.frame_capture_client.service_is_ready()
            and self.stop_capture_client.service_is_ready()
        )

    def state_callback(self, msg: UAVState):
        # adjust field name if needed
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

        start_req = Trigger.Request()
        future_start = self.start_capture_client.call_async(start_req)

        def _start_done_cb(fut):
            try:
                resp = fut.result()
                self.get_logger().info(
                    f"{self.drone_id}: start_capture reply for window {label} "
                    f"(az={az_deg:.5f} deg): success={resp.success}, msg='{resp.message}'"
                )
                if not resp.success:
                    # abort sequence but still mark to avoid spamming this window
                    self.capture_busy = False
                    self.already_triggered[label] = True
                    return
            except Exception as e:
                self.get_logger().error(
                    f"{self.drone_id}: start_capture call failed for window {label}: {e}"
                )
                self.capture_busy = False
                self.already_triggered[label] = True
                return

            # schedule frame_capture after a short delay
            if self._frame_timer is not None:
                self._frame_timer.cancel()
            self._frame_timer = self.create_timer(
                CAPTURE_DELAY_SEC,
                lambda: self._frame_capture_step(label, az_deg),
            )

        future_start.add_done_callback(_start_done_cb)

    def _frame_capture_step(self, label: str, az_deg: float):
        # stop this timer so it only fires once
        if self._frame_timer is not None:
            self._frame_timer.cancel()
            self._frame_timer = None

        frame_req = Trigger.Request()
        future_frame = self.frame_capture_client.call_async(frame_req)

        def _frame_done_cb(fut2):
            try:
                resp2 = fut2.result()
                self.get_logger().info(
                    f"{self.drone_id}: frame_capture reply for window {label} "
                    f"(az={az_deg:.5f} deg): success={resp2.success}, msg='{resp2.message}'"
                )
            except Exception as e2:
                self.get_logger().error(
                    f"{self.drone_id}: frame_capture call failed for window {label}: {e2}"
                )

            # stop_capture regardless of frame result
            stop_req = Trigger.Request()
            future_stop = self.stop_capture_client.call_async(stop_req)

            def _stop_done_cb(fut3):
                try:
                    resp3 = fut3.result()
                    self.get_logger().info(
                        f"{self.drone_id}: stop_capture reply for window {label} "
                        f"(az={az_deg:.5f} deg): success={resp3.success}, msg='{resp3.message}'"
                    )
                except Exception as e3:
                    self.get_logger().error(
                        f"{self.drone_id}: stop_capture call failed for window {label}: {e3}"
                    )

                # now mark this window as handled
                self.already_triggered[label] = True
                self.capture_busy = False

            future_stop.add_done_callback(_stop_done_cb)

        future_frame.add_done_callback(_frame_done_cb)


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
