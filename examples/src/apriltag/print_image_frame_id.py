#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class PrintImageFrameId(Node):
    def __init__(self):
        super().__init__("print_image_frame_id")
        self.sub = self.create_subscription(
            Image,
            "/R1/camera/image_raw",
            self.cb,
            10
        )
        self.get_logger().info("Waiting for one Image message on /R1/camera/image_raw ...")

    def cb(self, msg: Image):
        self.get_logger().info(f"Image header.frame_id = '{msg.header.frame_id}'")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PrintImageFrameId()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
