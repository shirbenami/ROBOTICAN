#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros

class PrintTfFrames(Node):
    def __init__(self):
        super().__init__("print_tf_frames")
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(2.0, self.on_timer)
        self.get_logger().info("Collecting TF frames for 2 seconds...")

    def on_timer(self):
        try:
            yaml = self.tf_buffer.all_frames_as_yaml()
            print("\n=== TF FRAMES (YAML) ===\n")
            print(yaml)
        except Exception as e:
            self.get_logger().error(f"Failed to get TF frames: {e}")
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = PrintTfFrames()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
