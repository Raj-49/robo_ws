#!/usr/bin/env python3
"""
Silent Initial Detach Script
-----------------------------
Sends detach commands on startup to clear any pre-attachment.
Runs automatically from launch file.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time


class InitialDetachNode(Node):
    def __init__(self):
        super().__init__('initial_detach_node')
        
        # Publishers for all colored boxes
        self.red_detach = self.create_publisher(Empty, '/red_box/detach', 10)
        self.green_detach = self.create_publisher(Empty, '/green_box/detach', 10)
        self.blue_detach = self.create_publisher(Empty, '/blue_box/detach', 10)
        
        # Wait longer for Gazebo and bridge to be fully ready
        self.get_logger().info("Waiting for Gazebo to be ready...")
        time.sleep(3.0)
        
        # Send detach commands
        self.get_logger().info("Detaching all colored boxes...")
        
        # Send multiple times to ensure receipt
        for i in range(10):
            self.red_detach.publish(Empty())
            self.green_detach.publish(Empty())
            self.blue_detach.publish(Empty())
            time.sleep(0.3)
        
        self.get_logger().info("✅ Initial detach complete - All boxes are free")
        
        # Shutdown after completing
        rclpy.shutdown()


def main():
    rclpy.init()
    node = InitialDetachNode()
    # Note: Node shuts down automatically after detach sequence
    # No need to spin


if __name__ == "__main__":
    main()
