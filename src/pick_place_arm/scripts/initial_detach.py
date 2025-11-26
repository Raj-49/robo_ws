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
        
        # Publisher
        self.detach_pub = self.create_publisher(Empty, '/box1/detach', 10)
        
        # Wait for bridge to be ready
        time.sleep(2.0)
        
        # Send detach commands
        for i in range(10):
            self.detach_pub.publish(Empty())
            time.sleep(0.5)
        
        self.get_logger().info("Initial detach complete - Box is free")
        
        # Shutdown after completing
        rclpy.shutdown()


def main():
    rclpy.init()
    node = InitialDetachNode()
    # Note: Node shuts down automatically after detach sequence
    # No need to spin


if __name__ == "__main__":
    main()
