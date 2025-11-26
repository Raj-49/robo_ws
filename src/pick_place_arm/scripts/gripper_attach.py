#!/usr/bin/env python3
"""
Gripper attachment controller - publishes attach/detach commands.

Usage:
  python3 src/pick_place_arm/scripts/gripper_attach.py attach
  python3 src/pick_place_arm/scripts/gripper_attach.py detach
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import sys


def main():
    if len(sys.argv) < 2:
        print("Usage: gripper_attach.py [attach|detach]")
        return
    
    command = sys.argv[1].lower()
    
    rclpy.init()
    node = Node("gripper_attach_controller")
    
    # Publisher for attach/detach
    pub = node.create_publisher(Empty, '/box1/attach', 10)
    
    # Wait for connection
    import time
    time.sleep(0.5)
    
    if command == "attach":
        node.get_logger().info("Attaching box to gripper...")
        pub.publish(Empty())
        node.get_logger().info("✓ Attach command sent!")
    elif command == "detach":
        node.get_logger().info("Detaching box from gripper...")
        pub.publish(Empty())
        node.get_logger().info("✓ Detach command sent!")
    else:
        node.get_logger().error(f"Unknown command: {command}")
    
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
