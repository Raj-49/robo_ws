#!/usr/bin/env python3
"""
Pre-detach all colored boxes at startup.
This script should be run after Gazebo launches to free the boxes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time

class PreDetachBoxes(Node):
    def __init__(self):
        super().__init__('pre_detach_boxes')
        
        # Create detach publishers for all boxes
        self.red_detach = self.create_publisher(Empty, '/red_box/detach', 10)
        self.green_detach = self.create_publisher(Empty, '/green_box/detach', 10)
        self.blue_detach = self.create_publisher(Empty, '/blue_box/detach', 10)
        
        self.get_logger().info("Pre-detach script started")
        
        # Wait for publishers to be ready
        time.sleep(2.0)
        
        # Detach all boxes
        self.get_logger().info("Detaching red_box...")
        self.red_detach.publish(Empty())
        time.sleep(0.5)
        
        self.get_logger().info("Detaching green_box...")
        self.green_detach.publish(Empty())
        time.sleep(0.5)
        
        self.get_logger().info("Detaching blue_box...")
        self.blue_detach.publish(Empty())
        time.sleep(0.5)
        
        self.get_logger().info("✅ All boxes pre-detached and free!")
        self.get_logger().info("Boxes can now be attached/detached at will")

def main(args=None):
    rclpy.init(args=args)
    node = PreDetachBoxes()
    
    # Keep node alive for a moment
    time.sleep(1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
