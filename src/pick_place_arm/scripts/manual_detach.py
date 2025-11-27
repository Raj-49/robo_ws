#!/usr/bin/env python3
"""
Manual detach script - run this to free all boxes
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time

def main():
    rclpy.init()
    node = Node('manual_detach')
    
    # Create publishers
    red_detach = node.create_publisher(Empty, '/red_box/detach', 10)
    green_detach = node.create_publisher(Empty, '/green_box/detach', 10)
    blue_detach = node.create_publisher(Empty, '/blue_box/detach', 10)
    
    print("Waiting for publishers to be ready...")
    time.sleep(1.0)
    
    print("Detaching all boxes...")
    for i in range(10):
        red_detach.publish(Empty())
        green_detach.publish(Empty())
        blue_detach.publish(Empty())
        time.sleep(0.2)
        print(f"  Sent detach command {i+1}/10")
    
    print("✅ All boxes should now be detached!")
    print("Check Gazebo - boxes should fall if they were attached")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
