#!/usr/bin/env python3
"""
Debug script to check DetachableJoint topics and test attach/detach
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time
import subprocess

def main():
    print("="*60)
    print("DetachableJoint Debug Script")
    print("="*60)
    
    # Check available topics
    print("\n1. Checking available topics...")
    result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
    topics = result.stdout.split('\n')
    
    detach_topics = [t for t in topics if 'detach' in t.lower()]
    attach_topics = [t for t in topics if 'attach' in t.lower()]
    
    print(f"\nFound {len(detach_topics)} detach topics:")
    for topic in detach_topics:
        if topic:
            print(f"  - {topic}")
    
    print(f"\nFound {len(attach_topics)} attach topics:")
    for topic in attach_topics:
        if topic:
            print(f"  - {topic}")
    
    # Initialize ROS
    rclpy.init()
    node = Node('detach_debug')
    
    # Create publishers for all possible topic variations
    publishers = {}
    for box in ['red_box', 'green_box', 'blue_box']:
        publishers[f'{box}_detach'] = node.create_publisher(Empty, f'/{box}/detach', 10)
        publishers[f'{box}_attach'] = node.create_publisher(Empty, f'/{box}/attach', 10)
    
    print("\n2. Waiting for publishers to be ready...")
    time.sleep(2.0)
    
    print("\n3. Sending detach commands to all boxes...")
    for i in range(15):
        for box in ['red_box', 'green_box', 'blue_box']:
            publishers[f'{box}_detach'].publish(Empty())
        time.sleep(0.3)
        print(f"  Sent detach round {i+1}/15")
    
    print("\n✅ Detach commands sent!")
    print("\nCheck Gazebo:")
    print("  - If boxes fall/drop, detach is working")
    print("  - If boxes stay attached to gripper, plugin may not be initialized")
    
    print("\n4. Testing attach on red_box...")
    time.sleep(2.0)
    for i in range(5):
        publishers['red_box_attach'].publish(Empty())
        time.sleep(0.2)
        print(f"  Sent attach command {i+1}/5")
    
    print("\n✅ Attach commands sent to red_box!")
    print("Check if red_box attaches to gripper")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
