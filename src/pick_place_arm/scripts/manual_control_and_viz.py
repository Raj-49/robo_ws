#!/usr/bin/env python3
"""
Manual Control & Visualization Script
-------------------------------------
1. Publishes static Markers to RViz for Tables, Baskets, and Boxes.
2. Provides manual Attach/Detach control via terminal.
3. Prints current joint states for recording.

Bypasses Gazebo bridge/relay issues by using hardcoded coordinates.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
import threading
import sys
import time

class ManualControlViz(Node):
    def __init__(self):
        super().__init__("manual_control_viz")
        
        # Publishers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Publishers for attachment
        self.attach_pubs = {
            'red': self.create_publisher(Empty, '/red_box/attach', 10),
            'green': self.create_publisher(Empty, '/green_box/attach', 10),
            'blue': self.create_publisher(Empty, '/blue_box/attach', 10)
        }
        self.detach_pubs = {
            'red': self.create_publisher(Empty, '/red_box/detach', 10),
            'green': self.create_publisher(Empty, '/green_box/detach', 10),
            'blue': self.create_publisher(Empty, '/blue_box/detach', 10)
        }
        
        # Joint state subscription
        self.latest_joints = None
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # Timer for publishing markers
        self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info("Manual Control & Viz Node Started")

    def joint_callback(self, msg):
        self.latest_joints = msg

    def publish_markers(self):
        ma = MarkerArray()
        id_counter = 0
        timestamp = self.get_clock().now().to_msg()
        
        # Helper to create marker
        def create_marker(ns, type, pos, scale, color, frame="world"):
            nonlocal id_counter
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = timestamp
            m.ns = ns
            m.id = id_counter
            id_counter += 1
            m.type = type
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            m.scale.x = scale[0]
            m.scale.y = scale[1]
            m.scale.z = scale[2]
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 0.8
            return m

        # --- TABLES ---
        # Table 1 (Magenta)
        # Stand
        ma.markers.append(create_marker("tables", Marker.CYLINDER, [0.1351, -0.4045, -0.6886 + 0.5], [0.02, 0.02, 1.0], [0.3, 0.3, 0.3]))
        # Plate (Magenta)
        ma.markers.append(create_marker("tables", Marker.CUBE, [0.1351, -0.4045, -0.6886 + 1.005], [0.07, 0.07, 0.01], [1.0, 0.0, 1.0]))
        
        # Table 2 (Yellow)
        # Stand
        ma.markers.append(create_marker("tables", Marker.CYLINDER, [0.4673, -0.2480, -0.6703 + 0.5], [0.02, 0.02, 1.0], [0.3, 0.3, 0.3]))
        # Plate (Yellow)
        ma.markers.append(create_marker("tables", Marker.CUBE, [0.4673, -0.2480, -0.6703 + 1.005], [0.07, 0.07, 0.01], [1.0, 1.0, 0.0]))
        
        # Table 3 (Cyan)
        # Stand
        ma.markers.append(create_marker("tables", Marker.CYLINDER, [-0.1886, -0.2271, -0.7102 + 0.5], [0.02, 0.02, 1.0], [0.3, 0.3, 0.3]))
        # Plate (Cyan)
        ma.markers.append(create_marker("tables", Marker.CUBE, [-0.1886, -0.2271, -0.7102 + 1.005], [0.07, 0.07, 0.01], [0.0, 1.0, 1.0]))

        # --- BASKETS ---
        # Red Basket
        ma.markers.append(create_marker("baskets", Marker.MESH_RESOURCE, [0.1488, 0.3298, 0.0], [0.003, 0.0015, 0.003], [1.0, 0.0, 0.0]))
        ma.markers[-1].mesh_resource = "package://pick_place_arm/models/basket_red/meshes/basket.obj"
        
        # Green Basket
        ma.markers.append(create_marker("baskets", Marker.MESH_RESOURCE, [-0.0101, 0.2789, 0.0], [0.003, 0.0015, 0.003], [0.0, 1.0, 0.0]))
        ma.markers[-1].mesh_resource = "package://pick_place_arm/models/basket_red/meshes/basket.obj"
        
        # Blue Basket
        ma.markers.append(create_marker("baskets", Marker.MESH_RESOURCE, [0.3059, 0.2712, 0.0], [0.003, 0.0015, 0.003], [0.0, 0.0, 1.0]))
        ma.markers[-1].mesh_resource = "package://pick_place_arm/models/basket_red/meshes/basket.obj"

        # --- BOXES (Initial Positions) ---
        # Red Box (on Table 2)
        ma.markers.append(create_marker("boxes", Marker.CUBE, [0.1388, -0.3719, 0.3497], [0.05, 0.05, 0.05], [1.0, 0.0, 0.0]))
        # Green Box (on Table 3)
        ma.markers.append(create_marker("boxes", Marker.CUBE, [0.4413, -0.2343, 0.3647], [0.05, 0.05, 0.05], [0.0, 1.0, 0.0]))
        # Blue Box (on Table 1)
        ma.markers.append(create_marker("boxes", Marker.CUBE, [-0.1601, -0.2155, 0.3248], [0.05, 0.05, 0.05], [0.0, 0.0, 1.0]))
        
        self.marker_pub.publish(ma)

    def print_joints(self):
        if not self.latest_joints:
            print("No joint states received yet.")
            return
        
        # Filter for arm joints (j1-j6)
        arm_joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        positions = []
        try:
            for name in arm_joints:
                if name in self.latest_joints.name:
                    idx = self.latest_joints.name.index(name)
                    positions.append(self.latest_joints.position[idx])
            
            if len(positions) == 6:
                print("\n" + "="*40)
                print(f"JOINT POSITIONS: [{', '.join([f'{p:.6f}' for p in positions])}]")
                print("="*40 + "\n")
            else:
                print(f"Found {len(positions)}/6 arm joints.")
        except Exception as e:
            print(f"Error: {e}")

    def attach(self, color):
        # Detach ALL boxes first to ensure clean state
        for c in ['red', 'green', 'blue']:
            self.detach_pubs[c].publish(Empty())
        
        # Small delay to allow detach to process
        time.sleep(0.5)
        
        print(f"Attaching {color} box (others detached)...")
        self.attach_pubs[color].publish(Empty())

    def detach(self, color):
        print(f"Detaching {color} box...")
        self.detach_pubs[color].publish(Empty())

def menu_loop(node):
    while rclpy.ok():
        print("\n--- MANUAL CONTROL ---")
        print("1. Attach BLUE Box (Table 1 - Magenta)")
        print("2. Attach RED Box (Table 2 - Yellow)")
        print("3. Attach GREEN Box (Table 3 - Cyan)")
        print("4. Detach ALL")
        print("5. Print Joint Positions")
        print("0. Exit")
        
        choice = input("Choice: ").strip()
        
        if choice == '0':
            rclpy.shutdown()
            break
        elif choice == '1':
            node.attach('blue')
        elif choice == '2':
            node.attach('red')
        elif choice == '3':
            node.attach('green')
        elif choice == '4':
            node.detach('red')
            node.detach('green')
            node.detach('blue')
        elif choice == '5':
            node.print_joints()
        else:
            print("Invalid choice")

def main():
    rclpy.init()
    node = ManualControlViz()
    
    # Run ROS spin in separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        menu_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
