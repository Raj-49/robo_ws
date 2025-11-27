#!/usr/bin/env python3
"""
Visualize the robot arm's reachable workspace in RViz.
Samples points in 3D space and tests IK to determine reachability.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
import sys
import os

# Add the scripts directory to path to import ik_utils
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from ik_utils import IKSolver

class WorkspaceVisualizer(Node):
    def __init__(self):
        super().__init__('workspace_visualizer')
        
        # Create marker publisher
        self.marker_pub = self.create_publisher(Marker, '/workspace_cloud', 10)
        
        # Initialize IK solver
        self.ik_solver = IKSolver(self)
        
        # Workspace sampling parameters
        self.x_range = (0.1, 0.6)   # 10cm to 60cm from base
        self.y_range = (-0.4, 0.4)  # -40cm to +40cm
        self.z_range = (0.05, 0.4)  # 5cm to 40cm height
        self.resolution = 0.05      # 5cm grid spacing
        
        self.get_logger().info("Workspace Visualizer started!")
        self.get_logger().info(f"Sampling workspace:")
        self.get_logger().info(f"  X: {self.x_range[0]} to {self.x_range[1]}m")
        self.get_logger().info(f"  Y: {self.y_range[0]} to {self.y_range[1]}m")
        self.get_logger().info(f"  Z: {self.z_range[0]} to {self.z_range[1]}m")
        self.get_logger().info(f"  Resolution: {self.resolution}m")
        
        # Start workspace computation
        self.timer = self.create_timer(1.0, self.compute_workspace)
        self.workspace_computed = False
        
    def compute_workspace(self):
        """Sample workspace and test reachability"""
        if self.workspace_computed:
            return
            
        self.get_logger().info("Computing reachable workspace... (this may take a minute)")
        
        # Generate sample points
        x_points = np.arange(self.x_range[0], self.x_range[1], self.resolution)
        y_points = np.arange(self.y_range[0], self.y_range[1], self.resolution)
        z_points = np.arange(self.z_range[0], self.z_range[1], self.resolution)
        
        total_points = len(x_points) * len(y_points) * len(z_points)
        self.get_logger().info(f"Testing {total_points} points...")
        
        reachable_points = []
        unreachable_points = []
        
        count = 0
        for x in x_points:
            for y in y_points:
                for z in z_points:
                    count += 1
                    if count % 100 == 0:
                        self.get_logger().info(f"Progress: {count}/{total_points}")
                    
                    # Test if this position is reachable
                    position = [x, y, z]
                    joint_positions = self.ik_solver.get_ik(position, avoid_collisions=False)
                    
                    if joint_positions is not None:
                        reachable_points.append(position)
                    else:
                        unreachable_points.append(position)
        
        self.get_logger().info(f"✅ Workspace computation complete!")
        self.get_logger().info(f"  Reachable: {len(reachable_points)} points")
        self.get_logger().info(f"  Unreachable: {len(unreachable_points)} points")
        self.get_logger().info(f"  Coverage: {100*len(reachable_points)/total_points:.1f}%")
        
        # Publish workspace visualization
        self.publish_workspace(reachable_points, unreachable_points)
        self.workspace_computed = True
        
    def publish_workspace(self, reachable, unreachable):
        """Publish workspace as point cloud markers"""
        # Reachable points (green)
        reachable_marker = Marker()
        reachable_marker.header.frame_id = "world"
        reachable_marker.header.stamp = self.get_clock().now().to_msg()
        reachable_marker.ns = "reachable"
        reachable_marker.id = 0
        reachable_marker.type = Marker.POINTS
        reachable_marker.action = Marker.ADD
        reachable_marker.pose.orientation.w = 1.0
        
        # Point size
        reachable_marker.scale.x = 0.02
        reachable_marker.scale.y = 0.02
        
        # Green color
        reachable_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
        
        # Add points
        for pos in reachable:
            p = Point()
            p.x, p.y, p.z = pos
            reachable_marker.points.append(p)
        
        # Unreachable points (red) - optional, can be commented out to reduce clutter
        unreachable_marker = Marker()
        unreachable_marker.header.frame_id = "world"
        unreachable_marker.header.stamp = self.get_clock().now().to_msg()
        unreachable_marker.ns = "unreachable"
        unreachable_marker.id = 1
        unreachable_marker.type = Marker.POINTS
        unreachable_marker.action = Marker.ADD
        unreachable_marker.pose.orientation.w = 1.0
        
        unreachable_marker.scale.x = 0.015
        unreachable_marker.scale.y = 0.015
        
        # Red color, more transparent
        unreachable_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.1)
        
        for pos in unreachable:
            p = Point()
            p.x, p.y, p.z = pos
            unreachable_marker.points.append(p)
        
        # Publish markers repeatedly
        self.timer.cancel()
        self.timer = self.create_timer(0.5, lambda: self.publish_markers(reachable_marker, unreachable_marker))
        
    def publish_markers(self, reachable_marker, unreachable_marker):
        """Continuously publish workspace markers"""
        reachable_marker.header.stamp = self.get_clock().now().to_msg()
        unreachable_marker.header.stamp = self.get_clock().now().to_msg()
        
        self.marker_pub.publish(reachable_marker)
        self.marker_pub.publish(unreachable_marker)

def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*60)
    print("WORKSPACE VISUALIZER")
    print("="*60)
    print("This will compute and visualize the robot's reachable workspace.")
    print("Green points = reachable")
    print("Red points = unreachable")
    print("\nIn RViz, add: /workspace_cloud (Marker)")
    print("="*60 + "\n")
    
    node = WorkspaceVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
