#!/usr/bin/env python3
"""
RViz 3D Model Visualizer - Gazebo-Accurate Version
--------------------------------------------------
Publishes 3D visual markers matching Gazebo models exactly.
- Boxes: 5cm cubes with correct colors
- Tables: Cylinder pillars + square plates
- Baskets: Mesh files (OBJ) with correct scaling
- Smart publishing: Only updates when TF changes (no blinking)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import os
from ament_index_python.packages import get_package_share_directory

class RVizModelVisualizer(Node):
    def __init__(self):
        super().__init__("rviz_model_visualizer")
        
        # Track TF frames and their last known transforms
        self.model_transforms = {}
        self.last_published_transforms = {}
        
        # Subscribe to TF
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, '/model_markers', 10)
        
        # Get package path for mesh files
        try:
            self.package_share = get_package_share_directory('pick_place_arm')
        except:
            self.package_share = None
            self.get_logger().warn("Could not find pick_place_arm package share directory")
        
        # Timer to check for updates and publish (10Hz for smooth updates)
        self.timer = self.create_timer(0.1, self.publish_markers_if_changed)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("RViz 3D Model Visualizer started")
        self.get_logger().info("Publishing to /model_markers @ 10Hz")
        self.get_logger().info("")
        self.get_logger().info("In RViz:")
        self.get_logger().info("  1. Add → By topic → /model_markers → MarkerArray")
        self.get_logger().info("  2. Models will appear matching Gazebo exactly")
        self.get_logger().info("=" * 70)
    
    def tf_callback(self, msg):
        """Track TF transforms for models"""
        for transform in msg.transforms:
            frame_id = transform.child_frame_id
            # Only track model frames we care about
            if frame_id in ['red_box', 'green_box', 'blue_box', 
                           'pillar_table-1', 'pillar_table-2', 'pillar_table-3',
                           'basket_red', 'basket_green', 'basket_blue']:
                self.model_transforms[frame_id] = transform
    
    def has_transform_changed(self, frame_id, transform):
        """Check if transform has changed significantly"""
        if frame_id not in self.last_published_transforms:
            return True
        
        last = self.last_published_transforms[frame_id]
        current = transform.transform
        
        # Check position change (> 0.1mm - very sensitive)
        pos_changed = (
            abs(current.translation.x - last.translation.x) > 0.0001 or
            abs(current.translation.y - last.translation.y) > 0.0001 or
            abs(current.translation.z - last.translation.z) > 0.0001
        )
        
        # Check rotation change (> 0.001 radians - very sensitive)
        rot_changed = (
            abs(current.rotation.x - last.rotation.x) > 0.001 or
            abs(current.rotation.y - last.rotation.y) > 0.001 or
            abs(current.rotation.z - last.rotation.z) > 0.001 or
            abs(current.rotation.w - last.rotation.w) > 0.001
        )
        
        return pos_changed or rot_changed
    
    def publish_markers_if_changed(self):
        """Publish markers - always publish to keep them alive"""
        # Always publish to prevent expiration
        self.publish_markers()
        
        # Update last published transforms
        for frame_id, transform in self.model_transforms.items():
            self.last_published_transforms[frame_id] = transform.transform
    
    def publish_markers(self):
        """Publish all model markers"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Boxes (0.05m cubes)
        box_configs = [
            ('red_box', [1.0, 0.0, 0.0, 0.9]),
            ('green_box', [0.0, 1.0, 0.0, 0.9]),
            ('blue_box', [0.0, 0.0, 1.0, 0.9])
        ]
        
        for frame_id, rgba in box_configs:
            if frame_id in self.model_transforms:
                marker = self.create_box_marker(marker_id, frame_id, rgba)
                marker_array.markers.append(marker)
                marker_id += 1
        
        # Tables (cylinder pillar + square plate)
        table_configs = [
            ('pillar_table-1', [0.6, 0.4, 0.2, 0.8]),
            ('pillar_table-2', [0.6, 0.6, 0.2, 0.8]),
            ('pillar_table-3', [0.4, 0.6, 0.6, 0.8])
        ]
        
        for frame_id, rgba in table_configs:
            if frame_id in self.model_transforms:
                # Pillar (cylinder)
                pillar = self.create_table_pillar(marker_id, frame_id, rgba)
                marker_array.markers.append(pillar)
                marker_id += 1
                
                # Plate (cube)
                plate = self.create_table_plate(marker_id, frame_id, rgba)
                marker_array.markers.append(plate)
                marker_id += 1
        
        # Baskets (mesh or cube fallback)
        basket_configs = [
            ('basket_red', [1.0, 0.0, 0.0, 0.8]),
            ('basket_green', [0.0, 1.0, 0.0, 0.8]),
            ('basket_blue', [0.0, 0.0, 1.0, 0.8])
        ]
        
        for frame_id, rgba in basket_configs:
            if frame_id in self.model_transforms:
                marker = self.create_basket_marker(marker_id, frame_id, rgba)
                marker_array.markers.append(marker)
                marker_id += 1
        
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)
    
    def create_box_marker(self, marker_id, frame_id, rgba):
        """Create 5cm cube marker for box"""
        marker = Marker()
        marker.header.frame_id = frame_id
        # Use 0 timestamp to show at latest available transform (prevents flickering)
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "boxes"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position at frame origin
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale: 0.05m cube (exact from SDF)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        
        # Lifetime: 0 = forever (no expiration)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_table_pillar(self, marker_id, frame_id, rgba):
        """Create cylinder pillar for table (1m tall, 1cm radius)"""
        marker = Marker()
        marker.header.frame_id = frame_id
        # Use 0 timestamp to show at latest available transform (prevents flickering)
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "table_pillars"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position: center at z=0.5 (half of 1m height)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        
        # Scale: radius=0.01m, height=1m (from SDF)
        marker.scale.x = 0.02  # diameter (2 * radius)
        marker.scale.y = 0.02
        marker.scale.z = 1.0   # height
        
        # Color (darker for pillar)
        marker.color.r = rgba[0] * 0.5
        marker.color.g = rgba[1] * 0.5
        marker.color.b = rgba[2] * 0.5
        marker.color.a = rgba[3]
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_table_plate(self, marker_id, frame_id, rgba):
        """Create square plate for table (7cm x 7cm x 1cm)"""
        marker = Marker()
        marker.header.frame_id = frame_id
        # Use 0 timestamp to show at latest available transform (prevents flickering)
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "table_plates"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position: at z=1.005 (from SDF)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.005
        marker.pose.orientation.w = 1.0
        
        # Scale: 0.07m x 0.07m x 0.01m (exact from SDF)
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.01
        
        # Color
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_basket_marker(self, marker_id, frame_id, rgba):
        """Create basket marker (mesh if available, cube fallback)"""
        marker = Marker()
        marker.header.frame_id = frame_id
        # Use 0 timestamp to show at latest available transform (prevents flickering)
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "baskets"
        marker.id = marker_id
        marker.action = Marker.ADD
        
        # Try to use mesh
        color_name = frame_id.split('_')[1]  # 'red', 'green', or 'blue'
        mesh_path = None
        
        if self.package_share:
            mesh_file = os.path.join(self.package_share, 'models', f'basket_{color_name}', 'meshes', 'basket.obj')
            if os.path.exists(mesh_file):
                mesh_path = f"file://{mesh_file}"
        
        if mesh_path:
            # Use mesh
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = mesh_path
            
            # Scale from SDF: 0.003, 0.0015, 0.0015
            marker.scale.x = 0.003
            marker.scale.y = 0.0015
            marker.scale.z = 0.0015
            
            # Position at origin
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
        else:
            # Fallback to cube (approximate 15cm x 15cm x 8cm)
            marker.type = Marker.CUBE
            
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.08
            
            # Position: offset up by half height
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.04
            marker.pose.orientation.w = 1.0
        
        # Color
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker

def main():
    rclpy.init()
    node = RVizModelVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
