#!/usr/bin/env python3
"""
RViz Interactive Control & Visualization
----------------------------------------
Spawns Interactive Markers for Gazebo objects in RViz.
- Visualizes Boxes, Tables, and Baskets attached to their TF frames
- Provides Context Menu for Boxes: "Attach" and "Detach"
"""

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from std_msgs.msg import Empty
from geometry_msgs.msg import Point

class RVizInteractiveControl(Node):
    def __init__(self):
        super().__init__("rviz_interactive_control")
        
        self.server = InteractiveMarkerServer(self, "box_controls")
        self.menu_handler = MenuHandler()
        
        # Menu entries
        self.entry_attach = self.menu_handler.insert("Attach", callback=self.process_feedback)
        self.entry_detach = self.menu_handler.insert("Detach", callback=self.process_feedback)
        
        # Publishers
        self.pubs = {
            'red': {
                'attach': self.create_publisher(Empty, '/red_box/attach', 10),
                'detach': self.create_publisher(Empty, '/red_box/detach', 10)
            },
            'green': {
                'attach': self.create_publisher(Empty, '/green_box/attach', 10),
                'detach': self.create_publisher(Empty, '/green_box/detach', 10)
            },
            'blue': {
                'attach': self.create_publisher(Empty, '/blue_box/attach', 10),
                'detach': self.create_publisher(Empty, '/blue_box/detach', 10)
            }
        }
        
        # Create Markers
        self.create_box_marker("red_box", "red", [1.0, 0.0, 0.0])
        self.create_box_marker("green_box", "green", [0.0, 1.0, 0.0])
        self.create_box_marker("blue_box", "blue", [0.0, 0.0, 1.0])
        
        # Create Table Markers (Visual only, no menu)
        # Colors: Table 1 (Magenta), Table 2 (Yellow), Table 3 (Cyan)
        self.create_static_marker("pillar_table-1", "table1", [1.0, 0.0, 1.0], scale=[0.7, 0.7, 0.02])
        self.create_static_marker("pillar_table-2", "table2", [1.0, 1.0, 0.0], scale=[0.7, 0.7, 0.02])
        self.create_static_marker("pillar_table-3", "table3", [0.0, 1.0, 1.0], scale=[0.7, 0.7, 0.02])
        
        # Create Basket Markers
        self.create_static_marker("basket_red", "basket_red", [1.0, 0.0, 0.0], scale=[0.3, 0.3, 0.1])
        self.create_static_marker("basket_green", "basket_green", [0.0, 1.0, 0.0], scale=[0.3, 0.3, 0.1])
        self.create_static_marker("basket_blue", "basket_blue", [0.0, 0.0, 1.0], scale=[0.3, 0.3, 0.1])
        
        self.server.applyChanges()
        self.get_logger().info("RViz Interactive Controls Ready! Markers should be visible.")
        self.get_logger().info(f"Created markers for: {list(self.pubs.keys())} and tables/baskets")

    def create_box_marker(self, frame_id, name, rgb):
        """Create interactive marker for a box"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.scale = 0.1
        
        # Visual Control (Box)
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = float(rgb[0])
        marker.color.g = float(rgb[1])
        marker.color.b = float(rgb[2])
        marker.color.a = 0.8
        
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        self.server.insert(int_marker)
        self.menu_handler.apply(self.server, name)

    def create_static_marker(self, frame_id, name, rgb, scale):
        """Create visual-only marker for tables/baskets"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.scale = 0.1
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.NONE # Non-interactive
        
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = float(rgb[0])
        marker.color.g = float(rgb[1])
        marker.color.b = float(rgb[2])
        marker.color.a = 0.6
        
        # Offset for table plate (approximate based on SDF)
        if "table" in name:
             marker.pose.position.z = 1.0 # Plate height relative to origin? 
             # Actually, Gazebo model origin is usually at bottom or center.
             # In SDF, plate is at z=1.005 relative to model origin.
             marker.pose.position.z = 1.0
        
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        self.server.insert(int_marker)

    def process_feedback(self, feedback):
        """Handle menu clicks"""
        if feedback.event_type == 2: # MENU_SELECT
            color = feedback.marker_name # 'red', 'green', 'blue'
            
            if feedback.menu_entry_id == self.entry_attach:
                self.get_logger().info(f"Attaching {color} box...")
                if color in self.pubs:
                    self.pubs[color]['attach'].publish(Empty())
            
            elif feedback.menu_entry_id == self.entry_detach:
                self.get_logger().info(f"Detaching {color} box...")
                if color in self.pubs:
                    self.pubs[color]['detach'].publish(Empty())

def main():
    rclpy.init()
    node = RVizInteractiveControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
