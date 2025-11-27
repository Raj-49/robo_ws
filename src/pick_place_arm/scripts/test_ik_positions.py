#!/usr/bin/env python3
"""
Visualize the 3 colored boxes in RViz at their exact Gazebo positions.
Includes attach/detach functionality to simulate picking.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
import threading
import sys
import select

class BoxVisualizer(Node):
    def __init__(self):
        super().__init__('box_visualizer')
        
        # Create marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/box_markers', 10)
        
        # Create planning scene service client
        self.planning_scene_client = self.create_client(
            ApplyPlanningScene, 
            '/apply_planning_scene'
        )
        
        # Box definitions from my_world.sdf
        self.boxes = {
            'red_box': {
                'position': [0.4065, 0.1622, 0.1815],
                'color': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                'id': 0,
                'attached': False
            },
            'green_box': {
                'position': [0.4065, -0.085, 0.1815],
                'color': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                'id': 1,
                'attached': False
            },
            'blue_box': {
                'position': [0.4065, 0.045, 0.1815],
                'color': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
                'id': 2,
                'attached': False
            }
        }
        
        # Box size from my_world.sdf (0.05 x 0.05 x 0.05)
        self.box_size = 0.05
        
        # Create timer to publish markers
        self.timer = self.create_timer(0.5, self.publish_markers)
        
        # Start interactive menu in separate thread
        self.menu_thread = threading.Thread(target=self.interactive_menu, daemon=True)
        self.menu_thread.start()
        
        self.get_logger().info("Box Visualizer with Attach/Detach started!")
        self.get_logger().info("Publishing 3 colored boxes in RViz:")
        for name, data in self.boxes.items():
            pos = data['position']
            self.get_logger().info(f"  {name}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
        
    def publish_markers(self):
        """Publish marker array for all boxes"""
        marker_array = MarkerArray()
        
        for name, data in self.boxes.items():
            # Skip if attached (will be shown by MoveIt)
            if data['attached']:
                continue
                
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "boxes"
            marker.id = data['id']
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = data['position'][0]
            marker.pose.position.y = data['position'][1]
            marker.pose.position.z = data['position'][2]
            marker.pose.orientation.w = 1.0
            
            # Set size (0.05 x 0.05 x 0.05 from SDF)
            marker.scale.x = self.box_size
            marker.scale.y = self.box_size
            marker.scale.z = self.box_size
            
            # Set color
            marker.color = data['color']
            
            # Marker lifetime
            marker.lifetime.sec = 0  # 0 = forever
            
            marker_array.markers.append(marker)
            
            # Add text label above each box
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "labels"
            text_marker.id = data['id'] + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position text above box
            text_marker.pose.position.x = data['position'][0]
            text_marker.pose.position.y = data['position'][1]
            text_marker.pose.position.z = data['position'][2] + 0.08
            text_marker.pose.orientation.w = 1.0
            
            # Text properties
            status = "ATTACHED" if data['attached'] else "FREE"
            text_marker.text = f"{name} ({status})\n[{data['position'][0]:.3f}, {data['position'][1]:.3f}, {data['position'][2]:.3f}]"
            text_marker.scale.z = 0.03  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def attach_box(self, box_name):
        """Attach box to end-effector in MoveIt planning scene"""
        if box_name not in self.boxes:
            self.get_logger().error(f"Unknown box: {box_name}")
            return
        
        box_data = self.boxes[box_name]
        
        # Create attached collision object
        aco = AttachedCollisionObject()
        aco.link_name = "l6"  # End-effector link
        aco.object.header.frame_id = "world"
        aco.object.id = box_name
        
        # Define box shape
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [self.box_size, self.box_size, self.box_size]
        
        # Set box pose
        aco.object.primitives.append(primitive)
        pose = aco.object.primitive_poses.add()
        pose.position.x = box_data['position'][0]
        pose.position.y = box_data['position'][1]
        pose.position.z = box_data['position'][2]
        pose.orientation.w = 1.0
        
        aco.object.operation = CollisionObject.ADD
        
        # Apply to planning scene
        from moveit_msgs.msg import PlanningScene
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects.append(aco)
        scene.is_diff = True
        
        # Call service
        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        future = self.planning_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().success:
            box_data['attached'] = True
            self.get_logger().info(f"✅ Attached {box_name} to end-effector")
        else:
            self.get_logger().error(f"❌ Failed to attach {box_name}")
    
    def detach_box(self, box_name):
        """Detach box from end-effector"""
        if box_name not in self.boxes:
            self.get_logger().error(f"Unknown box: {box_name}")
            return
        
        box_data = self.boxes[box_name]
        
        # Create detach message
        aco = AttachedCollisionObject()
        aco.object.id = box_name
        aco.object.operation = CollisionObject.REMOVE
        
        from moveit_msgs.msg import PlanningScene
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects.append(aco)
        scene.is_diff = True
        
        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        future = self.planning_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().success:
            box_data['attached'] = False
            self.get_logger().info(f"✅ Detached {box_name}")
        else:
            self.get_logger().error(f"❌ Failed to detach {box_name}")
    
    def interactive_menu(self):
        """Interactive menu for attach/detach"""
        import time
        time.sleep(2)  # Wait for node to initialize
        
        print("\n" + "="*60)
        print("BOX ATTACH/DETACH MENU")
        print("="*60)
        print("Commands:")
        print("  a <box> - Attach box (e.g., 'a red_box')")
        print("  d <box> - Detach box (e.g., 'd red_box')")
        print("  s       - Show status")
        print("  q       - Quit")
        print("="*60)
        
        while rclpy.ok():
            try:
                cmd = input("\nEnter command: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    print("\nBox Status:")
                    for name, data in self.boxes.items():
                        status = "ATTACHED" if data['attached'] else "FREE"
                        print(f"  {name}: {status}")
                elif cmd.startswith('a '):
                    box_name = cmd[2:].strip()
                    self.attach_box(box_name)
                elif cmd.startswith('d '):
                    box_name = cmd[2:].strip()
                    self.detach_box(box_name)
                else:
                    print("Unknown command. Use 'a <box>', 'd <box>', 's', or 'q'")
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BoxVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
