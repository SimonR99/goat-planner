#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict
from dataclasses import asdict

from goat_planner.goat_state import GoatState
from goat_planner.goat_controller import GoatController
from goat_planner.utils.json_xml_convertor import json_to_xml

class GoatStateBridge(Node):
    def __init__(self):
        super().__init__('goat_state_bridge')
        
        # Initialize GoatState
        self.goat_state = GoatState()
        
        # Initialize GoatController with callback for plan updates
        self.controller = GoatController(
            on_plan_update_callback=self.on_plan_update
        )
        
        # Create subscription to shepherd detections
        self.create_subscription(
            String,
            '/shepherd/object_detections',
            self.object_detection_callback,
            10
        )
        
        # Create publishers
        self.state_publisher = self.create_publisher(
            String,
            '/goat/world_state',
            10
        )
        
        # Add behavior tree publisher
        self.behavior_publisher = self.create_publisher(
            String,
            '/goat/behavior_tree',
            10
        )
        
        # Timer to periodically publish state and behavior tree
        self.create_timer(1.0, self.publish_state)
        self.create_timer(0.1, self.publish_behavior_tree)  # Higher frequency for behavior tree
        
        # Store last published behavior tree to avoid unnecessary updates
        self.last_published_tree = None
        
        self.get_logger().info('GoatStateBridge node initialized')

    def on_plan_update(self, tree: Dict):
        """Callback for when the behavior tree is updated"""
        try:
            self.publish_behavior_tree()
        except Exception as e:
            self.get_logger().error(f'Error in plan update callback: {str(e)}')

    def _convert_values_to_str(self, data):
        """Recursively convert all values in a dictionary to strings"""
        if isinstance(data, dict):
            return {k: self._convert_values_to_str(v) for k, v in data.items()}
        elif isinstance(data, list):
            return [self._convert_values_to_str(item) for item in data]
        elif isinstance(data, (int, float, bool)):
            return str(data)
        return data

    def publish_behavior_tree(self):
        """Publish the current behavior tree in XML format"""
        try:
            # Get current behavior tree
            tree = self.goat_state.get_behavior_tree()
            
            # Skip if tree is empty or unchanged
            if not tree or tree == self.last_published_tree:
                return
                
            # Convert all numeric values to strings
            tree = self._convert_values_to_str(tree)
            
            # Convert tree to proper format for XML conversion
            tree_wrapper = {
                "root": {
                    "BehaviorTree": tree
                }
            }
            
            # Convert to XML
            xml_tree = json_to_xml(json.dumps(tree_wrapper))
            
            # Create and publish message
            tree_msg = String()
            tree_msg.data = xml_tree
            self.behavior_publisher.publish(tree_msg)
            
            # Update last published tree
            self.last_published_tree = tree
            
            self.get_logger().debug('Published behavior tree update')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing behavior tree: {str(e)}')

    def object_detection_callback(self, msg: String):
        """Handle new object detections from Shepherd"""
        try:
            detection = json.loads(msg.data)
            
            # Debug log the received detection
            self.get_logger().debug(f"Received detection: {detection}")
            
            # Extract object information
            object_id = detection.get('object_id')
            if not object_id:
                self.get_logger().warn("Received detection without object_id")
                return
                
            # Get object position from point cloud centroid
            position = detection.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0})
            
            # Get caption and log it
            caption = detection.get('metadata', {}).get('caption', '')
            self.get_logger().info(f"Processing object {object_id} with caption: {caption}")
            
            # Create properties dictionary with relevant information
            properties = {
                'caption': caption,
                'similarity': detection.get('similarity'),
                'confidence': detection.get('detection', {}).get('confidence'),
                'class': detection.get('detection', {}).get('class'),
                'timestamp': detection.get('timestamp')
            }
            
            # Update the world object in GoatState
            self.controller.update_world_object(
                obj_id=object_id,
                obj_type='detected_object',
                position=position,
                properties=properties
            )
            
            self.get_logger().info(f'Updated object {object_id} in GoatState')
            
        except Exception as e:
            self.get_logger().error(f'Error processing object detection: {str(e)}')

    def publish_state(self):
        """Publish the current world state"""
        try:
            # Get current state - already serializable
            state = self.goat_state.get_full_state()
            
            # Convert to JSON string
            state_msg = String()
            state_msg.data = json.dumps(state)
            
            # Publish
            self.state_publisher.publish(state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing state: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = GoatStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 