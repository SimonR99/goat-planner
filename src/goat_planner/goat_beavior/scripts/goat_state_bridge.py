#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict
from dataclasses import asdict

from goat_planner.goat_state import GoatState
from goat_planner.goat_controller import GoatController

class GoatStateBridge(Node):
    def __init__(self):
        super().__init__('goat_state_bridge')
        
        # Initialize GoatState
        self.goat_state = GoatState()
        
        # Initialize GoatController
        self.controller = GoatController()
        
        # Create subscription to shepherd detections
        self.create_subscription(
            String,
            '/shepherd/object_detections',
            self.object_detection_callback,
            10
        )
        
        # Create publisher for state updates
        self.state_publisher = self.create_publisher(
            String,
            '/goat/world_state',
            10
        )
        
        # Timer to periodically publish state
        self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('GoatStateBridge node initialized')

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
            
            # Debug log the properties
            self.get_logger().debug(f"Object properties: {properties}")
            
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