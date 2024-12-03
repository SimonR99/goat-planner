#!/usr/bin/env python3

import json
import subprocess
from datetime import datetime
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from goat_planner.goat_controller import GoatController
from goat_planner.goat_state import GoatState


class GoatStateBridge(Node):
    def __init__(self):
        super().__init__("goat_state_bridge")

        # Initialize GoatState
        self.goat_state = GoatState()

        # Initialize GoatController with callback for plan updates
        self.controller = GoatController(on_plan_update_callback=self.on_plan_update)

        # Create subscription to shepherd detections
        self.create_subscription(
            String, "/shepherd/object_detections", self.object_detection_callback, 10
        )

        # Create subscription for user commands
        self.create_subscription(
            String, "/goat/user_commands", self.user_command_callback, 10
        )

        # Create publishers
        self.state_publisher = self.create_publisher(String, "/goat/world_state", 10)

        # Timer to periodically publish state
        self.create_timer(1.0, self.publish_state)

        # Timer to check for updates in the behavior tree
        self.create_timer(1.0, self.check_for_plan_updates)

        # Initialize tracking for the last update time of the behavior tree
        self.last_behavior_tree_update_time = None

        # Add flag to track if behavior manager is running
        self.behavior_manager_process = None

        self.get_logger().info("GoatStateBridge node initialized")

    def on_plan_update(self, tree: Dict):
        """Callback for when the behavior tree is updated"""
        try:
            self.get_logger().info("Behavior tree updated, running behavior manager")
            self.run_behavior_manager()
        except Exception as e:
            self.get_logger().error(f"Error in plan update callback: {str(e)}")

    def run_behavior_manager(self):
        """Run the behavior manager node"""
        try:
            # Kill previous process if it exists
            if self.behavior_manager_process:
                self.behavior_manager_process.terminate()
                self.behavior_manager_process.wait()

            # Start new behavior manager process
            self.behavior_manager_process = subprocess.Popen(
                ["ros2", "run", "goat_behavior", "behavior_manager"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.get_logger().info("Started behavior manager node")

        except Exception as e:
            self.get_logger().error(f"Error running behavior manager: {str(e)}")

    def object_detection_callback(self, msg: String):
        """Handle new object detections from Shepherd"""
        try:
            detection = json.loads(msg.data)

            # Debug log the received detection
            self.get_logger().debug(f"Received detection: {detection}")

            # Extract object information
            object_id = detection.get("object_id")
            if not object_id:
                self.get_logger().warn("Received detection without object_id")
                return

            # Get object position from point cloud centroid
            position = detection.get("position", {"x": 0.0, "y": 0.0, "z": 0.0})

            # Get caption and log it
            caption = detection.get("metadata", {}).get("caption", "")
            self.get_logger().info(
                f"Processing object {object_id} with caption: {caption}"
            )

            # Create properties dictionary with relevant information
            properties = {
                "caption": caption,
                "similarity": detection.get("similarity"),
                "confidence": detection.get("detection", {}).get("confidence"),
                "class": detection.get("detection", {}).get("class"),
                "timestamp": detection.get("timestamp"),
            }

            # Update the world object in GoatState
            self.controller.update_world_object(
                obj_id=object_id,
                obj_type="detected_object",
                position=position,
                properties=properties,
            )

            self.get_logger().info(f"Updated object {object_id} in GoatState")

        except Exception as e:
            self.get_logger().error(f"Error processing object detection: {str(e)}")

    def user_command_callback(self, msg: String):
        """Handle new user commands"""
        try:
            command = msg.data

            # Use a default conversation ID or manage multiple conversations as needed
            conversation_id = "default_conversation"
            if not any(
                c["id"] == conversation_id for c in self.controller.conversations
            ):
                self.controller.create_conversation()

            # Process the message through GoatController
            response = self.controller.process_message(conversation_id, command)

            # Optionally, you can publish the response or log it
            self.get_logger().info(f"Processed command: {command}")
            self.get_logger().info(f"Response: {response.get('response')}")

        except Exception as e:
            self.get_logger().error(f"Error processing user command: {str(e)}")

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
            self.get_logger().error(f"Error publishing state: {str(e)}")

    def check_for_plan_updates(self):
        """Check if the behavior tree has been updated in GoatState"""
        try:
            # Get the current behavior tree and last_updated timestamp
            behavior_tree_data = self.goat_state.get_behavior_tree()
            last_updated = (
                self.goat_state.behavior_tree_last_updated
            )  # Ensure this attribute exists

            # If we don't have a last known update time, set it
            if self.last_behavior_tree_update_time is None:
                self.last_behavior_tree_update_time = last_updated
                return

            # Compare the timestamps
            if last_updated != self.last_behavior_tree_update_time:
                self.last_behavior_tree_update_time = last_updated
                # The behavior tree has been updated
                self.get_logger().info(
                    "Behavior tree updated, triggering on_plan_update callback"
                )
                self.on_plan_update(behavior_tree_data)

        except Exception as e:
            self.get_logger().error(f"Error checking for plan updates: {str(e)}")

    def destroy_node(self):
        """Clean up when node is destroyed"""
        # Kill behavior manager process if it exists
        if self.behavior_manager_process:
            self.behavior_manager_process.terminate()
            self.behavior_manager_process.wait()

        super().destroy_node()


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


if __name__ == "__main__":
    main()
