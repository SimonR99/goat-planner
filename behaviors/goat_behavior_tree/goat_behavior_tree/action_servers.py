#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from goat_behavior_tree.srv import NavigateTo, Pick, Place, LocateObject, RequestAssistance, Wait, ExecuteCommand
from geometry_msgs.msg import Pose
import subprocess
import time

class RobotActionServers(Node):
    def __init__(self):
        super().__init__('robot_action_servers')
        
        # Create all services
        self.navigate_srv = self.create_service(
            NavigateTo, 'navigate_to', self.navigate_to_callback)
        
        self.pick_srv = self.create_service(
            Pick, 'pick', self.pick_callback)
        
        self.place_srv = self.create_service(
            Place, 'place', self.place_callback)
        
        self.locate_srv = self.create_service(
            LocateObject, 'locate_object', self.locate_object_callback)
        
        self.assist_srv = self.create_service(
            RequestAssistance, 'request_assistance', self.request_assistance_callback)
        
        self.wait_srv = self.create_service(
            Wait, 'wait', self.wait_callback)
        
        self.execute_srv = self.create_service(
            ExecuteCommand, 'execute_command', self.execute_command_callback)
        
        self.get_logger().info('Action servers are ready')

    def navigate_to_callback(self, request, response):
        self.get_logger().info(f'Navigating to: {request.target}')
        # Implement your navigation logic here
        response.success = True
        response.message = f"Successfully navigated to {request.target}"
        return response

    def pick_callback(self, request, response):
        self.get_logger().info(f'Picking object: {request.object}')
        # Implement your pick logic here
        response.success = True
        response.message = f"Successfully picked {request.object}"
        return response

    def place_callback(self, request, response):
        self.get_logger().info(f'Placing {request.object} at {request.location}')
        # Implement your place logic here
        response.success = True
        response.message = f"Successfully placed {request.object} at {request.location}"
        return response

    def locate_object_callback(self, request, response):
        self.get_logger().info(f'Locating object: {request.object}')
        # Implement your object location logic here
        response.success = True
        response.message = f"Found {request.object}"
        response.pose = Pose()  # Fill with actual pose data
        return response

    def request_assistance_callback(self, request, response):
        self.get_logger().info(f'Requesting assistance for: {request.task}')
        # Implement your assistance request logic here
        response.success = True
        response.message = f"Assistance requested for {request.task}"
        return response

    def wait_callback(self, request, response):
        self.get_logger().info(f'Waiting for {request.duration} seconds')
        time.sleep(request.duration)
        response.success = True
        response.message = f"Waited for {request.duration} seconds"
        return response

    def execute_command_callback(self, request, response):
        self.get_logger().info(f'Executing command: {request.command}')
        try:
            result = subprocess.run(request.command, shell=True, 
                                  capture_output=True, text=True)
            response.success = result.returncode == 0
            response.output = result.stdout
            response.message = "Command executed successfully" if response.success else "Command failed"
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.output = ""
        return response

def main():
    rclpy.init()
    action_servers = RobotActionServers()
    
    try:
        rclpy.spin(action_servers)
    except KeyboardInterrupt:
        pass
    finally:
        action_servers.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 