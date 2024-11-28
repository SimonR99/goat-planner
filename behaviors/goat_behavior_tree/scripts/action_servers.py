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
        self.get_logger().info('='*50)
        self.get_logger().info(f'🤖 Robot is navigating to: {request.target}')
        self.get_logger().info('Moving...')
        time.sleep(2)  # Simulate movement
        self.get_logger().info(f'✅ Arrived at {request.target}!')
        self.get_logger().info('='*50)
        response.success = True
        response.message = f"Successfully navigated to {request.target}"
        return response

    def pick_callback(self, request, response):
        self.get_logger().info('='*50)
        self.get_logger().info(f'🤖 Robot is picking up: {request.object}')
        self.get_logger().info('Extending arm...')
        time.sleep(1)
        self.get_logger().info('Grasping object...')
        time.sleep(1)
        self.get_logger().info(f'✅ Successfully picked up {request.object}!')
        self.get_logger().info('='*50)
        response.success = True
        response.message = f"Successfully picked {request.object}"
        return response

    def place_callback(self, request, response):
        self.get_logger().info('='*50)
        self.get_logger().info(f'🤖 Robot is placing {request.object} at {request.location}')
        self.get_logger().info('Extending arm...')
        time.sleep(1)
        self.get_logger().info('Releasing object...')
        time.sleep(1)
        self.get_logger().info(f'✅ Successfully placed {request.object} at {request.location}!')
        self.get_logger().info('='*50)
        response.success = True
        response.message = f"Successfully placed {request.object} at {request.location}"
        return response

    def locate_object_callback(self, request, response):
        self.get_logger().info('='*50)
        self.get_logger().info(f'🤖 Robot is searching for: {request.object}')
        self.get_logger().info('Scanning environment...')
        time.sleep(2)
        self.get_logger().info(f'✅ Found {request.object}!')
        self.get_logger().info('='*50)
        response.success = True
        response.message = f"Found {request.object}"
        # Set a dummy pose
        response.pose = Pose()
        response.pose.position.x = 1.0
        response.pose.position.y = 1.0
        response.pose.position.z = 0.0
        return response

    def request_assistance_callback(self, request, response):
        self.get_logger().info('='*50)
        self.get_logger().info(f'🤖 Robot needs help with: {request.task}')
        self.get_logger().info('Sending notification...')
        time.sleep(1)
        self.get_logger().info('✅ Assistance request sent!')
        self.get_logger().info('='*50)
        response.success = True
        response.message = f"Assistance requested for {request.task}"
        return response

    def wait_callback(self, request, response):
        self.get_logger().info('='*50)
        self.get_logger().info(f'🤖 Robot is waiting for {request.duration} seconds')
        time.sleep(request.duration)
        self.get_logger().info('✅ Wait completed!')
        self.get_logger().info('='*50)
        response.success = True
        response.message = f"Waited for {request.duration} seconds"
        return response

    def execute_command_callback(self, request, response):
        self.get_logger().info('='*50)
        self.get_logger().info(f'🤖 Robot is executing command: {request.command}')
        try:
            result = subprocess.run(request.command, shell=True, 
                                  capture_output=True, text=True)
            response.success = result.returncode == 0
            response.output = result.stdout
            self.get_logger().info(f'Command output: {result.stdout}')
            self.get_logger().info('✅ Command executed successfully!')
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.output = ""
            self.get_logger().error(f'❌ Command failed: {str(e)}')
        self.get_logger().info('='*50)
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