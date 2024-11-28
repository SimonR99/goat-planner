#!/usr/bin/env python3
from base_action_server import BaseActionServer
from goat_behavior_tree.srv import ExecuteCommand
import subprocess
import time
import rclpy

class ExecuteServer(BaseActionServer):
    def __init__(self):
        super().__init__('execute_server')
        self.srv = self.create_service(ExecuteCommand, 'execute_command', self.execute_callback)

    def execute_callback(self, request, response):
        self.print_start_action('Execute Command', command=request.command)
        try:
            result = subprocess.run(request.command, shell=True, 
                                  capture_output=True, text=True)
            response.success = result.returncode == 0
            response.output = result.stdout
            self.get_logger().info(f'Command output: {result.stdout}')
            self.print_end_action('Execute Command', success=response.success)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.output = ""
            self.get_logger().error(f'Command failed: {str(e)}')
            self.print_end_action('Execute Command', success=False)
        return response

def main():
    rclpy.init()
    server = ExecuteServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 