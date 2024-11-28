#!/usr/bin/env python3
from base_action_server import BaseActionServer
from goat_behavior_tree.srv import RequestAssistance
import time
import rclpy

class AssistServer(BaseActionServer):
    def __init__(self):
        super().__init__('assist_server')
        self.srv = self.create_service(RequestAssistance, 'request_assistance', self.assist_callback)

    def assist_callback(self, request, response):
        self.print_start_action('Request Assistance', task=request.task)
        self.get_logger().info('Sending notification...')
        time.sleep(1)
        self.print_end_action('Request Assistance')
        response.success = True
        response.message = f"Assistance requested for {request.task}"
        return response

def main():
    rclpy.init()
    server = AssistServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 