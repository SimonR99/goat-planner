#!/usr/bin/env python3
from base_action_server import BaseActionServer
from goat_behavior_tree.srv import Wait
import time
import rclpy

class WaitServer(BaseActionServer):
    def __init__(self):
        super().__init__('wait_server')
        self.srv = self.create_service(Wait, 'wait', self.wait_callback)

    def wait_callback(self, request, response):
        self.print_start_action('Wait', duration=request.duration)
        time.sleep(request.duration)
        self.print_end_action('Wait')
        response.success = True
        response.message = f"Waited for {request.duration} seconds"
        return response

def main():
    rclpy.init()
    server = WaitServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 