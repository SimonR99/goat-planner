#!/usr/bin/env python3
import rclpy
from base_action_server import BaseActionServer
from goat_behavior_tree.srv import NavigateTo
import time

class NavigateServer(BaseActionServer):
    def __init__(self):
        super().__init__('navigate_server')
        self.srv = self.create_service(NavigateTo, 'navigate_to', self.navigate_callback)

    def navigate_callback(self, request, response):
        self.print_start_action('Navigation', target=request.target)
        self.get_logger().info('Moving...')
        time.sleep(2)
        self.print_end_action('Navigation')
        response.success = True
        response.message = f"Successfully navigated to {request.target}"
        return response

def main():
    rclpy.init()
    server = NavigateServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 