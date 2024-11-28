#!/usr/bin/env python3
import rclpy
from base_action_server import BaseActionServer
from goat_behavior_tree.srv import Pick
import time

class PickServer(BaseActionServer):
    def __init__(self):
        super().__init__('pick_server')
        self.srv = self.create_service(Pick, 'pick', self.pick_callback)

    def pick_callback(self, request, response):
        self.print_start_action('Pick', object=request.object)
        self.get_logger().info('Extending arm...')
        time.sleep(1)
        self.get_logger().info('Grasping object...')
        time.sleep(1)
        self.print_end_action('Pick')
        response.success = True
        response.message = f"Successfully picked {request.object}"
        return response

def main():
    rclpy.init()
    server = PickServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 