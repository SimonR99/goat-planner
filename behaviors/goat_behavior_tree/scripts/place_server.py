#!/usr/bin/env python3
import rclpy
from base_action_server import BaseActionServer
from goat_behavior_tree.srv import Place
import time

class PlaceServer(BaseActionServer):
    def __init__(self):
        super().__init__('place_server')
        self.srv = self.create_service(Place, 'place', self.place_callback)

    def place_callback(self, request, response):
        self.print_start_action('Place', object=request.object, location=request.location)
        self.get_logger().info('Extending arm...')
        time.sleep(1)
        self.get_logger().info('Releasing object...')
        time.sleep(1)
        self.print_end_action('Place')
        response.success = True
        response.message = f"Successfully placed {request.object} at {request.location}"
        return response

def main():
    rclpy.init()
    server = PlaceServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 