#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from goat_behavior.srv import Pick  # Import the custom service
import time

class PickService(Node):
    def __init__(self):
        super().__init__('pick_service')
        self.srv = self.create_service(Pick, 'pick_object', self.pick_callback)

    def pick_callback(self, request, response):
        self.get_logger().info(f'Executing pick action for object: {request.object_name}')
        # Simulate some work (e.g., wait for 2 seconds)
        time.sleep(2)
        response.success = True
        self.get_logger().info(f'Pick action for object {request.object_name} completed successfully')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PickService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
