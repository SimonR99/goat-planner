#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from goat_behavior.srv import Place
import time


class PlaceService(Node):
    def __init__(self):
        super().__init__("place_service")
        self.srv = self.create_service(Place, "place_object", self.place_callback)

    def place_callback(self, request, response):
        self.get_logger().info(
            f"Executing place action for object: {request.object_name} at location: {request.location}"
        )
        # Simulate some work (e.g., wait for 2 seconds)
        time.sleep(2)
        response.success = True
        self.get_logger().info(f"Place action completed successfully")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlaceService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
