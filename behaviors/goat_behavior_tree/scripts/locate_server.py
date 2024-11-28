#!/usr/bin/env python3
from base_action_server import BaseActionServer
from goat_behavior_tree.srv import LocateObject
from geometry_msgs.msg import Pose
import time
import rclpy

class LocateServer(BaseActionServer):
    def __init__(self):
        super().__init__('locate_server')
        self.srv = self.create_service(LocateObject, 'locate_object', self.locate_callback)

    def locate_callback(self, request, response):
        self.print_start_action('Locate', object=request.object)
        self.get_logger().info('Scanning environment...')
        time.sleep(2)
        self.print_end_action('Locate')
        response.success = True
        response.message = f"Found {request.object}"
        # Set a dummy pose
        response.pose = Pose()
        response.pose.position.x = 1.0
        response.pose.position.y = 1.0
        response.pose.position.z = 0.0
        return response

def main():
    rclpy.init()
    server = LocateServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 