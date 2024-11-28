#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from goat_behavior_tree.srv import NavigateTo
import sys

class NavigateTestClient(Node):
    def __init__(self):
        super().__init__('navigate_test_client')
        self.client = self.create_client(NavigateTo, 'navigate_to')
        
        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Navigation service not available, waiting...')

    def send_request(self, x, y, target_name=""):
        request = NavigateTo.Request()
        request.x = float(x)
        request.y = float(y)
        request.target = target_name
        
        self.get_logger().info(f'Sending navigation request to (x={x}, y={y})')
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = NavigateTestClient()
    
    # Get coordinates from command line arguments or use defaults
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    
    future = client.send_request(x, y)
    
    while rclpy.ok():
        rclpy.spin_once(client)
        if future.done():
            try:
                response = future.result()
                client.get_logger().info(f'Result: success={response.success}, message="{response.message}"')
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
            break
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 