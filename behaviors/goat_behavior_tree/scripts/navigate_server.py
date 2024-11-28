#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from goat_behavior_tree.srv import NavigateTo
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
import math

class NavigateServer(Node):
    def __init__(self):
        super().__init__('navigate_server')
        self.srv = self.create_service(NavigateTo, 'navigate_to', self.navigate_callback)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Navigate server is ready')

    def create_pose_stamped(self, x, y):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        return pose_msg

    def navigate_callback(self, request, response):
        self.get_logger().info('='*50)
        
        # Log target information
        if request.target:
            self.get_logger().info(f'🤖 Navigating to location: {request.target}')
        self.get_logger().info(f'🤖 Target coordinates: x={request.x}, y={request.y}')

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            response.success = False
            response.message = "Navigation action server not available"
            return response

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(request.x, request.y)

        # Send goal
        self.get_logger().info('Sending goal to Nav2...')
        
        try:
            goal_handle_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, goal_handle_future)
            goal_handle = goal_handle_future.result()

            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by Nav2!')
                response.success = False
                response.message = "Navigation goal rejected"
                return response

            self.get_logger().info('Goal accepted by Nav2, waiting for result...')

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result.status == result.status.SUCCEEDED:
                self.get_logger().info('✅ Navigation successful!')
                response.success = True
                response.message = f"Successfully navigated to x={request.x}, y={request.y}"
            else:
                self.get_logger().error(f'❌ Navigation failed with status: {result.status}')
                response.success = False
                response.message = f"Navigation failed with status: {result.status}"

        except Exception as e:
            self.get_logger().error(f'Navigation failed with error: {str(e)}')
            response.success = False
            response.message = str(e)

        self.get_logger().info('='*50)
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