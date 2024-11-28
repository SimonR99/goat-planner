#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from goat_behavior_tree.srv import NavigateTo
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class NavigateServer(Node):
    def __init__(self):
        super().__init__('navigate_server')
        # Use ReentrantCallbackGroup to allow concurrent callbacks
        callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            NavigateTo, 
            'navigate_to', 
            self.navigate_callback,
            callback_group=callback_group
        )
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            '/navigate_to_pose',
            callback_group=callback_group
        )
        self.get_logger().info('Navigate server is ready')
        self._current_response = None
        self._navigation_complete = False

    def send_goal(self, x, y, frame_id='map', w=1.0):
        """Sends a navigation goal to the NavigateToPose action server."""
        self.get_logger().info(f'Sending goal to navigate to (x={x}, y={y}) in frame "{frame_id}"')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set target position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (no rotation in this case)
        goal_msg.pose.pose.orientation.w = w

        self._action_client.wait_for_server()
        self.get_logger().info('Action server available. Sending goal...')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Wait for navigation to complete, with a timeout
        timeout = 60  # seconds
        rate = self.create_rate(2)  # 2Hz checking rate
        elapsed = 0
        
        while not self._navigation_complete and rclpy.ok() and elapsed < timeout:
            rate.sleep()
            elapsed += 0.5  # 0.5 seconds per iteration
            
        if elapsed >= timeout:
            self.get_logger().warn('Navigation timed out after 60 seconds')
            if self._current_response:
                self._current_response.success = False
                self._current_response.message = "Navigation timed out"
            return False
            
        return self._navigation_complete

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by the action server.')
            if self._current_response:
                self._current_response.success = False
                self._current_response.message = "Goal rejected by Nav2"
            self._navigation_complete = True
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Pose: x={feedback.current_pose.pose.position.x}, '
                              f'y={feedback.current_pose.pose.position.y}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation completed successfully.')
        if self._current_response:
            self._current_response.success = True
            self._current_response.message = "Navigation completed successfully"
        self._navigation_complete = True

    def navigate_callback(self, request, response):
        self.get_logger().info('='*50)
        if request.target:
            self.get_logger().info(f'🤖 Navigating to location: {request.target}')
        self.get_logger().info(f'🤖 Target coordinates: x={request.x}, y={request.y}')
        
        self._current_response = response
        self._navigation_complete = False
        
        success = self.send_goal(request.x, request.y)
        self.get_logger().info(f'🤖 Navigation (part 2) complete: {success}')
        
        return self._current_response

def main():
    rclpy.init()
    server = NavigateServer()
    
    # Use MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Just destroy the node, don't call shutdown
        executor.shutdown()
        server.destroy_node()

if __name__ == '__main__':
    main()