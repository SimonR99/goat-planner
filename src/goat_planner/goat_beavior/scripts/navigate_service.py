#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from goat_behavior.srv import Navigate
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class NavigateService(Node):
    def __init__(self):
        super().__init__('navigate_service')
        
        # Use ReentrantCallbackGroup to allow concurrent callbacks
        callback_group = ReentrantCallbackGroup()
        
        self.srv = self.create_service(
            Navigate, 
            'navigate_to_pose', 
            self.navigate_callback,
            callback_group=callback_group
        )
        
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=callback_group
        )
        
        self.get_logger().info('Navigation service started')

    def navigate_callback(self, request, response):
        self.get_logger().info(f'Received navigation request to position: '
                              f'({request.x}, {request.y}, {request.z}), '
                              f'orientation: ({request.qx}, {request.qy}, {request.qz}, {request.qw})')

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            response.success = False
            return response

        # Create and send goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal.pose.pose.position.x = float(request.x)
        goal.pose.pose.position.y = float(request.y)
        goal.pose.pose.position.z = float(request.z)
        
        # Set orientation
        goal.pose.pose.orientation.x = float(request.qx)
        goal.pose.pose.orientation.y = float(request.qy)
        goal.pose.pose.orientation.z = float(request.qz)
        goal.pose.pose.orientation.w = float(request.qw)

        future = self.nav_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            response.success = False
            return response

        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for action completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        response.success = (result.status == 4)  # 4 is SUCCESS in action result status
        
        if response.success:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().error('Navigation failed')
            
        return response

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = NavigateService()
    
    # Use MultiThreadedExecutor to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 