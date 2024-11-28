#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time

class BaseActionServer(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} is ready')

    def print_start_action(self, action_name, **kwargs):
        self.get_logger().info('='*50)
        message = f'🤖 {action_name}'
        if kwargs:
            message += ': ' + ', '.join(f'{k}={v}' for k, v in kwargs.items())
        self.get_logger().info(message)

    def print_end_action(self, action_name, success=True):
        status = '✅' if success else '❌'
        self.get_logger().info(f'{status} {action_name} completed!')
        self.get_logger().info('='*50) 