#!/usr/bin/env python3
import threading  # Import threading
import time

import rclpy
from goat_behavior.srv import Navigate
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class NavigateService(Node):
    def __init__(self):
        super().__init__("navigate_service")

        # Use ReentrantCallbackGroup to allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            Navigate,
            "navigate_to_pose",
            self.navigate_callback,
            callback_group=self.callback_group,
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.callback_group
        )

        self.get_logger().info("Navigation service started")

    def navigate_callback(self, request, response):
        self.get_logger().info(
            f"Received navigation request to position: "
            f"({request.x}, {request.y}, {request.z}), "
            f"orientation: ({request.qx}, {request.qy}, {request.qz}, {request.qw})"
        )

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            response.success = False
            return response

        # Create and send goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
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

        # Initialize an Event to wait for the action to complete
        self._action_done_event = threading.Event()
        self._response = response  # Store the response object to modify later

        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait for the action to complete
        self._action_done_event.wait()

        # Return the response after the action is done
        return self._response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self._response.success = False
            # Signal that the action is done
            self._action_done_event.set()
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self._response.success = (
            result.status == 4
        )  # 4 is SUCCESS in action result status

        if self._response.success:
            self.get_logger().info("Navigation succeeded")
        else:
            self.get_logger().error("Navigation failed")

        # Signal that the action is done
        self._action_done_event.set()


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = NavigateService()

    # Use MultiThreadedExecutor to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # Keep the main thread alive
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()
