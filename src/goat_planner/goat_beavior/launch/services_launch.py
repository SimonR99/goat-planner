from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Pick service
            Node(
                package="goat_behavior",
                executable="pick_service.py",
                name="pick_service",
                output="screen",
            ),
            # Place service
            Node(
                package="goat_behavior",
                executable="place_service.py",
                name="place_service",
                output="screen",
            ),
            # Navigate service
            Node(
                package="goat_behavior",
                executable="navigate_service.py",
                name="navigate_service",
                output="screen",
            ),
            Node(
                package="goat_behavior",
                executable="goat_state_bridge.py",
                name="goat_state_bridge",
                output="screen",
            ),
            Node(
                package="goat_behavior",
                executable="shepherd_connector.py",
                name="shepherd_connector",
                output="screen",
            ),
        ]
    )
