from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Create all action server nodes
    navigate_server = Node(
        package='goat_behavior_tree',
        executable='navigate_server.py',
        name='navigate_server',
        output='screen'
    )

    pick_server = Node(
        package='goat_behavior_tree',
        executable='pick_server.py',
        name='pick_server',
        output='screen'
    )

    place_server = Node(
        package='goat_behavior_tree',
        executable='place_server.py',
        name='place_server',
        output='screen'
    )

    locate_server = Node(
        package='goat_behavior_tree',
        executable='locate_server.py',
        name='locate_server',
        output='screen'
    )

    assist_server = Node(
        package='goat_behavior_tree',
        executable='assist_server.py',
        name='assist_server',
        output='screen'
    )

    wait_server = Node(
        package='goat_behavior_tree',
        executable='wait_server.py',
        name='wait_server',
        output='screen'
    )

    execute_server = Node(
        package='goat_behavior_tree',
        executable='execute_server.py',
        name='execute_server',
        output='screen'
    )

    # Create the behavior tree node with a delay
    bt_node = Node(
        package='goat_behavior_tree',
        executable='goat_bt_main',
        name='goat_bt_main',
        output='screen'
    )

    # Wrap the BT node in a timer to delay its start
    delayed_bt_node = TimerAction(
        period=4.0,  # 2 second delay
        actions=[bt_node]
    )

    # Return the launch description
    return LaunchDescription([
        navigate_server,
        pick_server,
        place_server,
        locate_server,
        assist_server,
        wait_server,
        execute_server,
        delayed_bt_node
    ]) 