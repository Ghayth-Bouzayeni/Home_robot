from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # SLAM node
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Shutdown after 10 seconds
    shutdown_timer = TimerAction(
        period=10.0,
        actions=[Shutdown()]
    )

    return LaunchDescription([
        slam_node,
        shutdown_timer
    ])
