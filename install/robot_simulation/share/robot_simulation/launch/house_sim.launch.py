import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths for packages
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_robot_description = FindPackageShare(package='robot_description').find('robot_description')
    pkg_robot_simulation = FindPackageShare(package='robot_simulation').find('robot_simulation')

    # Default file paths for various resources
    default_model_path = os.path.join(pkg_robot_description, 'urdf/mobile_robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_robot_simulation, 'config/nav2_config.rviz')
    world_file_name = 'house.world'
    world_path = os.path.join(pkg_robot_simulation, 'worlds', world_file_name)

    # Declare Launch Configuration variables (allows dynamic adjustment when launching)
    use_simulator = LaunchConfiguration('use_simulator')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')  # Add use_rviz configuration

    # Declare the launch arguments (to make them configurable when running the launch file)
    declare_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator', default_value='True', description='Whether to start the simulator')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_path, description='RVIZ config file path')
    declare_world_cmd = DeclareLaunchArgument(
        name='world', default_value=world_path, description='Path to the world file')
    declare_use_rviz_cmd = DeclareLaunchArgument(  # Declare use_rviz argument
        name='use_rviz',
        default_value='true',
        description='Whether to launch RViz')

    # Start Gazebo server and client (only if simulator is enabled)
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())
    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(use_simulator))

    # Spawn the robot in Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot']
    )

    # Start robot localization (EKF)
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{'use_sim_time': 'True'}]
    )

    # Start the robot state publisher to publish joint states and robot transforms
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    # Launch RViz (if enabled)
    start_rviz_cmd = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    # Assemble the launch description
    ld = LaunchDescription()

    # Declare all the launch arguments
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_rviz_cmd)  # Add the use_rviz declaration

    # Add the actions to launch the simulator and other nodes
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
