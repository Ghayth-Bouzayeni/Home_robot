import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   
    pkg_robot_simulation = FindPackageShare(package='robot_simulation').find('robot_simulation')

    declare_map_file_path_cmd = DeclareLaunchArgument(
        name='map_file_path',
        default_value='./src/robot_simulation/maps/house_map.yaml',
        description='Path to the map file'
    )

    map_file_path = os.path.join(pkg_robot_simulation, 'maps', 'house_map.yaml')
    log_map_path_cmd = LogInfo(
        condition=None,
        msg=f"Launching with map: {map_file_path}"
    )
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(log_map_path_cmd)

    return ld
