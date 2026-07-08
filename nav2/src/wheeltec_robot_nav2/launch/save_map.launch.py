import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    default_map_path = os.path.join(wheeltec_nav_dir, 'map', 'WHEELTEC')
    map_path = LaunchConfiguration('map_path')

    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Map output path without extension')

    # Example:
    # ros2 launch wheeltec_nav2 save_map.launch.py map_path:=/path/to/map/WHEELTEC
    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', map_path],
        
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    ld = LaunchDescription()

    ld.add_action(declare_map_path)
    ld.add_action(map_saver)
    return ld
