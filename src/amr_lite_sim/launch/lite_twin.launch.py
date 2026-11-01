"""Lite engine + robot_state_publisher + Gateway (world-state v1). No Gazebo, no Nav2."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lite = Path(get_package_share_directory('amr_lite_sim'))/'launch/lite.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='operations', choices=['operations', 'mapping', 'empty']),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lite)),
                                 launch_arguments={'profile': LaunchConfiguration('profile'),
                                                   'real_time_factor': LaunchConfiguration('real_time_factor')}.items()),
        # The engine keeps running if the Gateway exits; the Gateway only distributes state.
        Node(package='digital_twin_gateway', executable='gateway', output='screen',
             additional_env={'TWIN_SCENARIO_SOURCE': 'unspecified'}),
    ])
