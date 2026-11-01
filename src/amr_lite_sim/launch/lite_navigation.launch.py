"""Lite engine + Gateway + saved-map AMCL + Nav2 + command guard. Starts no Gazebo process.

The campus map's origin is the spawn pose (SLAM starts there), so the robot is initialised at
map (0, 0, 0); campus_registration.json relates that map to the world frame for the Gateway.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lite = Path(get_package_share_directory('amr_lite_sim'))/'launch'
    navigation = Path(get_package_share_directory('amr_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='operations', choices=['operations', 'mapping', 'empty']),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('map', default_value=str(navigation/'maps/campus.yaml')),
        # The campus map was built from the 360-degree scanner; AMCL must use the same sensor.
        DeclareLaunchArgument('amcl_scan_topic', default_value='/top_scan'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lite/'lite_twin.launch.py')),
                                 launch_arguments={'profile': LaunchConfiguration('profile'),
                                                   'real_time_factor': LaunchConfiguration('real_time_factor')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation/'launch/navigation.launch.py')),
                                 launch_arguments={'map': LaunchConfiguration('map'), 'amcl_scan_topic': LaunchConfiguration('amcl_scan_topic'),
                                                   'initialize': 'true', 'initial_x': '0', 'initial_y': '0', 'initial_yaw': '0',
                                                   'simulation_backend': 'lite', 'rviz': LaunchConfiguration('rviz')}.items()),
    ])
