"""Compose the existing warehouse headless stack with read-only telemetry."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    warehouse = Path(get_package_share_directory('amr_simulation')) / 'launch/warehouse.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='normal', choices=[
            'normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(warehouse)),
                                 launch_arguments={'scenario': LaunchConfiguration('scenario'),
                                                   'gui': 'false', 'rviz': 'false'}.items()),
        Node(package='digital_twin_gateway', executable='simulation_pose_adapter', output='screen'),
        Node(package='digital_twin_gateway', executable='gateway', output='screen',
             additional_env={'TWIN_SCENARIO': LaunchConfiguration('scenario'),
                             'TWIN_SCENARIO_SOURCE': 'launch_configuration'},
             on_exit=Shutdown(reason='Digital Twin Gateway exited')),
    ])
