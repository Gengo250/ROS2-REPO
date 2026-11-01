"""Lite engine (no agents) + SLAM Toolbox, optionally driven by the coverage survey.

Never starts AMCL, Nav2 or Gazebo. Save the map while SLAM runs with
``amr_navigation/scripts/save_map.sh /absolute/path/name``.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    lite = Path(get_package_share_directory('amr_lite_sim'))
    navigation = Path(get_package_share_directory('amr_navigation'))
    value = lambda key: LaunchConfiguration(key).perform(context)
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lite/'launch/lite.launch.py')),
                                 launch_arguments={'profile': 'mapping', 'real_time_factor': value('real_time_factor'),
                                                   'lidar_noise_stddev': value('lidar_noise_stddev')}.items()),
        # SLAM reads /top_scan_mapping: no-return beams clear free space (see free_space_relay.py).
        Node(package='amr_lite_sim', executable='free_space_relay', output='screen', parameters=[{'use_sim_time': True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation/'launch/mapping.launch.py')),
                                 launch_arguments={'slam_params_file': value('slam_params_file')}.items()),
    ]
    if value('survey') == 'true':
        actions.append(Node(package='amr_lite_sim', executable='lite_survey', output='screen',
                            parameters=[{'plan': value('plan'), 'report': value('report')}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('amr_lite_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        # Exact ranges land on the faces of SLAM's grid bounds and the walls on the two maximum
        # faces are dropped; scanner-like noise keeps them in the map (Phase 6, map v9).
        DeclareLaunchArgument('lidar_noise_stddev', default_value='0.01'),
        DeclareLaunchArgument('survey', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('plan', default_value=str(share/'config/campus_survey.json')),
        DeclareLaunchArgument('report', default_value=''),
        DeclareLaunchArgument('slam_params_file', default_value=str(share/'config/slam_params_campus.yaml')),
        OpaqueFunction(function=setup),
    ])
