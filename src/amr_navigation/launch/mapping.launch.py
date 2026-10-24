"""Attach SLAM to one existing simulation. Never starts AMCL or Nav2."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from amr_navigation.launch_checks import exclusive_mode
from launch.launch_description_sources import PythonLaunchDescriptionSource

def start(context):
    exclusive_mode()
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(share('slam_toolbox'))/'launch/online_async_launch.py')),
        launch_arguments={'use_sim_time':'true','slam_params_file':
            LaunchConfiguration('slam_params_file').perform(context)}.items())]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('slam_params_file', default_value=str(Path(share('amr_navigation'))/'config/slam_params.yaml')), OpaqueFunction(function=start)])
