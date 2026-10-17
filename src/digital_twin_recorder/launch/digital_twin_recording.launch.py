"""Opt-in composition; original launch and its shutdown behavior remain intact."""
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gateway=Path(get_package_share_directory('digital_twin_gateway'))/'launch/digital_twin.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value='normal',choices=['normal','person_crossing','obstacle_in_aisle','corridor_blocked']),
        DeclareLaunchArgument('recorder_python',default_value=sys.executable,description='Python from the isolated recorder venv'),
        DeclareLaunchArgument('recording_output',default_value='recordings'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gateway)),launch_arguments={'scenario':LaunchConfiguration('scenario')}.items()),
        ExecuteProcess(cmd=[LaunchConfiguration('recorder_python'),'-m','digital_twin_recorder.ros.recorder_node',
            '--scenario',LaunchConfiguration('scenario'),'--output',LaunchConfiguration('recording_output')],
            output='screen',sigterm_timeout='40',sigkill_timeout='5'),
    ])
