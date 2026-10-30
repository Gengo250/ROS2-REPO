"""One existing headless Digital Twin simulation plus saved-map navigation."""
import os
import uuid
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from amr_navigation.launch_support import share

def generate_launch_description():
    args=[DeclareLaunchArgument('scenario',default_value='normal',choices=['normal','person_crossing','obstacle_in_aisle','corridor_blocked']),
          DeclareLaunchArgument('map',default_value=str(share()/'maps/warehouse.yaml')),
          DeclareLaunchArgument('initialize',default_value='false',choices=['true','false']),
          DeclareLaunchArgument('initial_x',default_value='0'),DeclareLaunchArgument('initial_y',default_value='0'),
          DeclareLaunchArgument('initial_yaw',default_value='0'),DeclareLaunchArgument('rviz',default_value='false'),
          DeclareLaunchArgument('amcl_scan_topic',default_value='')]
    return LaunchDescription(args+[
        SetEnvironmentVariable('GZ_PARTITION',os.environ.get('GZ_PARTITION') or 'amr_navigation_'+uuid.uuid4().hex),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('digital_twin_gateway'))/'launch/digital_twin.launch.py')),
                                 launch_arguments={'scenario':LaunchConfiguration('scenario')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share()/'launch/navigation.launch.py')),
                                 launch_arguments={key:LaunchConfiguration(key) for key in ['map','initialize','initial_x','initial_y','initial_yaw','rviz','amcl_scan_topic']}.items())])
