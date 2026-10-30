"""Saved-map AMCL; initialize using an explicit map-frame pose or RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from amr_navigation.launch_checks import exclusive_mode
from amr_navigation.launch_support import share, localization_actions

def start(context):
    exclusive_mode()
    return localization_actions(LaunchConfiguration('map').perform(context),LaunchConfiguration('amcl_scan_topic').perform(context))

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map',default_value=str(share()/'maps/warehouse.yaml')),
                              DeclareLaunchArgument('amcl_scan_topic',default_value=''),
                              OpaqueFunction(function=start)])
