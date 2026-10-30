"""Saved map + AMCL + minimal Nav2. Attach to the one existing simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from amr_navigation.launch_checks import exclusive_mode
from amr_navigation.launch_support import share, localization_actions

def start(context):
    exclusive_mode()
    config=str(share()/'config/nav2_params.yaml')
    nodes=localization_actions(LaunchConfiguration('map').perform(context),LaunchConfiguration('amcl_scan_topic').perform(context))
    for package,executable in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),
                              ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
        params=[config]
        if executable=='bt_navigator':params.append({'default_nav_to_pose_bt_xml':str(share()/'config/navigate_to_pose.xml'),
                                                     'default_nav_through_poses_bt_xml':str(share()/'config/navigate_through_poses.xml')})
        nodes.append(Node(package=package,executable=executable,name=executable,output='screen',parameters=params,
                          remappings=[('cmd_vel','nav_cmd_vel')] if executable in ['controller_server','behavior_server'] else []))
    nodes += [Node(package='amr_navigation',executable='command_guard',output='screen',parameters=[{'use_sim_time':True}],on_exit=Shutdown(reason='Navigation command interlock exited')),
              Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',output='screen',
                   parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':4.0,
                                'node_names':['controller_server','planner_server','behavior_server','bt_navigator']}])]
    if LaunchConfiguration('initialize').perform(context)=='true':
        nodes.append(Node(package='amr_navigation',executable='initial_pose',output='screen',arguments=[
            LaunchConfiguration(key).perform(context) for key in ['initial_x','initial_y','initial_yaw']]))
    if LaunchConfiguration('rviz').perform(context)=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share()/'rviz/navigation.rviz')],parameters=[{'use_sim_time':True}]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map',default_value=str(share()/'maps/warehouse.yaml')),
        DeclareLaunchArgument('amcl_scan_topic',default_value='',description='Override AMCL scan topic (must match the map sensor)'),
        DeclareLaunchArgument('initialize',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('initial_x',default_value='0'),DeclareLaunchArgument('initial_y',default_value='0'),
        DeclareLaunchArgument('initial_yaw',default_value='0'),DeclareLaunchArgument('rviz',default_value='false'),
        OpaqueFunction(function=start)])
