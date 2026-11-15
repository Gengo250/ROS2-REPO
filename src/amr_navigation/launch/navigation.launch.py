"""Saved map + AMCL + minimal Nav2. Attach to the one existing simulation.

``namespace``/``frame_prefix`` start the stack for one robot of a multi-robot lite twin (e.g.
namespace:=amr_02 frame_prefix:=amr_02/); empty (default) is the single-robot stack, unchanged.
``fleet_obstacles:=true`` adds the fleet tracking costmap layer (other AMRs, amr_lite_sim fleet_tracker).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from amr_navigation.launch_checks import exclusive_mode
from amr_navigation.launch_support import share, localization_actions, robot_params

def start(context):
    namespace=LaunchConfiguration('namespace').perform(context).strip('/');prefix=LaunchConfiguration('frame_prefix').perform(context)
    exclusive_mode(namespace)
    config=robot_params(share()/'config/nav2_params.yaml',namespace,prefix,fleet=LaunchConfiguration('fleet_obstacles').perform(context)=='true')
    nodes=localization_actions(LaunchConfiguration('map').perform(context),LaunchConfiguration('amcl_scan_topic').perform(context),namespace,prefix)
    for package,executable in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),
                              ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
        params=[config]
        if executable=='bt_navigator':params.append({'default_nav_to_pose_bt_xml':str(share()/'config/navigate_to_pose.xml'),
                                                     'default_nav_through_poses_bt_xml':str(share()/'config/navigate_through_poses.xml')})
        nodes.append(Node(package=package,executable=executable,name=executable,output='screen',parameters=params,namespace=namespace or None,
                          remappings=[('cmd_vel','nav_cmd_vel')] if executable in ['controller_server','behavior_server'] else []))
    nodes += [Node(package='amr_navigation',executable='command_guard',output='screen',namespace=namespace or None,parameters=[{'use_sim_time':True,
                   'simulation_backend':LaunchConfiguration('simulation_backend').perform(context),'frame_prefix':prefix}],on_exit=Shutdown(reason='Navigation command interlock exited')),
              Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',output='screen',namespace=namespace or None,
                   parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':4.0,
                                'node_names':['controller_server','planner_server','behavior_server','bt_navigator']}])]
    if LaunchConfiguration('initialize').perform(context)=='true':
        nodes.append(Node(package='amr_navigation',executable='initial_pose',output='screen',namespace=namespace or None,arguments=[
            LaunchConfiguration(key).perform(context) for key in ['initial_x','initial_y','initial_yaw']]+['--frame-prefix',prefix]))
    if LaunchConfiguration('rviz').perform(context)=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share()/'rviz/navigation.rviz')],parameters=[{'use_sim_time':True}]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map',default_value=str(share()/'maps/warehouse.yaml')),
        DeclareLaunchArgument('amcl_scan_topic',default_value='',description='Override AMCL scan topic (must match the map sensor)'),
        DeclareLaunchArgument('simulation_backend',default_value='gazebo',choices=['gazebo','lite'],description='Emergency pause target of the command guard'),
        DeclareLaunchArgument('initialize',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('initial_x',default_value='0'),DeclareLaunchArgument('initial_y',default_value='0'),
        DeclareLaunchArgument('initial_yaw',default_value='0'),DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('namespace',default_value='',description='Robot namespace (multi-robot lite twin); empty: root'),
        DeclareLaunchArgument('frame_prefix',default_value='',description='TF frame prefix of that robot, e.g. amr_02/'),
        DeclareLaunchArgument('fleet_obstacles',default_value='false',choices=['true','false'],description='Costmap layer with the other AMRs (<ns>/fleet/robots, multi-robot lite twin)'),
        OpaqueFunction(function=start)])
