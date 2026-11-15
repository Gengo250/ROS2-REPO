"""Lite engine + Gateway + saved-map AMCL + Nav2 + command guard per robot. Starts no Gazebo process.

``robots`` ('all' = every active spawn: amr_01, amr_02, amr_03; or e.g. 'amr_01') selects the robots
the engine creates; ``navigation_robots`` (default: the same) the robots that get their own
independent stack (map server, AMCL, Nav2 servers, command guard) in their namespace
(amr_lite_sim/fleet.py: amr_01 on the root topics, amr_02/amr_03 under /amr_02, /amr_03).

The campus map's origin is amr_01's spawn pose (SLAM starts there), so amr_01 is initialised at
map (0, 0, 0); every other robot's spawn is converted world → map through the verified
campus_registration.json (the same transform the Gateway uses for goals and plans).

With more than one simulated AMR, ``fleet_tracker`` publishes the other robots' footprints to each
stack's fleet tracking costmap layer (navigation.launch.py fleet_obstacles:=true).
"""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_in_map(registration, pose):
    """World spawn pose (x, y, z, roll, pitch, yaw) → map (x, y, yaw); map = R(yaw)·world + (x, y)."""
    t = registration['transform']
    c, s = math.cos(t['yaw']), math.sin(t['yaw'])
    x, y, yaw = pose[0], pose[1], pose[5]
    return c*x-s*y+t['x'], s*x+c*y+t['y'], math.atan2(math.sin(yaw+t['yaw']), math.cos(yaw+t['yaw']))


def setup(context):
    from amr_lite_sim import fleet
    from atlas_scene.scene import Scene
    navigation = Path(get_package_share_directory('amr_navigation'))
    value = lambda key: LaunchConfiguration(key).perform(context)
    spawns = {s['robot_id']: s for s in Scene.load(None).section('spawns') if s['active']}
    robots = fleet.parse_robots(value('robots')) or list(spawns)
    navigated = fleet.parse_robots(value('navigation_robots')) or robots
    if set(navigated)-set(robots):
        raise RuntimeError(f'navigation_robots {sorted(set(navigated)-set(robots))} are not simulated (robots:={value("robots")})')
    registration = json.loads(Path(value('registration')).read_text())
    if not registration.get('verified') or registration.get('parent_frame') != 'world' or registration.get('child_frame') != 'map':
        raise RuntimeError(f'Unverified world/map registration {value("registration")}')
    actions = []
    # Several AMRs: each stack sees the others through the fleet tracking layer (amr_lite_sim
    # fleet_scan.py). One robot: the validated single-robot parameters, unchanged.
    fleet_tracking = len(robots) > 1
    if fleet_tracking:
        actions.append(Node(package='amr_lite_sim', executable='fleet_tracker', output='screen',
                            parameters=[{'robots': ','.join(navigated)}]))  # stamps come from the engine state, no /clock needed
    stagger = float(value('stack_stagger'))
    for index, robot in enumerate(navigated):
        x, y, yaw = spawn_in_map(registration, spawns[robot]['pose'])
        # Stacks start one after the other: bringing up three map servers, AMCLs and Nav2 servers
        # at once saturated the CPU and a lifecycle manager lost an activation response (its stack
        # stayed inactive). Each stack is independent; only its start is delayed.
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation/'launch/navigation.launch.py')),
            launch_arguments={'map': value('map'), 'amcl_scan_topic': value('amcl_scan_topic'), 'initialize': 'true',
                              'initial_x': f'{x:.6f}', 'initial_y': f'{y:.6f}', 'initial_yaw': f'{yaw:.6f}',
                              'simulation_backend': 'lite', 'rviz': value('rviz') if robot == navigated[0] else 'false',
                              'namespace': fleet.namespace(robot), 'frame_prefix': fleet.frame_prefix(robot),
                              'fleet_obstacles': 'true' if fleet_tracking else 'false'}.items())
        actions.append(TimerAction(period=index*stagger, actions=[include]) if index and stagger > 0 else include)
    return actions


def generate_launch_description():
    lite = Path(get_package_share_directory('amr_lite_sim'))/'launch'
    navigation = Path(get_package_share_directory('amr_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='operations', choices=['operations', 'mapping', 'empty']),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('robots', default_value='all', description="'all' (amr_01, amr_02, amr_03) or comma-separated ids"),
        DeclareLaunchArgument('navigation_robots', default_value='all', description="robots with a Nav2 stack ('all' = robots)"),
        DeclareLaunchArgument('map', default_value=str(navigation/'maps/campus.yaml')),
        DeclareLaunchArgument('registration', default_value=str(navigation/'maps/campus_registration.json')),
        # The campus map was built from the 360-degree scanner; AMCL must use the same sensor.
        DeclareLaunchArgument('amcl_scan_topic', default_value='/top_scan'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('stack_stagger', default_value='10.0', description='seconds between the robots\' stack starts'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lite/'lite_twin.launch.py')),
                                 launch_arguments={'profile': LaunchConfiguration('profile'),
                                                   'real_time_factor': LaunchConfiguration('real_time_factor'),
                                                   'robots': LaunchConfiguration('robots')}.items()),
        OpaqueFunction(function=setup),
    ])
