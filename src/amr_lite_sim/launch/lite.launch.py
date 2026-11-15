"""Lite engine + one robot_state_publisher per robot. Starts no Gazebo process.

``robots``: 'all' (every active spawn of the scene: amr_01, amr_02, amr_03) or a comma-separated
subset ('amr_01' is the Phase 6 single-robot twin). Interface names: amr_lite_sim/fleet.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_ids(context, value):
    """Robots the engine will create: the requested ids, or every active spawn of the scene."""
    from amr_lite_sim import fleet
    from atlas_scene.scene import Scene
    requested = fleet.parse_robots(value)
    if requested is not None:
        return requested
    scene = Scene.load(LaunchConfiguration('scene_file').perform(context) or None)
    return [s['robot_id'] for s in scene.section('spawns') if s['active']]


def setup(context):
    from amr_lite_sim import fleet
    from amr_lite_sim.robot_model import robot_urdf
    value = lambda key: LaunchConfiguration(key).perform(context)
    urdf = robot_urdf()
    robots = robot_ids(context, value('robots'))
    # One state publisher per robot: same URDF, frames prefixed per robot (shared /tf_static).
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                    namespace=fleet.namespace(r) or None,
                    parameters=[{'robot_description': urdf, 'use_sim_time': True, 'frame_prefix': fleet.frame_prefix(r)}])
               for r in robots]
    actions.append(Node(package='amr_lite_sim', executable='lite_sim', output='screen', on_exit=Shutdown(reason='lite_sim exited'),
                        parameters=[{'profile': value('profile'), 'real_time_factor': float(value('real_time_factor')),
                                     'scene_file': value('scene_file'), 'lidar_noise_stddev': float(value('lidar_noise_stddev')),
                                     'robots': ','.join(robots)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='operations', choices=['operations', 'mapping', 'empty'],
                              description='operations: workers + forklifts; mapping/empty: no agents'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('scene_file', default_value='', description='Empty: installed atlas_scene campus'),
        DeclareLaunchArgument('lidar_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('robots', default_value='all',
                              description="'all' (every active spawn) or comma-separated ids, e.g. amr_01"),
        OpaqueFunction(function=setup),
    ])
