"""Lite engine + robot_state_publisher. Starts no Gazebo process."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from amr_lite_sim.robot_model import robot_urdf
    value = lambda key: LaunchConfiguration(key).perform(context)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_urdf(), 'use_sim_time': True}]),
        Node(package='amr_lite_sim', executable='lite_sim', output='screen', on_exit=Shutdown(reason='lite_sim exited'),
             parameters=[{'profile': value('profile'), 'real_time_factor': float(value('real_time_factor')),
                          'scene_file': value('scene_file'), 'lidar_noise_stddev': float(value('lidar_noise_stddev'))}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='operations', choices=['operations', 'mapping', 'empty'],
                              description='operations: workers + forklifts; mapping/empty: no agents'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('scene_file', default_value='', description='Empty: installed atlas_scene campus'),
        DeclareLaunchArgument('lidar_noise_stddev', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
