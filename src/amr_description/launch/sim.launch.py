"""Harmonic simulation, sensors and ros2_control; no Gazebo Classic plugins."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    share = Path(get_package_share_directory('amr_description'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    description = xacro.process_file(str(share/'urdf/amr.urdf.xacro'),mappings={
        'gazebo':'true','controllers_file':str(share/'config/controllers.yaml')}).toxml()
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    # Keep the validated EGL sensor renderer independent of the optional GUI display.
    args = '-r -v 3 -s --headless-rendering ' + str(share/'worlds/warehouse_test.sdf')
    if gui:
        renderer = LaunchConfiguration('gui_renderer').perform(context)
        if renderer not in ('ogre','ogre2'):
            raise ValueError('gui_renderer must be ogre or ogre2')
    spawn = Node(package='ros_gz_sim',executable='create',output='screen',arguments=[
        '-world','amr_validation','-name','warehouse_amr','-topic','robot_description',
        '-x','0','-y','0','-z','0.015','-allow_renaming','false'])
    controllers = Node(package='controller_manager',executable='spawner',output='screen',arguments=[
        'joint_state_broadcaster','diff_drive_controller','--activate-as-group',
        '--controller-manager-timeout','90', '--controller-ros-args',
        '--remap /diff_drive_controller/cmd_vel:=/cmd_vel --remap /diff_drive_controller/odom:=/odom'])
    def after_spawn(event, _context):
        if event.returncode != 0:
            raise RuntimeError('AMR spawn failed; controllers were not started')
        return [controllers]
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(share.parent)+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_share/'launch/gz_sim.launch.py')),
                                 launch_arguments={'gz_args':args}.items()),
        Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
             parameters=[{'robot_description':description,'use_sim_time':True}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',output='screen',
             parameters=[{'config_file':str(share/'config/bridge.yaml'),'use_sim_time':True}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_spawn)),
        spawn]
    if gui:
        actions.append(ExecuteProcess(cmd=['gz','sim','-g','--render-engine-gui',renderer,
            '--gui-config',str(share/'config/gazebo_gui.config')],output='screen'))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/amr.rviz')],
                            parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('gui_renderer',default_value='ogre2'),
        DeclareLaunchArgument('rviz',default_value='false'),
        OpaqueFunction(function=setup)])
