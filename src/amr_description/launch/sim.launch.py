"""Harmonic simulation, sensors and ros2_control; no Gazebo Classic plugins."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    share = Path(get_package_share_directory('amr_description'))
    sensors = LaunchConfiguration('sensors').perform(context)
    if sensors not in ('full', 'lidar'):
        raise ValueError('sensors must be full or lidar')
    description = xacro.process_file(str(share/'urdf/amr.urdf.xacro'),mappings={
        'gazebo':'true','controllers_file':str(share/'config/controllers.yaml'),'sensors':sensors}).toxml()
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    # Keep the validated EGL sensor renderer independent of the optional GUI display.
    world = LaunchConfiguration('world').perform(context)
    # Own the server process directly: ros_gz_sim's shell=True wrapper can exit
    # on SIGINT while leaving its Ruby/Gazebo child alive in the same partition.
    server = ExecuteProcess(cmd=['gz', 'sim', '-r', '-v', '3', '-s',
        '--headless-rendering', world], output='screen', on_exit=Shutdown())
    if gui:
        renderer = LaunchConfiguration('gui_renderer').perform(context)
        if renderer not in ('ogre','ogre2'):
            raise ValueError('gui_renderer must be ogre or ogre2')
    spawn = Node(package='ros_gz_sim',executable='create',output='screen',arguments=[
        '-world',LaunchConfiguration('world_name').perform(context),
        '-name','warehouse_amr','-topic','robot_description',
        '-x',LaunchConfiguration('spawn_x').perform(context),
        '-y',LaunchConfiguration('spawn_y').perform(context),
        '-z',LaunchConfiguration('spawn_z').perform(context),
        '-Y',LaunchConfiguration('spawn_yaw').perform(context),'-allow_renaming','false'])
    ready = ExecuteProcess(cmd=['python3', str(share/'scripts/wait_gazebo.py'),
        LaunchConfiguration('world_name').perform(context)], output='screen')
    def after_ready(event, _context):
        if event.returncode != 0:
            raise RuntimeError('Gazebo was not ready; AMR was not spawned')
        return [spawn]
    controllers = Node(package='controller_manager',executable='spawner',output='screen',arguments=[
        'joint_state_broadcaster','diff_drive_controller','--activate-as-group',
        '--controller-manager-timeout','90', '--controller-ros-args',
        '--remap /diff_drive_controller/cmd_vel:=/cmd_vel --remap /diff_drive_controller/odom:=/odom'])
    def after_spawn(event, _context):
        if event.returncode != 0:
            raise RuntimeError('AMR spawn failed; controllers were not started')
        return [controllers]
    def after_controllers(event, _context):
        if event.returncode != 0:
            raise RuntimeError('AMR controllers failed; GUI was not started')
        # The warehouse has many visuals. Opening the GUI concurrently with
        # entity creation intermittently omitted the AMR from its initial
        # scene despite valid server poses. Load the complete scene after the
        # robot and controllers exist, as in the standalone AMR GUI validator.
        if gui:
            return [ExecuteProcess(cmd=['gz','sim','-g','--render-engine-gui',renderer,
                '--gui-config',LaunchConfiguration('gui_config').perform(context)],
                output='screen',on_exit=Shutdown())]
        return []
    actions = [
        LogInfo(msg='AMR xacro: '+str(share/'urdf/amr.urdf.xacro')+
                '; entity: warehouse_amr; world: '+LaunchConfiguration('world_name').perform(context)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',os.pathsep.join(filter(None,[
            str(share.parent),LaunchConfiguration('resource_path').perform(context),
            os.environ.get('GZ_SIM_RESOURCE_PATH','')]))),
        # Mesa's default zink-on-NVK GL path renders only the front cube face of the
        # ogre2 gpu_lidar: beams beyond +/-45 deg return +inf or phantom ranges
        # (PHASE_5_1_REMEDIATION.md, LiDAR rendering defect). The nouveau GL driver
        # renders every face; other drivers ignore this variable. An explicit value wins.
        SetEnvironmentVariable('NOUVEAU_USE_ZINK',os.environ.get('NOUVEAU_USE_ZINK','0')),
        server,
        Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
             parameters=[{'robot_description':description,'use_sim_time':True}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',output='screen',
             parameters=[{'config_file':LaunchConfiguration('bridge_config').perform(context) or str(share/'config/bridge.yaml'),
                          'use_sim_time':True}]),
        RegisterEventHandler(OnProcessExit(target_action=ready,on_exit=after_ready)),
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=controllers,on_exit=after_controllers)),
        ready]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/amr.rviz')],
                            parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('amr_description'))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('gui_renderer',default_value='ogre2'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('world',default_value=str(share/'worlds/warehouse_test.sdf')),
        DeclareLaunchArgument('world_name',default_value='amr_validation'),
        DeclareLaunchArgument('spawn_x',default_value='0'),
        DeclareLaunchArgument('spawn_y',default_value='0'),
        DeclareLaunchArgument('spawn_z',default_value='0.015'),
        DeclareLaunchArgument('spawn_yaw',default_value='0'),
        DeclareLaunchArgument('resource_path',default_value=''),
        DeclareLaunchArgument('gui_config',default_value=str(share/'config/gazebo_gui.config')),
        DeclareLaunchArgument('sensors',default_value='full',description='full or lidar (no RGB-D camera)'),
        DeclareLaunchArgument('bridge_config',default_value='',description='Empty: config/bridge.yaml'),
        OpaqueFunction(function=setup)])
