"""Warehouse composition, reusing the AMR launch and its exact control stack."""
import hashlib
import json
import math
import os
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def verify_generated(share, amr):
    """Fail clearly if any source or generated artifact is stale or edited."""
    manifest = json.loads((share/'config/generated_manifest.json').read_text())
    for name, expected in {**manifest['source_sha256'], **manifest['files']}.items():
        path = share/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Warehouse file changed: '+name+'; run build_warehouse.sh to regenerate.')
    dimensions = json.loads((amr/'config/amr_dimensions.json').read_text())
    if hashlib.sha256(json.dumps(dimensions,sort_keys=True).encode()).hexdigest() != manifest['amr_config_sha256']:
        raise RuntimeError('AMR dimensions changed; run build_warehouse.sh to revalidate warehouse clearances.')


def setup(context):
    share = Path(get_package_share_directory('amr_simulation'))
    amr = Path(get_package_share_directory('amr_description'))
    scenario = LaunchConfiguration('scenario').perform(context)
    if scenario not in ('normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked'):
        raise ValueError('Unknown warehouse scenario: '+scenario)
    verify_generated(share, amr)
    c = json.loads((share/'config/warehouse_layout.json').read_text())
    spawn = next(s for s in c['spawns'] if s['active'])['pose']
    camera = c['camera']
    args = {'world': str(share/'worlds'/('warehouse_'+scenario+'.sdf')),
            'world_name': c['world_name'], 'resource_path': str(share/'models'),
            'gui_config': str(share/'config'/('warehouse_gui_'+scenario+'.config')),
            'gui': LaunchConfiguration('gui').perform(context),
            'gui_renderer': LaunchConfiguration('gui_renderer').perform(context),
            'rviz': LaunchConfiguration('rviz').perform(context)}
    for key, value in zip(('spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw'),
                          (spawn[0], spawn[1], spawn[2], spawn[5])):
        args[key] = str(value)
    def transform(parent, child, pose):
        options = ['--frame-id', parent, '--child-frame-id', child]
        for flag, value in zip(('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), pose):
            options += [flag, str(value)]
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=child+'_tf', arguments=options, parameters=[{'use_sim_time': True}])
    return [
        # Explicit partitions remain usable by test observers. Interactive
        # launches get a fresh partition so an abandoned server cannot answer
        # their create/scene requests or supply the GUI with an older world.
        SetEnvironmentVariable('GZ_PARTITION', partition := os.environ.get(
            'GZ_PARTITION') or 'warehouse_'+uuid.uuid4().hex),
        LogInfo(msg=f'Warehouse scenario={scenario}; world={args["world"]}; '
                f'sha256={hashlib.sha256(Path(args["world"]).read_bytes()).hexdigest()}; '
                f'spawn={spawn}; GZ_PARTITION={partition}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(amr/'launch/sim.launch.py')),
                                 launch_arguments=args.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='warehouse_bridge',
             parameters=[{'config_file': str(share/'config/warehouse_bridge.yaml'), 'use_sim_time': True}],
             output='screen'),
        transform(c['coordinates']['frame'], camera['frame'], camera['pose']),
        transform(camera['frame'], camera['optical_frame'], [0, 0, 0, -math.pi/2, 0, -math.pi/2]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='normal'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gui_renderer', default_value='ogre'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup),
    ])
