"""Warehouse composition, reusing the AMR launch and its exact control stack."""
import hashlib
import json
import math
import os
import uuid
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnShutdown
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
    cleanup = []
    if LaunchConfiguration('mapping_only', default='false').perform(context).lower() == 'true':
        if scenario != 'normal':
            raise ValueError('mapping_only requires the normal scenario')
        # Controlled survey: derive from the verified authoritative world.
        # Only workers are omitted; production worlds and collision geometry
        # are unchanged. Never derive an occupancy grid from this geometry.
        tree = ET.parse(args['world'])
        world = tree.getroot().find('world')
        human_ids = {human['id'] for human in c['humans']}
        for model in list(world.findall('model')):
            if model.get('name') in human_ids:
                world.remove(model)
        temporary = tempfile.TemporaryDirectory(prefix='warehouse_mapping_')
        args['world'] = str(Path(temporary.name)/'warehouse_mapping.sdf')
        tree.write(args['world'], encoding='unicode', xml_declaration=True)
        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup())])))
    for key, value in zip(('spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw'),
                          (spawn[0], spawn[1], spawn[2], spawn[5])):
        args[key] = str(value)
    def transform(parent, child, pose):
        options = ['--frame-id', parent, '--child-frame-id', child]
        for flag, value in zip(('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), pose):
            options += [flag, str(value)]
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=child+'_tf', arguments=options, parameters=[{'use_sim_time': True}])
    return cleanup + [
        # Explicit partitions remain usable by test observers. Interactive
        # launches get a fresh partition so an abandoned server cannot answer
        # their create/scene requests or supply the GUI with an older world.
        SetEnvironmentVariable('GZ_PARTITION', partition := os.environ.get(
            'GZ_PARTITION') or 'warehouse_'+uuid.uuid4().hex),
        # Worker yielding follows warehouse_layout.json. human_yielding:=false exists ONLY for the
        # intentional physical-contact acceptance test (validate_crossing_contact.py).
        *([SetEnvironmentVariable('WAREHOUSE_HUMAN_YIELDING', 'disabled'),
           LogInfo(msg='TEST MODE: worker yielding disabled for intentional contact validation')]
          if LaunchConfiguration('human_yielding', default='true').perform(context) == 'false' else []),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join(filter(None, [
            str(Path(get_package_prefix('amr_simulation'))/'lib'),
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))),
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
        DeclareLaunchArgument('mapping_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('human_yielding', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
