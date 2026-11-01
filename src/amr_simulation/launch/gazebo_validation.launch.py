"""Optional physical validation of the lite engine: headless Gazebo, simplified campus, LiDAR-only AMR.

The world is generated from the same compiled scene as the lite engine (no workers, vehicles or
cameras); the AMR has both LiDARs and the IMU but no RGB-D camera; the bridge carries only clock,
scans and IMU. Never needed by lite mode. No GUI is started.
"""
import os
from pathlib import Path
import tempfile
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def setup(context):
    import importlib.util
    from atlas_scene.scene import Scene
    share = Path(get_package_share_directory('amr_simulation'))
    amr = Path(get_package_share_directory('amr_description'))
    spec = importlib.util.spec_from_file_location('campus_world', share/'scripts/generate_campus_world.py')
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    scene = Scene.load()
    text, count = generator.build(scene)
    temporary = tempfile.TemporaryDirectory(prefix='campus_validation_')
    world = Path(temporary.name)/'campus_validation.sdf'
    world.write_text(text)
    spawn = next(s for s in scene.section('spawns') if s['active'])['pose']
    return [
        SetEnvironmentVariable('GZ_PARTITION', os.environ.get('GZ_PARTITION') or 'campus_validation_'+uuid.uuid4().hex),
        LogInfo(msg=f'gazebo_validation: {count} static shapes from scene {scene.scene_sha256[:12]}; world {world}'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: temporary.cleanup())])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(amr/'launch/sim.launch.py')), launch_arguments={
            'world': str(world), 'world_name': 'campus_validation', 'gui': 'false', 'rviz': 'false',
            'sensors': 'lidar', 'bridge_config': str(amr/'config/bridge_lidar.yaml'),
            'spawn_x': str(spawn[0]), 'spawn_y': str(spawn[1]), 'spawn_z': '0.015',
            'spawn_yaw': str(spawn[5])}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])
