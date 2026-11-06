"""Execution metadata stored with every Mosaico sequence.

Only facts that can be read reliably are stored. Facts discovered from the running system
(engine session, simulation profile, loaded map) come from ``discovery``; anything that
could not be read is listed under ``unavailable`` instead of being guessed.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
import subprocess
from pathlib import Path
from uuid import uuid4

# Same tags as models/ontology.py (the ROS process never imports the storage SDK; a unit test
# checks that both stay equal).
ONTOLOGY_TAGS = {'odometry': 'digital_twin_ros_odometry_v1', 'imu': 'digital_twin_ros_imu_v1',
                 'world_state': 'digital_twin_lite_world_state_v1', 'obstacles': 'digital_twin_lite_obstacles_v1',
                 'sim_event': 'digital_twin_lite_event_v1', 'goal_status': 'digital_twin_goal_status_array_v1',
                 'diagnostics': 'digital_twin_diagnostic_array_v1', 'path': 'digital_twin_nav_path_v1',
                 'tf': 'digital_twin_tf_message_v1', 'tf_static': 'digital_twin_tf_message_v1',
                 'scan': 'digital_twin_laser_scan_v1'}
FRAMES = {'odometry': 'odom -> base_link: local odometry from the spawn pose, not global localization',
          'imu': 'imu_link', 'world_state': 'world (x east, y north, z up): engine ground truth',
          'obstacles': 'world', 'sim_event': 'world', 'path': 'map (Nav2 plan); world = inverse map registration',
          'tf': 'as published (configured parent -> child pairs)', 'tf_static': 'as published',
          'scan': 'sensor frame of the message', 'goal_status': 'none', 'diagnostics': 'none'}


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git_state():
    """Commit and dirty flag of the source tree this module was loaded from (symlink installs)."""
    here = Path(__file__).resolve().parent
    try:
        commit = subprocess.check_output(['git', '-C', str(here), 'rev-parse', 'HEAD'], text=True,
                                         stderr=subprocess.DEVNULL, timeout=5).strip()
        dirty = bool(subprocess.check_output(['git', '-C', str(here), 'status', '--porcelain', '--untracked-files=no'],
                                             text=True, stderr=subprocess.DEVNULL, timeout=5).strip())
        return commit, dirty
    except (subprocess.SubprocessError, OSError):
        return None, None


def recording_configuration(config, specs):
    return dict(profile=config.profile, simulation_backend=config.simulation_backend, robot_id=config.robot_id,
                queue_size=config.queue_size, io_timeout_s=config.io_timeout, finalize_timeout_s=config.finalize_timeout,
                tf_pairs=[list(p) for p in config.tf_pairs],
                topics=[dict(topic=s.topic, ros_type=s.ros_type, kind=s.kind, ontology_tag=ONTOLOGY_TAGS[s.kind], qos=s.qos,
                             max_rate_hz=s.max_rate_hz, required=s.required, source_stamp=s.stamp) for s in specs])


def share(package, relative):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))/relative


def create_metadata(config, specs, discovery=None):
    discovery = dict(discovery or {})
    unavailable = list(discovery.pop('unavailable', []))
    now = datetime.now(timezone.utc)
    execution_id = f'digital_twin_{config.simulation_backend}_{config.robot_id}_{now.strftime("%Y%m%dT%H%M%SZ")}_{uuid4().hex[:12]}'
    settings = recording_configuration(config, specs)
    commit, dirty = git_state()
    meta = dict(
        execution_id=execution_id, project='digital_twin', environment='simulation', recording_source='ros2',
        schema_version='2.0', ros_distribution=os.environ.get('ROS_DISTRO', 'jazzy'), robot_id=config.robot_id,
        simulation_backend=config.simulation_backend, recording_profile=config.profile, started_wall_utc=now.isoformat(),
        topics={s.topic: s.ros_type for s in specs}, recording_configuration=settings,
        configuration_sha256=hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest(),
        coordinate_frames={s.kind: FRAMES[s.kind] for s in specs},
        pose_semantics='odometry topics are local odometry; world_state entities are engine ground truth in world',
        timestamp_semantics={'source_timestamp_ns': 'ROS header stamp, simulation time (source_stamp=reception: '
                                                    'recorder simulation-clock time at reception)',
                             'received_wall_ns': 'Unix UTC nanoseconds at reception; Mosaico message timestamp'})
    if commit:
        meta.update(git_commit=commit, git_dirty=dirty)
    else:
        unavailable.append('git_commit')
    try:
        meta['amr_config_sha256'] = sha256_file(share('amr_description', 'config/amr_dimensions.json'))
    except Exception:
        unavailable.append('amr_config_sha256')
    if config.simulation_backend == 'lite':
        meta.update(simulator='amr_lite_sim', world='atlas_campus')
        try:
            scene_file = share('atlas_scene', 'config/atlas_campus.scene.json')
            scene = json.loads(scene_file.read_text())
            meta.update(scene_id=scene['scene_id'], scene_sha256=scene['metadata']['scene_sha256'],
                        scene_geometry_sha256=scene['metadata']['geometry_sha256'])
        except Exception:
            unavailable.append('scene_sha256')
    else:
        meta.update(simulator='gazebo_harmonic', world='warehouse')
        if config.scenario:
            meta['scenario'] = config.scenario
        try:
            meta['warehouse_config_sha256'] = sha256_file(share('amr_simulation', 'config/warehouse_layout.json'))
        except Exception:
            unavailable.append('warehouse_config_sha256')
    # Running-system facts (engine session, simulation profile, loaded map and its registration).
    meta.update({k: v for k, v in discovery.items() if v is not None})
    if config.simulation_backend == 'lite' and discovery.get('engine_scene_sha256') and meta.get('scene_sha256') \
            and discovery['engine_scene_sha256'] != meta['scene_sha256']:
        meta['scene_mismatch'] = 'engine scene differs from the installed scene file'
    if unavailable:
        meta['unavailable'] = sorted(set(unavailable))
    return meta


def map_facts(yaml_path):
    """map_version (image SHA-256, as Phase 5) and the verified registration that names that image."""
    import yaml
    path = Path(yaml_path)
    image = path.parent/yaml.safe_load(path.read_text())['image']
    facts = dict(map_yaml=str(path), map_version=sha256_file(image))
    registration = path.with_name(path.stem+'_registration.json')
    if registration.is_file():
        data = json.loads(registration.read_text())
        if data.get('verified') and data.get('map_sha256') == facts['map_version']:
            facts['map_registration'] = dict(id=data['id'], parent_frame=data['parent_frame'],
                                             child_frame=data['child_frame'], transform=data['transform'])
    return facts
