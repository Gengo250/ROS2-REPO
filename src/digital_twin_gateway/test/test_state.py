import json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import pytest
from jsonschema import Draft202012Validator, FormatChecker
from pydantic import ValidationError
from digital_twin_gateway.config import RobotConfig, Settings
from digital_twin_gateway.ros.state_collector import normalize_odometry
from digital_twin_gateway.state.state_manager import StateManager

ROOT = Path(__file__).resolve().parents[3]


def assert_schema(value, name):
    schema = json.loads((ROOT / 'shared/schemas' / f'{name}.schema.json').read_text())
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema, format_checker=FormatChecker()).validate(value)


def test_odometry_quaternion_velocity_and_frames(state, odom):
    manager, _ = state
    robot = normalize_odometry('amr_01', '/odom', odom, manager.wall_clock())
    manager.update_robot(robot)
    data = manager.publish().model_dump(mode='json')
    r = data['robots'][0]
    assert r['frame_id'] == 'robot_a/odom' and r['child_frame_id'] == 'robot_a/base_link'
    assert r['pose_source'] == '/odom'
    assert r['pose']['position'] == {'x': 1.2, 'y': -2.3, 'z': 0.4}
    assert r['pose']['orientation'] == {'x': .1, 'y': .2, 'z': .3, 'w': .9}
    assert r['velocity']['linear'] == {'x': .4, 'y': .2, 'z': -.1}
    assert r['velocity']['angular'] == {'x': .1, 'y': -.2, 'z': .3}
    assert r['source_timestamp'] == 123.25
    assert data['world']['frame_id'] == 'world'
    assert data['world']['status'] == 'unknown'
    assert_schema(data, 'twin-state')
    assert_schema(data['connection'], 'connection-status')


def test_missing_stale_offline_recovery_and_clock(state, odom):
    manager, timer = state
    missing = manager.publish()
    assert missing.robots[0].pose is None and missing.robots[0].velocity is None
    assert missing.robots[0].status == 'unknown'
    timer.value = 11
    assert manager.publish().robots[0].status == 'offline'
    manager.update_robot(normalize_odometry('amr_01', '/odom', odom, manager.wall_clock()))
    manager.update_clock(0)
    assert manager.publish().world.status != 'running'
    manager.update_clock(.1)
    assert manager.publish().world.status == 'running'
    assert manager.publish().robots[0].status == 'online'
    timer.value += 3
    manager.update_clock(.1)  # repeated clock != running
    old = manager.publish()
    assert old.robots[0].status == 'stale' and old.world.availability == 'stale'
    assert old.robots[0].pose is not None
    timer.value += 8
    assert manager.publish().robots[0].status == 'offline'
    manager.update_robot(normalize_odometry('amr_01', '/odom', odom, manager.wall_clock()))
    manager.update_clock(0)  # simulation reset is tolerated
    assert manager.publish().robots[0].status == 'online'
    manager.set_ros_initialized(False)
    assert manager.publish().connection.ros == 'offline'
    assert manager.publish().world.status == 'unavailable'


def test_static_tf_receipt_does_not_expire(state):
    manager, timer = state
    manager.touch('/tf_static')
    timer.value += 20
    topics = {t.topic: t for t in manager.publish().connection.topics}
    assert topics['/tf_static'].status == 'online'
    assert topics['/tf'].status == 'offline'


def test_snapshots_sequence_and_thread_safety(state, odom):
    manager, _ = state
    first = manager.publish()
    assert manager.snapshot() is first
    robot = normalize_odometry('amr_01', '/odom', odom, manager.wall_clock())
    with ThreadPoolExecutor(max_workers=3) as pool:
        list(pool.map(lambda _: manager.update_robot(robot), range(100)))
        sequences = list(pool.map(lambda _: manager.publish().sequence, range(100)))
    assert len(set(sequences)) == 100
    assert manager.snapshot().sequence == 101
    assert first.robots[0].pose is None
    with pytest.raises(ValidationError):
        first.sequence = 123
    assert manager.snapshot().connection.topics[3].messages_received == 100


def test_multiple_robots_independent_topics(odom):
    settings = Settings(robots=(RobotConfig(id='one', odom_topic='/one/odom'),
                                RobotConfig(id='two', odom_topic='/two/odom')))
    manager = StateManager(settings)
    manager.set_ros_initialized(True)
    manager.update_robot(normalize_odometry('two', '/two/odom', odom, manager.wall_clock()))
    one, two = manager.publish().robots
    assert one.pose is None and two.pose is not None


def test_bad_data_and_configuration_rejected(state, odom):
    manager, _ = state
    odom.pose.pose.position.x = float('nan')
    with pytest.raises(ValidationError):
        normalize_odometry('amr_01', '/odom', odom, manager.wall_clock())
    for config in ({'publication_hz': 0}, {'allowed_origins': ('*',)},
                   {'offline_seconds': 1}, {'robots': (RobotConfig(), RobotConfig())},
                   {'scenario': 'invented'}):
        with pytest.raises(ValidationError):
            Settings(**config)


def test_committed_schemas_match_models():
    import importlib.util
    path = ROOT / 'src/digital_twin_gateway/scripts/export_schemas.py'
    spec = importlib.util.spec_from_file_location('export_schemas', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for model, name in ((module.TwinState, 'twin-state'), (module.ConnectionStatus, 'connection-status')):
        assert json.loads((ROOT / f'shared/schemas/{name}.schema.json').read_text()) == module.schema(model, name)
