"""Ground-truth frame, hierarchy, epoch and v1 compatibility regressions."""
from datetime import datetime, timezone
import pytest
from fastapi.testclient import TestClient
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.pose_v_pb2 import Pose_V
from digital_twin_gateway.config import Settings
from digital_twin_gateway.ros.simulation_adapter import scene_frames, convert_poses
from digital_twin_gateway.ros.state_collector import normalize_simulation
from digital_twin_gateway.api.app import create_app
from digital_twin_gateway.state.models import TwinStateV2
from conftest import Runtime
from test_state import assert_schema


def sample(stamp=5):
    scene = Scene()
    model = scene.model.add(name='human_01', id=1)
    link = model.link.add(name='body', id=2)
    link.visual.add(name='leg', id=3)
    scene.model.add(name='warehouse_amr', id=4)
    scene.model.add(name='human_03', id=5)
    poses = Pose_V()
    poses.header.stamp.sec = stamp
    for identity, name, x in [(1, 'human_01', 3), (2, 'body', .1), (3, 'leg', .2),
                               (4, 'warehouse_amr', -11), (5, 'human_03', 11.5)]:
        p = poses.pose.add(id=identity, name=name)
        p.position.x = x
        p.orientation.z, p.orientation.w = .6, .8
    return convert_poses(poses, scene_frames(scene, Settings().simulation_entities, 'world'))


def normalized(message):
    return normalize_simulation(Settings(), message, datetime.now(timezone.utc))


def test_scoped_hierarchy_and_quaternion():
    message = sample()
    entities = normalized(message)
    human = next(e for e in entities if e.id == 'human_01')
    assert human.pose.position.x == 3
    assert human.pose.orientation.z == .6
    assert human.components[0].frame_id == 'human_01'
    assert human.components[1].frame_id == 'human_01/body'
    assert human.components[1].pose.position.x == .2
    assert all(e.source_timestamp == 5 for e in entities)
    assert next(e for e in entities if e.id == 'amr_01').gazebo_name == 'warehouse_amr'
    assert all(e.velocity is None for e in entities)


@pytest.mark.parametrize('failure', ['duplicate', 'frame', 'quaternion', 'time', 'parent'])
def test_reject_bad_frames(failure):
    msg = sample()
    if failure == 'duplicate': msg.transforms.append(msg.transforms[0])
    if failure == 'frame': msg.transforms[0].header.frame_id = 'odom'
    if failure == 'quaternion': msg.transforms[0].transform.rotation.w = 0.
    if failure == 'time': msg.transforms[0].header.stamp.sec = 9
    if failure == 'parent': msg.transforms[2].header.frame_id = 'human_03'
    with pytest.raises(ValueError): normalized(msg)


def test_missing_stale_reappearance_and_reset(state):
    manager, timer = state
    manager.update_entities(normalized(sample()))
    v1 = manager.publish()
    assert v1.humans == () and v1.protocol_version == '1.0'
    v2 = manager.snapshot(2)
    TwinStateV2.model_validate_json(v2.model_dump_json())
    assert_schema(v2.model_dump(mode='json'), 'twin-state-v2')
    assert v2.humans[1].id == 'human_02' and v2.humans[1].status == 'unknown'
    timer.value = 3
    manager.update_entities(normalized(sample()))
    manager.publish()
    assert manager.snapshot(2).humans[0].status == 'stale'
    timer.value = 11
    manager.publish()
    assert manager.snapshot(2).humans[0].status == 'offline'
    manager.update_entities(normalized(sample(6)))
    manager.publish()
    assert manager.snapshot(2).humans[0].status == 'online'
    manager.update_entities(normalized(sample(1)))
    manager.publish()
    assert manager.snapshot(2).simulation.epoch == 1
    assert manager.snapshot(2).humans[0].source_timestamp == 1
    manager.update_clock(10)
    manager.update_clock(0)
    manager.publish()
    assert manager.snapshot(2).humans[0].pose is None


def test_v2_endpoints_preserve_v1():
    with TestClient(create_app(Settings(), Runtime)) as client:
        assert client.get('/api/v1/state').json()['humans'] == []
        state = client.get('/api/v2/state').json()
        assert len(state['humans']) == 3
        assert state['protocol_version'] == '2.0'
        with client.websocket_connect('/ws/v2/twin?protocol_version=2.0') as ws:
            TwinStateV2.model_validate(ws.receive_json())
        with client.websocket_connect('/ws/v2/twin') as ws:
            assert ws.receive_json()['protocol_version'] == '2.0'


def test_disappearance_is_independent_of_other_entities(state):
    manager, timer = state
    manager.update_entities(normalized(sample()))
    timer.value = 3
    message = sample(6)
    message.transforms = [t for t in message.transforms if not t.child_frame_id.startswith('human_01')]
    manager.update_entities(normalized(message))
    manager.publish()
    state = manager.snapshot(2)
    assert state.simulation_robots[0].status == 'online'
    assert state.humans[0].status == 'stale'
    manager.update_entities(normalized(sample(7)))
    manager.publish()
    assert manager.snapshot(2).humans[0].status == 'online'


def test_temporary_ids_and_ambiguous_names():
    scene = Scene()
    scene.model.add(name='human_01', id=10)
    poses = Pose_V()
    poses.header.stamp.sec = 1
    poses.pose.add(name='human_01', id=11).orientation.w = 1
    frames = scene_frames(scene, Settings().simulation_entities, 'world')
    assert not convert_poses(poses, frames).transforms
    poses.pose[0].id = 10
    assert convert_poses(poses, frames).transforms[0].child_frame_id == 'human_01'
    scene.model.add(name='human_01', id=12)
    with pytest.raises(ValueError, match='Ambiguous'):
        scene_frames(scene, Settings().simulation_entities, 'world')


@pytest.mark.parametrize('values', [
    {'simulation_pose_topic':'/tf'}, {'simulation_pose_topic':'/tf_static'},
    {'simulation_entities':{'one':'human_01','two':'human_01'}},
    {'simulation_entities':{'nested/model':'amr_01'}},
])
def test_invalid_ground_truth_configuration(values):
    with pytest.raises(ValueError): Settings(**values)
