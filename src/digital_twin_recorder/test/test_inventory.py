"""Profiles, converters of the lite/navigation messages and their Mosaico ontologies."""
import math
import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from amr_lite_interfaces.msg import EntityState, Obstacle, ObstacleArray, SimEvent, WorldState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from digital_twin_recorder.config import Config
from digital_twin_recorder.models.records import Integrity, canonical, convert_message
from digital_twin_recorder.models.recording_metadata import ONTOLOGY_TAGS, create_metadata, recording_configuration
from digital_twin_recorder.topics import inventory


def world_state():
    msg = WorldState(session_id='3f1c', epoch=0, sim_time=12.5, real_time_factor=.998, target_real_time_factor=1.,
                     paused=False, overruns=3, scene_sha256='ab'*32)
    msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id = 12, 500_000_000, 'world'
    msg.entities = [EntityState(id='amr_01', type='amr', x=2., y=1.5, yaw=1.57, vx=.0, vy=.6, wz=.01, speed=.6,
                                state='moving', field_state=1, field_length=.66),
                    EntityState(id='COL-01', type='human', x=-55., y=33., z=.03, yaw=-1.2, speed=1.1, state='walking',
                                activity='scan', activity_progress=.25, carrying=True, look=.4),
                    EntityState(id='EMP-1', type='forklift', x=10., y=22.5, yaw=3.14, speed=1.3, state='moving',
                                field_state=0, field_length=2.4, reason='')]
    msg.obstacles = [Obstacle(id='obstacle_0001', kind='box', origin='user', x=1., y=2., length=.62, width=.52, height=.5)]
    return msg


def scan():
    msg = LaserScan(angle_min=-math.pi/2, angle_max=math.pi/2, angle_increment=math.pi/360, scan_time=1/15,
                    range_min=.08, range_max=20.)
    msg.header.stamp.sec, msg.header.frame_id = 7, 'laser_frame'
    msg.ranges = [1.25, math.inf, 3.5, math.inf]
    return msg


def messages():
    event = SimEvent(seq=4, kind='protective_stop', entity='amr_01', other='COL-03', detail='person inside', x=1., y=2.)
    event.header.stamp.sec, event.header.frame_id = 9, 'world'
    obstacles = ObstacleArray(obstacles=[Obstacle(id='obstacle_0002', kind='cone', origin='user', x=3., y=4.)])
    obstacles.header.frame_id = 'world'
    status = GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)])
    status.status_list[0].goal_info.goal_id.uuid = list(range(16))
    status.status_list[0].goal_info.stamp.sec = 5
    diagnostics = DiagnosticArray(status=[DiagnosticStatus(level=DiagnosticStatus.WARN, name='navigation/interlock',
                                                           message='Missing or stale command', hardware_id='warehouse_amr',
                                                           values=[KeyValue(key='localization_source', value='AMCL')])])
    diagnostics.header.stamp.sec = 9
    path = Path(poses=[PoseStamped(), PoseStamped()])
    path.header.frame_id = 'map'; path.header.stamp.sec = 10
    path.poses[1].pose.position.x, path.poses[1].pose.orientation.w = 1.5, 1.
    tf = TFMessage(transforms=[TransformStamped(), TransformStamped()])
    tf.transforms[0].header.frame_id, tf.transforms[0].child_frame_id = 'map', 'odom'
    tf.transforms[0].header.stamp.sec, tf.transforms[0].transform.rotation.w = 11, 1.
    tf.transforms[1].header.frame_id, tf.transforms[1].child_frame_id = 'odom', 'base_link'
    return {'world_state': world_state(), 'sim_event': event, 'obstacles': obstacles, 'goal_status': status,
            'diagnostics': diagnostics, 'path': path, 'tf': tf, 'tf_static': tf, 'scan': scan()}


def test_profiles_and_backends():
    core = {s.topic for s in inventory(Config())}
    extended = {s.topic for s in inventory(Config(profile='extended'))}
    assert {'/odom', '/imu/data', '/lite_sim/world_state', '/lite_sim/events', '/lite_sim/obstacles',
            '/navigate_to_pose/_action/status', '/navigation/diagnostics'} == core
    assert extended == core | {'/plan', '/tf', '/tf_static', '/scan', '/top_scan'}
    gazebo = {s.topic for s in inventory(Config(simulation_backend='gazebo', scenario='normal'))}
    assert '/lite_sim/world_state' not in gazebo and '/odom' in gazebo
    with pytest.raises(ValueError):
        Config(scenario='normal')          # Gazebo scenario names never apply to the lite engine
    with pytest.raises(ValueError):
        Config(profile='everything')
    with pytest.raises(ValueError):
        Config(world_state_hz=-1)


def test_every_lite_and_navigation_message_round_trips_its_ontology():
    pytest.importorskip('mosaicolabs')
    from digital_twin_recorder.models.ontology import ONTOLOGIES
    assert set(ONTOLOGY_TAGS.values()) <= set(ONTOLOGIES)
    for kind, msg in messages().items():
        record = convert_message(kind, msg, '/t', 'amr_01', 456, reception_sim_ns=789, tf_pairs=(('map', 'odom'),))
        model = ONTOLOGIES[ONTOLOGY_TAGS[kind]]
        stored = model.model_validate(record).model_dump()
        assert canonical(stored) == canonical(record), kind
        assert model.__msco_pyarrow_struct__.field('source_timestamp_ns').type.bit_width == 64


def test_conversion_keeps_identity_frames_and_values():
    m = messages()
    w = convert_message('world_state', m['world_state'], '/lite_sim/world_state', 'amr_01', 1)
    assert w['source_timestamp_ns'] == 12_500_000_000 and w['frame_id'] == 'world' and w['session_id'] == '3f1c'
    assert [e['id'] for e in w['entities']] == ['amr_01', 'COL-01', 'EMP-1']
    assert w['entities'][0]['speed'] == pytest.approx(.6) and w['entities'][1]['carrying'] is True
    assert w['obstacles'][0]['kind'] == 'box'
    g = convert_message('goal_status', m['goal_status'], '/s', 'amr_01', 1, reception_sim_ns=789)
    assert g['source_timestamp_ns'] == 789 and g['statuses'][0] == dict(goal_id=bytes(range(16)).hex(), status=4,
                                                                         accepted_ns=5_000_000_000)
    d = convert_message('diagnostics', m['diagnostics'], '/d', 'amr_01', 1)
    assert d['statuses'][0]['level'] == 1 and d['statuses'][0]['values'] == [{'key': 'localization_source', 'value': 'AMCL'}]
    p = convert_message('path', m['path'], '/plan', 'amr_01', 1)
    assert p['frame_id'] == 'map' and p['poses'][1]['position'] == [1.5, 0., 0.]


def test_tf_keeps_only_configured_pairs():
    tf = messages()['tf']
    kept = convert_message('tf', tf, '/tf', 'amr_01', 1, tf_pairs=(('map', 'odom'),))
    assert [t['child_frame_id'] for t in kept['transforms']] == ['odom'] and kept['source_timestamp_ns'] == 11_000_000_000
    assert convert_message('tf', tf, '/tf', 'amr_01', 1, tf_pairs=(('world', 'map'),)) is None
    static = convert_message('tf_static', tf, '/tf_static', 'amr_01', 1, reception_sim_ns=42)
    assert len(static['transforms']) == 2 and static['source_timestamp_ns'] == 42


def test_scan_keeps_rep117_infinity_and_hashes_it():
    record = convert_message('scan', scan(), '/scan', 'amr_01', 1)
    assert record['ranges'][1] == math.inf
    integrity = Integrity(); integrity.add(record)
    assert integrity.last['ranges'][1] == '+Inf'          # JSON-safe report
    other = dict(record, ranges=[1.25, 19.9, 3.5, math.inf])
    assert canonical(other) != canonical(record)
    bad = world_state(); bad.entities[0].x = math.nan
    with pytest.raises(ValueError, match='Nonfinite'):
        convert_message('world_state', bad, '/w', 'amr_01', 1)


def test_metadata_is_backend_specific_and_never_guessed():
    config = Config(profile='extended')
    specs = inventory(config)
    meta = create_metadata(config, specs, {'simulation_session_id': 'abc', 'unavailable': ['map_version']})
    assert meta['execution_id'].startswith('digital_twin_lite_amr_01_') and meta['simulator'] == 'amr_lite_sim'
    assert meta['simulation_session_id'] == 'abc' and 'map_version' in meta['unavailable'] and 'scenario' not in meta
    assert meta['recording_configuration'] == recording_configuration(config, specs)
    assert all(':' not in k and '/' not in k for k in meta)            # Mosaico metadata key charset
    gazebo = create_metadata(Config(simulation_backend='gazebo', scenario='normal'), inventory(
        Config(simulation_backend='gazebo', scenario='normal')))
    assert gazebo['simulator'] == 'gazebo_harmonic' and gazebo['scenario'] == 'normal'
