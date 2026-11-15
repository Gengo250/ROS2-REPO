import math
from datetime import datetime, timezone
import pytest
from fastapi.testclient import TestClient
from digital_twin_gateway.api.app import create_app
from digital_twin_gateway.config import Settings
from digital_twin_gateway.world.models import CommandResult, WorldObstacle
from digital_twin_gateway.world.registration import Registration
from digital_twin_gateway.world.store import WorldStore
from conftest import Runtime, Timer
from test_state import assert_schema

REG = Registration('reg', 0., 2., -math.pi/2, 'sha')  # campus: map origin at the spawn (2, 0, pi/2)


def engine(**changes):
    entity = dict(id='amr_01', type='amr', x=2., y=0., z=0., yaw=math.pi/2, vx=0., vy=.4, wz=0., speed=.4,
                  state='moving', activity=None, activity_progress=0., carrying=False, look=0.,
                  field_state='warning', field_length=.46, reason=None)
    worker = dict(entity, id='COL-01', type='human', state='working', activity='scan', activity_progress=.5,
                  field_state='free', field_length=0.)
    return {'session_id': 'engine-a', 'sim_time': 12.5, 'rtf': 1.0, 'target_rtf': 1.0, 'paused': False,
            'overruns': 0, 'scene_sha256': 'abc', 'entities': [entity, worker],
            'obstacles': [dict(id='obstacle_0001', kind='box', origin='user', x=5., y=1., yaw=0.,
                               length=.62, width=.52, height=.5)], **changes}


@pytest.fixture
def store():
    timer = Timer()
    s = WorldStore(Settings(), REG, timer, lambda: datetime(2026, 10, 31, tzinfo=timezone.utc), lambda: 1.0)
    s.set_ros_initialized(True)
    return s, timer


def test_snapshot_matches_contract_and_ages(store):
    s, timer = store
    first = s.snapshot().model_dump(mode='json')
    assert_schema(first, 'world-state.v1')
    assert first['simulation']['backend'] == 'none' and first['entities'] == []
    s.update_engine(engine())
    s.add_event(dict(seq=1, sim_time=12., kind='obstacle_added', entity='obstacle_0001', other='', detail='box', x=5., y=1.))
    s.add_event(dict(seq=1, sim_time=12., kind='duplicate', entity='', other='', detail='', x=0., y=0.))
    live = s.snapshot().model_dump(mode='json')
    assert_schema(live, 'world-state.v1')
    assert live['simulation']['status'] == 'running' and live['connection']['registration'] == 'verified'
    assert [e['kind'] for e in live['events']] == ['obstacle_added']
    assert live['robots'][0]['id'] == 'amr_01' and live['sequence'] > first['sequence']
    timer.value = 3.
    assert s.snapshot().simulation.status == 'stalled'
    timer.value = 11.
    stale = s.snapshot()
    # Last values are kept but explicitly offline; nothing is invented.
    assert stale.simulation.status == 'offline' and stale.entities[0].x == 2.


def test_new_engine_session_resets_events(store):
    s, _ = store
    s.update_engine(engine())
    s.add_event(dict(seq=5, sim_time=1., kind='contact', entity='amr_01', other='structure', detail='', x=0., y=0.))
    s.update_engine(engine(session_id='engine-b'))
    s.add_event(dict(seq=1, sim_time=1., kind='resumed', entity='simulation', other='', detail='', x=0., y=0.))
    assert [e.kind for e in s.snapshot().events] == ['resumed']


def test_plan_converted_to_world_or_omitted(store):
    s, _ = store
    s.update_engine(engine())
    s.update_plan('amr_01', [(0., 0.), (10., 0.)], 'map')
    path = s.snapshot().robots[0].path
    assert path[0] == pytest.approx((2., 0.)) and path[1] == pytest.approx((2., 10.))
    s.update_plan('amr_01', [(0., 0.)], 'odom')
    robot = s.snapshot().robots[0]
    assert robot.path == () and 'unsupported' in robot.path_omitted_reason
    s.update_plan('amr_01', [(float(i), 0.) for i in range(1000)], 'map')
    assert len(s.snapshot().robots[0].path) <= 256


def test_registration_round_trip():
    for wx, wy in [(2., 0.), (-55., 36.), (63.4, 33.)]:
        assert REG.map_to_world(*REG.world_to_map(wx, wy)) == pytest.approx((wx, wy))
    assert REG.world_to_map(2., 0.) == pytest.approx((0., 0.))
    assert REG.world_yaw_to_map(math.pi/2) == pytest.approx(0.)


class FakeCommands:
    def __init__(self):
        self.calls = []

    async def add_obstacle(self, request):
        self.calls.append(('add', request.kind))
        if request.x > 100:
            return CommandResult(accepted=False, reason='outside the building')
        return CommandResult(accepted=True, reason=None, obstacle=WorldObstacle(
            id='obstacle_0001', kind=request.kind, origin='user', x=request.x, y=request.y, yaw=0., length=1., width=1., height=1.))

    async def remove_obstacle(self, identifier=''):
        self.calls.append(('remove', identifier))
        return CommandResult(accepted=True, reason=None, removed=1)


def test_world_routes_commands_and_socket(tmp_path):
    scene = tmp_path/'scene.json'
    scene.write_text('{"stations": [{"id": "dock_01", "pose": [1, 2, 0, 0, 0, 0]}]}')
    fake = FakeCommands()

    class CommandRuntime(Runtime):
        commands = fake
    settings = Settings(world_scene_path=str(scene), world_publication_hz=50)
    with TestClient(create_app(settings, Runtime)) as client:
        assert client.post('/api/v1/obstacles', json={'kind': 'cone', 'x': 1, 'y': 2}).status_code == 503
    with TestClient(create_app(settings, CommandRuntime)) as client:
        assert_schema(client.get('/api/v1/world').json(), 'world-state.v1')
        assert client.get('/api/v1/world/scene').json()['stations'][0]['id'] == 'dock_01'
        created = client.post('/api/v1/obstacles', json={'kind': 'cone', 'x': 1, 'y': 2})
        assert created.status_code == 201 and created.json()['obstacle']['kind'] == 'cone'
        assert_schema(created.json(), 'command-result.v1')
        rejected = client.post('/api/v1/obstacles', json={'kind': 'box', 'x': 500, 'y': 2})
        assert rejected.status_code == 409 and rejected.json()['reason'] == 'outside the building'
        assert client.post('/api/v1/obstacles', json={'kind': 'piano', 'x': 1, 'y': 2}).status_code == 422
        assert client.post('/api/v1/obstacles', json={'kind': 'cone', 'x': 1, 'y': 2, 'extra': 1}).status_code == 422
        assert client.delete('/api/v1/obstacles').json()['removed'] == 1
        preflight = client.options('/api/v1/obstacles', headers={'Origin': 'http://localhost:5173',
                                   'Access-Control-Request-Method': 'POST', 'Access-Control-Request-Headers': 'content-type'})
        assert preflight.status_code == 200
        with client.websocket_connect('/ws/v1/world?protocol_version=1.0') as socket:
            first = socket.receive_json()
            assert_schema(first, 'world-state.v1')
            assert socket.receive_json()['sequence'] > first['sequence']
    assert ('remove', '') in fake.calls


def test_fleet_navigation_is_per_robot(store):
    """Each AMR has its own interlock, plan and availability; nothing is shared or reattributed."""
    s, timer = store
    amr = engine()['entities'][0]
    s.update_engine(engine(entities=[amr, dict(amr, id='amr_02', x=-5.), dict(amr, id='amr_03', x=9.)]))
    s.update_interlock('amr_02', 'ready')
    s.update_plan('amr_03', [(0., 0.), (0., 4.)], 'map')
    snap = s.snapshot()
    assert_schema(snap.model_dump(mode='json'), 'world-state.v1')
    robots = {r.id: r for r in snap.robots}
    assert set(robots) == {'amr_01', 'amr_02', 'amr_03'}
    assert robots['amr_02'].interlock_ready and robots['amr_02'].interlock_availability == 'online'
    assert robots['amr_01'].interlock is None and not robots['amr_01'].interlock_ready
    assert robots['amr_03'].path and robots['amr_01'].path == () and robots['amr_02'].path == ()
    # Fleet navigation availability is the least available robot's (amr_01/amr_03 never reported).
    assert snap.connection.navigation == 'unknown'
    for robot in ('amr_01', 'amr_03'):
        s.update_interlock(robot, 'Missing or stale scan')
    assert s.snapshot().connection.navigation == 'online'
    timer.value = 3.
    s.update_interlock('amr_01', 'ready')
    snap = s.snapshot()
    assert {r.id: r.interlock_availability for r in snap.robots} == {'amr_01': 'online', 'amr_02': 'stale', 'amr_03': 'stale'}
    assert snap.connection.navigation == 'stale'


def test_fleet_settings_reject_duplicates():
    from pydantic import ValidationError
    from digital_twin_gateway.config import FleetRobot
    assert [r.topic('plan') for r in Settings().fleet] == ['/plan', '/amr_02/plan', '/amr_03/plan']
    with pytest.raises(ValidationError):
        Settings(fleet=(FleetRobot(id='amr_01'), FleetRobot(id='amr_01', namespace='x')))
    with pytest.raises(ValidationError):
        Settings(fleet=(FleetRobot(id='amr_01'), FleetRobot(id='amr_02')))   # both on the root topics


def test_missions_go_to_the_robots_own_nav2(store):
    """start_mission sends the goal to the requested robot's NavigateToPose client only."""
    import asyncio
    from types import SimpleNamespace
    pytest.importorskip('nav2_msgs')
    from nav2_msgs.action import NavigateToPose
    from digital_twin_gateway.ros.world import Commands
    from digital_twin_gateway.world.models import MissionRequest
    s, _ = store
    sent = []

    class Client:
        def __init__(self, robot):
            self.robot = robot

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal, feedback_callback=None):
            sent.append((self.robot, goal.pose.pose.position.x, goal.pose.pose.position.y))
            future = SimpleNamespace(add_done_callback=lambda cb: cb(future), exception=lambda: None,
                                     result=lambda: SimpleNamespace(accepted=False))
            return future

    commands = object.__new__(Commands)
    from builtin_interfaces.msg import Time
    commands.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)))
    commands.store, commands.settings, commands.NavigateToPose, commands.goals = s, Settings(), NavigateToPose, {}
    commands.stations = {'dock_01': {'pose': [2., 4., 0, 0, 0, 0]}}
    from digital_twin_gateway.world.traffic import Traffic
    commands.traffic, commands.tokens, commands.intent, commands.recovery_mark = Traffic({'dock_01': (2., 4., 0.)}), {}, {}, {}
    commands.navigate = {r.id: Client(r.id) for r in Settings().fleet}

    async def run(robot):
        return await commands.start_mission(MissionRequest(robot_id=robot, destination='dock_01'))
    result = asyncio.run(run('amr_02'))
    assert sent == [('amr_02', pytest.approx(4.), pytest.approx(0.))]   # world (2, 4) → map (4, 0)
    assert not result.accepted and s.mission('amr_02').status == 'rejected' and s.mission('amr_01') is None
    unknown = asyncio.run(run('amr_09'))
    assert not unknown.accepted and 'unknown robot' in unknown.reason and len(sent) == 1
