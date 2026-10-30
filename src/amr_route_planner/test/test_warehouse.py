"""Graph generation from the real layout, artifact staleness, planning and executor checks."""
import json
import math
from pathlib import Path
import shutil
import pytest
from amr_route_planner import graph_builder as gb
from amr_route_planner.cost_model import Restrictions
from amr_route_planner.graph_model import Graph
from amr_route_planner.route_planner import RoutingError, attach_start, load_context, plan

PATHS = gb.Paths.source_tree()
ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope='module')
def artifact():
    return gb.build_artifact(PATHS)


def test_generation_is_deterministic_and_matches_the_committed_artifact(artifact):
    again = gb.build_artifact(PATHS)
    assert json.dumps(artifact, sort_keys=True) == json.dumps(again, sort_keys=True)
    committed = json.loads(PATHS.artifact().read_text())
    assert committed == json.loads(json.dumps(artifact)), 'Run build_graph: committed artifact is stale'


def test_topology_from_the_authoritative_layout(artifact):
    nodes = {n['id']: n for n in artifact['nodes']}
    junctions = sorted(n for n, v in nodes.items() if v['type'] == 'INTERSECTION')
    verticals = ['aisle_01', 'aisle_02', 'aisle_03', 'east_service_aisle', 'west_service_aisle']
    assert junctions == sorted(f'{h}__{v}' for h in ('main_cross_aisle', 'north_cross_aisle', 'south_cross_aisle') for v in verticals)
    assert nodes['main_cross_aisle__aisle_01']['position'] == [-4.5, 0.0]
    assert nodes['north_cross_aisle__west_service_aisle']['position'] == [-8.7, 8.5]
    stations = {s['id']: s for s in artifact['stations']}
    assert set(stations) == {'pickup_station_01', 'pickup_station_02', 'dropoff_station_01', 'dropoff_station_02',
                             'charging_station', 'staging_area'}
    # staging_area docks at its layout goal_pose, not its visual centre; others at their marked dock.
    assert stations['staging_area']['dock_source'] == 'goal_pose' and nodes['staging_area']['position'] == [-10.2, -7.3]
    assert stations['staging_area']['marking']['centre'] == [-12.4, -8.3]
    assert nodes['pickup_station_01']['yaw'] == pytest.approx(math.pi)
    # The camera mast at (-10, -8.5) rejects the south-aisle access to staging; the west service access remains.
    assert stations['staging_area']['approach_nodes'] == ['staging_area_via_west_service_aisle']
    assert any('staging_area via south_cross_aisle' in n and '(-10, -8.5)' in n for n in artifact['structure']['notes'])
    assert stations['charging_station']['approach_nodes'] == ['charging_station_via_north_cross_aisle']
    assert artifact['structure']['errors'] == [] and artifact['structure']['warnings'] == []
    assert len(artifact['components']['topology']) == 1


def test_no_physically_impossible_connection(artifact):
    for e in artifact['edges']:
        assert e['status'] != 'invalid' or 'endpoint' in e['reason'], e
        if e['kind'] == 'corridor':
            a, b = (next(n for n in artifact['nodes'] if n['id'] == k)['position'] for k in (e['source'], e['target']))
            assert a[0] == b[0] or a[1] == b[1], 'corridor edges follow axis-aligned centrelines'
    # Rack rows separate aisles: no edge may join two parallel aisles directly.
    assert not any(e['corridor_id'] is None and e['kind'] == 'corridor' for e in artifact['edges'])


def test_restriction_zones_affect_only_the_physically_crossed_connections(artifact):
    zoned = {e['connection']: e['zones'] for e in artifact['edges'] if e['zones']}
    assert zoned == {'main_cross_aisle__aisle_01--south_cross_aisle__aisle_01': ['aisle_01_block']}
    zones = {z['id']: z for z in artifact['zones']}
    assert zones['aisle_01_block']['scenario'] == 'corridor_blocked' and zones['electrical_clearance']['enabled'] is False


def test_hand_computed_topology_route(artifact):
    transform = gb.registration.SE2(**artifact['registration']['transform'])
    settings = gb.load_settings(PATHS.file('planner_config'))
    for algorithm in ('astar', 'dijkstra'):
        route, _ = plan(artifact, transform, settings, 'pickup_station_01', 'dropoff_station_02', algorithm,
                        Restrictions(require_validated=False))
        # 2.2 dock access + 5.0 west service + 16.8 main aisle + 5.0 east service + 2.8 dock access.
        assert route['distance_m'] == pytest.approx(31.8)
        assert [w['node'] for w in route['waypoints']] == [
            'pickup_station_01_via_west_service_aisle', 'main_cross_aisle__west_service_aisle',
            'main_cross_aisle__east_service_aisle', 'dropoff_station_02_via_east_service_aisle', 'dropoff_station_02']
        assert route['waypoints'][-1]['world'][2] == 0.0
        assert route['validation_status'] == 'unvalidated_topology_only'
    closed = plan(artifact, transform, settings, 'pickup_station_01', 'dropoff_station_02', 'astar',
                  Restrictions(blocked_connections=frozenset({'main_cross_aisle__aisle_01--main_cross_aisle__west_service_aisle'}),
                               require_validated=False))[0]
    assert closed['found'] and 'main_cross_aisle__aisle_01' not in closed['selected_nodes'][:4]
    assert closed['distance_m'] > 31.8


def test_unverified_destinations_are_rejected(artifact):
    transform = gb.registration.SE2(**artifact['registration']['transform'])
    settings = gb.load_settings(PATHS.file('planner_config'))
    unavailable = [s['id'] for s in artifact['stations'] if s['status'] != 'validated']
    for station in unavailable:
        with pytest.raises(RoutingError) as err:
            plan(artifact, transform, settings, 'current', station, start_pose_map=(0., 0., 0.))
        assert err.value.code == 'location_unavailable'
    with pytest.raises(RoutingError) as err:
        plan(artifact, transform, settings, 'current', 'nowhere', start_pose_map=(0., 0., 0.))
    assert err.value.code == 'unknown_location'


def test_start_attachment_projection_off_graph_and_retreat(artifact):
    graph = Graph.from_dict(artifact)
    start, connector = attach_start(graph, artifact, (-6.0, .4, 0.), set(), .5)
    assert connector['kind'] == 'edge' and connector['offset_m'] == pytest.approx(.4)
    assert sorted(e.target for e in graph.outgoing(start)) == ['main_cross_aisle__aisle_01', 'main_cross_aisle__west_service_aisle']
    with pytest.raises(RoutingError) as err:
        attach_start(Graph.from_dict(artifact), artifact, (-6.6, 4.5, 0.), set(), .5)
    assert err.value.code == 'robot_off_graph'
    connection = 'main_cross_aisle__aisle_01--main_cross_aisle__west_service_aisle'
    closed = {e['id'] for e in artifact['edges'] if e['connection'] == connection}
    graph = Graph.from_dict(artifact)
    start, connector = attach_start(graph, artifact, (-6.0, .4, 0.), closed, .5,
                                    retreat=(connection, 'main_cross_aisle__west_service_aisle'))
    assert connector['kind'] == 'retreat'
    assert [e.target for e in graph.outgoing(start)] == ['main_cross_aisle__west_service_aisle']


def test_stale_artifact_and_registration_are_rejected(tmp_path):
    src = tmp_path/'src'
    for package in ('amr_simulation', 'amr_description', 'amr_navigation', 'amr_route_planner'):
        shutil.copytree(ROOT/'src'/package, src/package, ignore=shutil.ignore_patterns('validation', '__pycache__', 'test'))
    paths = gb.Paths(src)
    load_context(paths)
    planner = paths.file('planner_config')
    planner.write_text(planner.read_text().replace('turn_weight_m_per_rad: 1.2857', 'turn_weight_m_per_rad: 2.0'))
    with pytest.raises(gb.GraphStaleError, match='planner_config'):
        load_context(paths)


def test_costmap_blockage_evidence():
    from nav_msgs.msg import OccupancyGrid
    from amr_route_planner.route_executor import blocked_sections
    m = OccupancyGrid()
    m.info.resolution, m.info.width, m.info.height = .05, 200, 80
    data = [0]*(200*80)
    m.data = data
    assert blocked_sections(m, (1., 2.), (9., 2.), 3.) == []
    # A barrier spanning the full 3 m corridor width at x = 5 m (inscribed cost, 99).
    for r in range(80):
        for c in range(98, 102):
            data[r*200+c] = 99
    m.data = data
    sections = blocked_sections(m, (1., 2.), (9., 2.), 3.)
    assert sections and all(4.8 <= p[0] <= 5.2 for p in sections)
    # Leave a 0.5 m gap of low cost: the robot centre can pass, so no blockage.
    for r in range(50, 60):
        for c in range(98, 102):
            data[r*200+c] = 40
    m.data = data
    assert blocked_sections(m, (1., 2.), (9., 2.), 3.) == []


def test_executor_status_satisfies_the_gateway_contract():
    """Every executor message must pass the strict route-status v1 schema the Gateway enforces."""
    import types
    from jsonschema import Draft202012Validator, FormatChecker
    from amr_route_planner.route_executor import Executor
    schema = json.loads((ROOT/'shared/schemas/route-status.v1.schema.json').read_text())
    validator = Draft202012Validator({'$defs': schema['$defs'], '$ref': '#/$defs/RouteExecution'}, format_checker=FormatChecker())
    clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=12_500_000_000))
    executor = Executor.__new__(Executor)
    executor.node = types.SimpleNamespace(get_clock=lambda: clock)
    executor.args = types.SimpleNamespace(origin='current', destination='pickup_station_01', algorithm='astar')
    executor.report = {'replans': [{}]}
    route = {'algorithm': 'dijkstra', 'graph_version': 'g', 'map_version': 'm', 'registration_id': 'r', 'distance_m': 9.2,
             'selected_nodes': ['robot_start', 'a', 'b'], 'selected_edges': ['robot_start->a', 'a->b'],
             'map_frame_path': [[0., 0.], [1., 0.], [1., 3.]], 'waypoints': [{'node': 'a', 'map': [1., 0., 1.57]}, {'node': 'b', 'map': [1., 3., 1.57]}]}
    for state in [{'status': 'IDLE', 'route': None, 'route_id': None, 'completed': 0, 'remaining_m': None, 'action': None,
                   'failure': None, 'detail': None, 'blocked': []},
                  {'status': 'EXECUTING', 'route': route, 'route_id': 'abc', 'completed': 1, 'remaining_m': 4.0,
                   'action': 'EXECUTING', 'failure': None, 'detail': 'x', 'blocked': ['a--b']},
                  {'status': 'FAILED', 'route': route, 'route_id': 'abc', 'completed': 1, 'remaining_m': 99.,
                   'action': 'ABORTED', 'failure': 'navigation_failed', 'detail': 'e'*5000, 'blocked': []}]:
        executor.state = state
        message = executor.message()
        validator.validate(message)
        assert 0 <= message['execution_progress'] <= 1


def test_direct_connector_requires_a_map_check(artifact):
    world = (-11.0, 0.0, 0.0)   # active spawn
    validated = {n['id'] for n in artifact['nodes'] if n['status'] == 'validated'}
    closed = {e['id'] for e in artifact['edges'] if e['status'] != 'validated'} | {n['id'] for n in artifact['nodes'] if n['status'] != 'validated'}
    # Put the robot in no available lane on any map: the Phase 4 map left the spawn's lanes
    # unvalidated; the Phase 5.1 map validates them, so they are closed explicitly here.
    spawn = 'amr_01_spawn'
    closed |= {spawn} | {e['id'] for e in artifact['edges'] if spawn in (e['source'], e['target'])}
    if 'main_cross_aisle__west_service_aisle' not in validated:
        pytest.skip('west-service junction not validated on this map')
    with pytest.raises(RoutingError):
        attach_start(Graph.from_dict(artifact), artifact, world, closed, .5, connector_ok=lambda p: False, connector_max=3.)
    graph = Graph.from_dict(artifact)
    start, connector = attach_start(graph, artifact, world, closed, .5, connector_ok=lambda p: True, connector_max=3.)
    # The nearest validated, available node (west-service junction on the Phase 4 map, receiving on 5.1).
    offset, node = min((math.dist(world[:2], n['position']), n['id']) for n in artifact['nodes']
                       if n['id'] in validated and n['id'] not in closed)
    assert connector == {'kind': 'direct', 'node': node, 'offset_m': pytest.approx(offset)}
    assert [e.target for e in graph.outgoing(start)] == [node]
