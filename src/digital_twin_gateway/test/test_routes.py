"""Phase 5 route extension: separate versioned contracts, strict input, unchanged twin snapshots."""
import json
from pathlib import Path
import pytest
from fastapi.testclient import TestClient
from digital_twin_gateway.api.app import create_app
from digital_twin_gateway.config import Settings
from digital_twin_gateway.routes import load_route_graph
from conftest import Runtime
from test_state import assert_schema

ROOT = Path(__file__).resolve().parents[3]
ARTIFACT = ROOT/'src/amr_route_planner/config/warehouse_graph.json'


def message(**changes):
    value = {'protocol_version': '1.0', 'message_type': 'route_status', 'route_id': 'abc123', 'status': 'EXECUTING',
             'origin': 'current', 'destination': 'dropoff_station_02', 'algorithm': 'astar', 'graph_version': 'g1',
             'map_version': 'm1', 'registration_id': 'r1', 'selected_nodes': ['robot_start', 'a', 'b'],
             'selected_edges': ['robot_start->a', 'a->b'], 'map_frame_path': [[0, 0], [1, 0], [1, 2]],
             'map_frame_waypoints': [[1, 0, 1.57], [1, 2, 1.57]], 'waypoint_nodes': ['a', 'b'],
             'current_waypoint': 0, 'completed_waypoints': 0, 'total_waypoints': 2, 'estimated_distance_m': 3.0,
             'distance_remaining_m': 2.5, 'execution_progress': 0.1667, 'replans': 0, 'blocked_connections': [],
             'navigation_action_status': 'EXECUTING', 'failure_reason': None, 'detail': None, 'stamp': 12.5,
             'updated_at': '2026-10-29T12:00:00+00:00'}
    value.update(changes)
    return json.dumps(value)


def test_route_status_is_versioned_strict_and_ages(state):
    manager, timer = state
    before_v1, before_v2 = manager.publish().model_dump(), manager.snapshot(2).model_dump()
    empty = manager.route_snapshot()
    assert empty.route is None and empty.message_type == 'route_status_snapshot'
    manager.update_route(message())
    snapshot = manager.route_snapshot()
    assert snapshot.availability == 'online' and snapshot.route.status == 'EXECUTING'
    assert_schema(snapshot.model_dump(mode='json'), 'route-status.v1')
    timer.value += manager.settings.stale_seconds+.1
    assert manager.route_snapshot().availability == 'stale'
    for bad in [message(protocol_version='2.0'), message(status='TELEPORTING'), message(execution_progress=1.5),
                message(extra_field=1), '{not json', 'x'*262145]:
        with pytest.raises(ValueError):
            manager.update_route(bad)
    assert manager.route_snapshot().route.route_id == 'abc123'
    # Route data never enters the twin v1/v2 snapshots.
    assert manager.snapshot().model_dump() == before_v1 and manager.snapshot(2).model_dump() == before_v2


def test_route_endpoints_and_unavailable_graph(tmp_path):
    missing = create_app(Settings(route_graph_path=str(tmp_path/'missing.json')), Runtime)
    with TestClient(missing) as client:
        assert client.get('/api/v1/routes/graph').status_code == 503
        status = client.get('/api/v1/routes/status')
        assert status.status_code == 200 and status.json()['route'] is None
        assert_schema(status.json(), 'route-status.v1')
        assert client.post('/api/v1/routes/status').status_code == 405


@pytest.mark.skipif(not ARTIFACT.is_file(), reason='Route graph artifact not generated')
def test_route_graph_projects_the_artifact_once_per_connection():
    graph = load_route_graph(ARTIFACT)
    artifact = json.loads(ARTIFACT.read_text())
    assert graph.graph_version == artifact['graph_version'] and graph.frame_id == 'world'
    assert len(graph.connections) == len({e['connection'] for e in artifact['edges']})
    assert sum(len(c.directions) for c in graph.connections) == len(artifact['edges'])
    assert graph.registration.id == artifact['registration']['id']
    assert_schema(graph.model_dump(mode='json'), 'route-graph.v1')
    app = create_app(Settings(route_graph_path=str(ARTIFACT)), Runtime)
    with TestClient(app) as client:
        response = client.get('/api/v1/routes/graph')
        assert response.status_code == 200 and response.json()['graph_version'] == graph.graph_version
