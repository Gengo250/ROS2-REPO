"""Read-only Phase 5 route graph adapter; the route planner owns planning and execution."""
import json
from pathlib import Path
from ..state.models import RouteGraph


def graph_path(configured=None):
    if configured:
        return Path(configured)
    # Optional extension: absent when the route planner package is not installed.
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        return Path(get_package_share_directory('amr_route_planner'))/'config/warehouse_graph.json'
    except PackageNotFoundError as exc:
        raise FileNotFoundError('amr_route_planner is not installed') from exc


def load_route_graph(path):
    """Project the versioned artifact onto the browser contract (world frame, one entry per connection)."""
    artifact = json.loads(Path(path).read_text())
    if artifact.get('schema_version') != 1 or artifact.get('frame_id') != 'world':
        raise ValueError('Unsupported route graph artifact')
    connections = {}
    for e in artifact['edges']:
        entry = connections.setdefault(e['connection'], {
            'id': e['connection'], 'source': e['source'], 'target': e['target'], 'directions': [],
            'kind': e['kind'], 'corridor_id': e['corridor_id'], 'width': e['width'], 'distance': e['distance'],
            'status': e['status'], 'zones': e['zones']})
        entry['directions'].append(e['id'])
    reg = artifact['registration']
    return RouteGraph(
        graph_version=artifact['graph_version'], map_version=artifact['map']['version'], frame_id='world',
        registration={'id': reg['id'], 'parent_frame': reg['parent_frame'], 'child_frame': reg['child_frame'],
                      **reg['transform'], 'valid': reg['valid']},
        nodes=[{'id': n['id'], 'type': n['type'], 'position': n['position'], 'yaw': n.get('yaw'),
                'status': n['status'], 'reason': n['reason']} for n in artifact['nodes']],
        connections=[{**c, 'directions': sorted(c['directions'])} for _, c in sorted(connections.items())],
        stations=[{'id': s['id'], 'label': s['label'], 'dock_node': s['dock_node'], 'status': s['status'],
                   'reason': s['reason']} for s in artifact['stations']])
