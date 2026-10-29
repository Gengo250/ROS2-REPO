"""Graph route planning (Dijkstra, A*) and conversion into Nav2 map-frame objectives.

Search state is (node, incoming edge) so turn costs remain exact. Dijkstra is A*
with a zero heuristic; both expand states in a deterministic order. Planning never
commands the robot.
"""
from dataclasses import dataclass, field
import heapq
import json
import math
import time
import uuid
from . import coordinate_registration as registration
from . import map_validation as mv
from .cost_model import CostModel, Restrictions, exclusions, wrap
from .graph_model import Edge, Graph, Node, heading

ALGORITHMS = ('astar', 'dijkstra')
START = 'robot_start'
# Lane half-width for a direct start connector (plan conformance only; the map check governs).
connector_lane_half_width = 1.0


class RoutingError(ValueError):
    """Request rejected; ``code`` is a stable machine-readable reason."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


@dataclass
class SearchResult:
    algorithm: str
    found: bool
    nodes: list = field(default_factory=list)
    edges: list = field(default_factory=list)
    cost: float = math.inf
    distance: float = 0.
    expanded_states: int = 0
    visited_nodes: int = 0
    duration_s: float = 0.


def search(graph, start, goal, cost_model, algorithm='astar', excluded_edges=(), excluded_nodes=(),
           start_yaw=None, goal_yaw=None):
    if algorithm not in ALGORITHMS:
        raise RoutingError('invalid_algorithm', f'Unknown algorithm {algorithm}')
    for node in (start, goal):
        if node not in graph.nodes:
            raise RoutingError('unknown_location', f'Unknown graph node {node}')
    began = time.perf_counter()
    result = SearchResult(algorithm, False)
    excluded_edges, excluded_nodes = set(excluded_edges), set(excluded_nodes)
    if start in excluded_nodes or goal in excluded_nodes:
        result.duration_s = time.perf_counter()-began
        return result
    adjacency = graph.adjacency()
    headings = {e.id: heading(graph, e) for e in graph.edges.values()}
    target = graph.nodes[goal].position

    def h(node):
        return cost_model.lower_bound(graph.nodes[node].position, target) if algorithm == 'astar' else 0.
    best = {(start, ''): 0.}
    parent = {}
    queue = [(h(start), 0., start, '')]
    closed, seen = set(), {start}
    found = None
    while queue:
        _, g, node, incoming = heapq.heappop(queue)
        state = (node, incoming)
        if state in closed or g > best[state]+1e-12:
            continue
        closed.add(state)
        result.expanded_states += 1
        seen.add(node)
        if node == goal:
            found = state
            break
        h_in = headings[incoming] if incoming else start_yaw
        for edge in adjacency[node]:
            if edge.id in excluded_edges or edge.target in excluded_nodes:
                continue
            step = cost_model.edge(edge)+cost_model.turn(h_in, headings[edge.id])
            if edge.target == goal and goal_yaw is not None:
                step += cost_model.turn(headings[edge.id], goal_yaw)
            nxt, ng = (edge.target, edge.id), g+step
            if ng < best.get(nxt, math.inf)-1e-12:
                best[nxt], parent[nxt] = ng, state
                heapq.heappush(queue, (ng+h(edge.target), ng, edge.target, edge.id))
    result.visited_nodes = len(seen)
    if found:
        result.found, result.cost = True, best[found]
        state = found
        while state[1]:
            result.edges.append(state[1])
            result.nodes.append(state[0])
            state = parent[state]
        result.nodes.append(start)
        result.nodes.reverse()
        result.edges.reverse()
        result.distance = sum(graph.edges[e].distance for e in result.edges)
    result.duration_s = time.perf_counter()-began
    return result


def resolve(artifact, name):
    """Station id → its dock node; otherwise a graph node id."""
    stations = {s['id']: s for s in artifact['stations']}
    if name in stations:
        return stations[name]['dock_node'], stations[name]
    if any(n['id'] == name for n in artifact['nodes']):
        return name, None
    raise RoutingError('unknown_location', f'Unknown station or graph node: {name}')


def require_available(artifact, name, role, restrictions):
    node, station = resolve(artifact, name)
    if not restrictions.require_validated:
        return node
    if station is not None and station['status'] != 'validated':
        raise RoutingError('location_unavailable', f"{role} {name} is not a verified destination: {station['status']} ({station['reason']})")
    status = next(n for n in artifact['nodes'] if n['id'] == node)
    if status['status'] != 'validated':
        raise RoutingError('location_unavailable', f"{role} {name} is not validated: {status['status']} ({status['reason']})")
    return node


def lane(graph, edge):
    """World-frame corridor lane of an edge, including its junction squares."""
    a, b = graph.nodes[edge.source].position, graph.nodes[edge.target].position
    length = math.dist(a, b)
    return mv.Box((a[0]+b[0])/2, (a[1]+b[1])/2, math.atan2(b[1]-a[1], b[0]-a[0]), length/2+edge.width/2, edge.width/2)


def attach_start(graph, artifact, world_pose, excluded_edges, attach_radius, retreat=None, connector_ok=None,
                 connector_max=0.):
    """Add a virtual start node at the robot; return its id and the connector it used.

    ``retreat=(connection, node)`` lets a robot inside a just-closed connection leave
    it towards the node behind it, never forwards through the closure. When the robot is
    in no lane, a straight connector to a validated node within ``connector_max`` is used
    only if ``connector_ok(world_point)`` confirms it against the live map.
    """
    x, y, _ = world_pose
    candidates = sorted((math.dist((x, y), n.position), n.id) for n in graph.nodes.values()
                        if n.id not in excluded_edges and artifact_node(artifact, n.id)['status'] == 'validated')
    if candidates and candidates[0][0] <= attach_radius:
        return candidates[0][1], {'kind': 'node', 'node': candidates[0][1], 'offset_m': candidates[0][0]}
    best = None
    for edge in sorted(graph.edges.values(), key=lambda e: e.id):
        backwards = retreat is not None and (edge.connection, edge.target) == tuple(retreat)
        if edge.id in excluded_edges and not backwards:
            continue
        a, b = graph.nodes[edge.source].position, graph.nodes[edge.target].position
        length = math.dist(a, b)
        t = max(0., min(1., ((x-a[0])*(b[0]-a[0])+(y-a[1])*(b[1]-a[1]))/length**2))
        p = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
        offset = math.dist((x, y), p)
        if offset <= edge.width/2 and (best is None or (offset, edge.connection) < best[:2]):
            best = (offset, edge.connection, p, backwards)
    if best is None:
        for distance, node in candidates:
            if distance <= connector_max and connector_ok is not None and connector_ok(graph.nodes[node].position):
                graph.add_node(Node(START, 'CORRIDOR_POINT', (round(x, 6), round(y, 6))))
                graph.add_edge(Edge(f'{START}->{node}', START, node, f'{START}--{node}', 'corridor', None,
                                    round(distance, 6), round(distance, 6), 2*connector_lane_half_width, 'one_way'))
                return START, {'kind': 'direct', 'node': node, 'offset_m': distance}
        raise RoutingError('robot_off_graph', 'Robot is not inside any available route lane')
    offset, connection, p, backwards = best
    graph.add_node(Node(START, 'CORRIDOR_POINT', (round(p[0], 6), round(p[1], 6))))
    allowed = [e for e in graph.edges.values() if e.connection == connection and
               (e.target == retreat[1] if backwards else e.id not in excluded_edges)]
    for edge in sorted(allowed, key=lambda e: e.id):
        # Keep the original direction: a→b allows start→b only.
        part = math.dist(p, graph.nodes[edge.target].position)
        if part < 1e-6:
            continue
        ratio = part/edge.distance
        graph.add_edge(Edge(f'{START}->{edge.target}', START, edge.target, connection, edge.kind, edge.corridor_id,
                            round(part, 6), round(edge.base_cost*ratio, 6), edge.width, 'one_way', edge.zones))
    return START, {'kind': 'retreat' if backwards else 'edge', 'connection': connection, 'projection': list(p),
                   'offset_m': offset}


def artifact_node(artifact, node_id):
    return next(n for n in artifact['nodes'] if n['id'] == node_id)


def objectives(graph, nodes, edges, threshold, transform):
    """Minimum Nav2 objective sequence: corridor changes, turns, and the destination pose."""
    chosen = []
    for i in range(1, len(nodes)):
        node = graph.nodes[nodes[i]]
        incoming = graph.edges[edges[i-1]]
        h_in = heading(graph, incoming)
        last = i == len(nodes)-1
        if last:
            yaw, reason = (node.yaw if node.yaw is not None else h_in), 'destination'
        else:
            outgoing = graph.edges[edges[i]]
            h_out = heading(graph, outgoing)
            turn = abs(wrap(h_out-h_in))
            changes = (incoming.corridor_id, incoming.kind) != (outgoing.corridor_id, outgoing.kind)
            if turn <= threshold and not changes:
                continue
            yaw, reason = h_out, ('turn' if turn > threshold else 'corridor_change')
        mx, my, myaw = transform.apply_pose(*node.position, yaw)
        chosen.append({'node': node.id, 'reason': reason, 'world': [node.position[0], node.position[1], yaw],
                       'map': [round(mx, 4), round(my, 4), round(myaw, 6)], 'route_index': i})
    return chosen


def plan(artifact, transform, settings, origin, destination, algorithm='astar', restrictions=Restrictions(),
         start_pose_map=None, route_id=None, retreat=None, connector_ok=None):
    """Return a structured route plan or raise RoutingError. ``origin='current'`` needs the robot map pose."""
    graph = Graph.from_dict(artifact)
    excluded_edges, excluded_nodes = exclusions(artifact, restrictions)
    goal = require_available(artifact, destination, 'Destination', restrictions)
    start_yaw = None
    connector = None
    if origin == 'current':
        if start_pose_map is None:
            raise RoutingError('localization_unavailable', 'Current robot pose required for origin=current')
        world = registration_inverse(transform, start_pose_map)
        start, connector = attach_start(graph, artifact, world, set(excluded_edges) | set(excluded_nodes),
                                        settings['executor']['attach_radius_m'], retreat, connector_ok,
                                        settings['executor']['max_connector_m'])
        start_yaw = world[2]
    else:
        start = require_available(artifact, origin, 'Origin', restrictions)
        start_yaw = graph.nodes[start].yaw
    if start == goal:
        raise RoutingError('trivial_route', 'Origin and destination are the same location')
    cost_model = CostModel.from_settings(settings)
    result = search(graph, start, goal, cost_model, algorithm, excluded_edges, excluded_nodes,
                    start_yaw=start_yaw, goal_yaw=graph.nodes[goal].yaw)
    report = {
        'route_id': route_id or uuid.uuid4().hex[:12], 'origin': origin, 'destination': destination,
        'algorithm': algorithm, 'graph_version': artifact['graph_version'], 'map_version': artifact['map']['version'],
        'registration_id': artifact['registration']['id'], 'start_node': start, 'goal_node': goal,
        'start_connector': connector, 'found': result.found,
        'expanded_states': result.expanded_states, 'visited_nodes': result.visited_nodes,
        'planning_time_s': result.duration_s,
        'restrictions': {'scenario': restrictions.scenario, 'blocked_connections': sorted(restrictions.blocked_connections),
                         'excluded_edges': len(excluded_edges), 'excluded_nodes': len(excluded_nodes)},
    }
    if not result.found:
        report.update(validation_status='no_route')
        return report, graph
    report.update({
        'selected_nodes': result.nodes, 'selected_edges': result.edges,
        'connections': [graph.edges[e].connection for e in result.edges],
        'edge_widths': [graph.edges[e].width for e in result.edges],
        'distance_m': round(result.distance, 4), 'cost': round(result.cost, 4),
        'path_world': [list(graph.nodes[n].position) for n in result.nodes],
        'map_frame_path': [[round(v, 4) for v in transform.apply(*graph.nodes[n].position)] for n in result.nodes],
        'waypoints': objectives(graph, result.nodes, result.edges, settings['waypoints']['turn_threshold_rad'], transform),
        'validation_status': 'validated' if restrictions.require_validated else 'unvalidated_topology_only',
    })
    return report, graph


def registration_inverse(transform, pose):
    x, y, yaw = pose
    return transform.inverse().apply_pose(x, y, yaw)


def route_lanes(graph, edge_ids):
    return [lane(graph, graph.edges[e]) for e in edge_ids]


def inside_lanes(lanes, point, tolerance=0.):
    return any(bool(box.contains(point[0], point[1], tolerance)) for box in lanes)


def load_context(paths):
    """Artifact, verified registration and settings; every source must still match."""
    from . import graph_builder as gb
    artifact = gb.load_artifact(paths)
    record = json.loads(paths.file('registration').read_text())
    transform = registration.evaluate(record, artifact['map']['version'], artifact['sources']['layout']['sha256'])
    if record['id'] != artifact['registration']['id']:
        raise registration.RegistrationError('Graph artifact was validated with a different registration')
    return artifact, transform, gb.load_settings(paths.file('planner_config'))
