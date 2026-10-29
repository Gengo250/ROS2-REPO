"""Directed warehouse navigation graph: stable identifiers, explicit directions.

A physical connection appears as one directed edge per permitted direction; both
share a ``connection`` id so a corridor closure removes every direction at once.
Positions are in the graph frame (``world``); map coordinates are always derived
through a verified registration, never stored as an independent truth.
"""
from dataclasses import dataclass, field, asdict
import hashlib
import json
import math

NODE_TYPES = ('STATION', 'INTERSECTION', 'CORRIDOR_POINT', 'OPEN_AREA')
EDGE_KINDS = ('corridor', 'station_access')
MAP_STATUSES = ('validated', 'unmapped', 'blocked', 'invalid')


class GraphError(ValueError):
    """Structural graph defect; the graph must not be used for routing."""


@dataclass(frozen=True)
class Node:
    id: str
    type: str
    position: tuple
    frame_id: str = 'world'
    yaw: float | None = None
    entity: str | None = None
    corridors: tuple = ()


@dataclass(frozen=True)
class Edge:
    id: str
    source: str
    target: str
    connection: str
    kind: str
    corridor_id: str | None
    distance: float
    base_cost: float
    width: float
    direction: str = 'bidirectional'
    zones: tuple = ()


@dataclass
class Graph:
    nodes: dict = field(default_factory=dict)
    edges: dict = field(default_factory=dict)
    frame_id: str = 'world'

    def add_node(self, node):
        if node.id in self.nodes:
            raise GraphError('Duplicate node id: '+node.id)
        self.nodes[node.id] = node

    def add_edge(self, edge):
        if edge.id in self.edges:
            raise GraphError('Duplicate edge id: '+edge.id)
        self.edges[edge.id] = edge

    def outgoing(self, node_id):
        # Sorted so every search sees neighbours in a reproducible order.
        return [e for _, e in sorted((e.id, e) for e in self.edges.values() if e.source == node_id)]

    def adjacency(self):
        table = {n: [] for n in self.nodes}
        for edge_id in sorted(self.edges):
            edge = self.edges[edge_id]
            table[edge.source].append(edge)
        return table

    def connections(self):
        return sorted({e.connection for e in self.edges.values()})

    def to_dict(self):
        return {'frame_id': self.frame_id,
                'nodes': [_node_dict(self.nodes[k]) for k in sorted(self.nodes)],
                'edges': [_edge_dict(self.edges[k]) for k in sorted(self.edges)]}

    @classmethod
    def from_dict(cls, data):
        graph = cls(frame_id=data['frame_id'])
        for n in data['nodes']:
            graph.add_node(Node(id=n['id'], type=n['type'], position=tuple(n['position']), frame_id=n['frame_id'],
                                yaw=n.get('yaw'), entity=n.get('entity'), corridors=tuple(n.get('corridors', ()))))
        for e in data['edges']:
            graph.add_edge(Edge(id=e['id'], source=e['source'], target=e['target'], connection=e['connection'],
                                kind=e['kind'], corridor_id=e['corridor_id'], distance=e['distance'],
                                base_cost=e['base_cost'], width=e['width'], direction=e['direction'],
                                zones=tuple(e.get('zones', ()))))
        return graph

    def topology_version(self):
        return canonical_hash(self.to_dict())


def _node_dict(node):
    data = asdict(node)
    data['position'] = [round(v, 6) for v in node.position]
    data['corridors'] = list(node.corridors)
    if node.yaw is not None:
        data['yaw'] = round(node.yaw, 9)
    return data


def _edge_dict(edge):
    data = asdict(edge)
    for key in ('distance', 'base_cost', 'width'):
        data[key] = round(data[key], 6)
    data['zones'] = list(edge.zones)
    return data


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def heading(graph, edge):
    (x0, y0), (x1, y1) = graph.nodes[edge.source].position, graph.nodes[edge.target].position
    return math.atan2(y1-y0, x1-x0)


def components(graph, edges=None):
    """Weakly connected components over the given edge subset, deterministic order."""
    edges = graph.edges.values() if edges is None else edges
    parent = {n: n for n in graph.nodes}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n
    for e in edges:
        if e.source not in parent or e.target not in parent:
            continue  # Dangling edges are reported by validate_structure, never joined.
        a, b = find(e.source), find(e.target)
        if a != b:
            parent[max(a, b)] = min(a, b)
    groups = {}
    for n in sorted(graph.nodes):
        groups.setdefault(find(n), []).append(n)
    return sorted(groups.values(), key=lambda g: (-len(g), g[0]))


def validate_structure(graph, stations=()):
    """Return errors (graph unusable) and warnings (reportable conditions)."""
    errors, warnings = [], []
    if graph.frame_id != 'world':
        errors.append('Graph frame must be world, got '+str(graph.frame_id))
    for n in graph.nodes.values():
        if n.type not in NODE_TYPES:
            errors.append(f'{n.id}: unknown node type {n.type}')
        if n.frame_id != graph.frame_id:
            errors.append(f'{n.id}: frame {n.frame_id} differs from graph frame')
        if len(n.position) != 2 or not all(math.isfinite(v) for v in n.position):
            errors.append(f'{n.id}: invalid position')
        if n.yaw is not None and not math.isfinite(n.yaw):
            errors.append(f'{n.id}: invalid yaw')
    pairs = {}
    for e in graph.edges.values():
        if e.source not in graph.nodes or e.target not in graph.nodes:
            errors.append(f'{e.id}: endpoint missing')
            continue
        if e.source == e.target:
            errors.append(f'{e.id}: self-loop')
        if (e.source, e.target) in pairs:
            errors.append(f'{e.id}: duplicates {pairs[(e.source, e.target)]}')
        pairs[(e.source, e.target)] = e.id
        if e.kind not in EDGE_KINDS:
            errors.append(f'{e.id}: unknown kind {e.kind}')
        if not all(math.isfinite(v) and v >= 0 for v in (e.distance, e.base_cost, e.width)):
            errors.append(f'{e.id}: negative or non-finite distance/cost/width')
        geometric = math.dist(graph.nodes[e.source].position, graph.nodes[e.target].position)
        if abs(geometric-e.distance) > 1e-3:
            errors.append(f'{e.id}: stored distance differs from node geometry')
        if e.base_cost < e.distance-1e-9:
            errors.append(f'{e.id}: base cost below distance breaks the A* lower bound')
    for e in graph.edges.values():
        reverse = pairs.get((e.target, e.source))
        if e.direction == 'bidirectional':
            other = graph.edges.get(reverse) if reverse else None
            if other is None or other.connection != e.connection or other.direction != 'bidirectional':
                errors.append(f'{e.id}: bidirectional edge lacks a matching reverse edge')
        elif e.direction == 'one_way':
            if reverse is not None:
                errors.append(f'{e.id}: one-way connection also has a reverse edge')
        else:
            errors.append(f'{e.id}: unknown direction {e.direction}')
    for station in stations:
        dock = graph.nodes.get(station['dock_node'])
        if dock is None or dock.type != 'STATION' or dock.entity != station['id']:
            errors.append(f"{station['id']}: dock node missing or not a STATION")
            continue
        if not any(e.kind == 'station_access' and dock.id in (e.source, e.target) for e in graph.edges.values()):
            errors.append(f"{station['id']}: no station access edge")
    groups = components(graph)
    if len(groups) > 1:
        warnings.append('Disconnected topology components: '+'; '.join(','.join(g) for g in groups[1:]))
    for n in graph.nodes.values():
        if not any(n.id in (e.source, e.target) for e in graph.edges.values()):
            (errors if n.type == 'STATION' else warnings).append(f'{n.id}: isolated node')
    return errors, warnings
