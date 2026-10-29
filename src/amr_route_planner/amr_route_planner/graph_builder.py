"""Deterministic warehouse graph generation from the authoritative layout.

Inputs are the existing warehouse compiler's validated layout, its compiled
building walls, the approved SLAM map, the verified registration and the Nav2
footprint. The same inputs always produce the same identifiers and artifact.
Layout geometry proposes candidate connections; only real map observations can
validate them.
"""
from dataclasses import dataclass
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from . import coordinate_registration as registration
from . import map_validation as mv
from .graph_model import Edge, Graph, Node, canonical_hash, components, validate_structure

ARTIFACT_SCHEMA = 1
SOURCE_FILES = {
    'layout': ('amr_simulation', 'config/warehouse_layout.json'),
    'compiler': ('amr_simulation', 'scripts/generate_warehouse.py'),
    'generated_manifest': ('amr_simulation', 'config/generated_manifest.json'),
    'building_model': ('amr_simulation', 'models/wh_building/model.sdf'),
    'amr_dimensions': ('amr_description', 'config/amr_dimensions.json'),
    'map_yaml': ('amr_navigation', 'maps/warehouse.yaml'),
    'map_image': ('amr_navigation', 'maps/warehouse.pgm'),
    'map_provenance': ('amr_navigation', 'maps/provenance.json'),
    'nav2_params': ('amr_navigation', 'config/nav2_params.yaml'),
    'planner_config': ('amr_route_planner', 'config/route_planner.yaml'),
    'registration': ('amr_route_planner', 'config/map_registration.json'),
}


class GraphStaleError(ValueError):
    """The artifact does not match the installed layout, map, registration or config."""


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Paths:
    """Resolve package files from the ament index, or from a source tree root (``src``).

    ``overrides`` evaluates a candidate map/registration without touching approved files.
    """

    def __init__(self, source_root=None, overrides=None):
        self.source_root = Path(source_root) if source_root else None
        self.overrides = {k: Path(v) for k, v in (overrides or {}).items() if v}
        if 'map_yaml' in self.overrides:
            meta = yaml.safe_load(self.overrides['map_yaml'].read_text())
            self.overrides.setdefault('map_image', self.overrides['map_yaml'].parent/meta['image'])

    def package(self, name):
        if self.source_root:
            return self.source_root/name
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(name))

    def file(self, key):
        if key in self.overrides:
            return self.overrides[key]
        package, relative = SOURCE_FILES[key]
        return self.package(package)/relative

    def artifact(self):
        return self.package('amr_route_planner')/'config/warehouse_graph.json'

    @classmethod
    def source_tree(cls):
        return cls(Path(__file__).resolve().parents[2])


def load_settings(path):
    settings = yaml.safe_load(Path(path).read_text())
    cost = settings['cost']
    for key in ('distance_weight', 'turn_weight_m_per_rad', 'station_access_penalty_m'):
        if not math.isfinite(cost[key]) or cost[key] < 0:
            raise ValueError(f'cost.{key} must be finite and nonnegative')
    if cost['distance_weight'] <= 0:
        raise ValueError('cost.distance_weight must be positive')
    if any(not math.isfinite(v) or v < 0 for v in cost['corridor_penalties_m'].values()):
        raise ValueError('corridor penalties must be finite and nonnegative')
    return settings


def load_layout(paths):
    """Reuse the warehouse compiler: identical validation and static-geometry definition."""
    spec = importlib.util.spec_from_file_location('warehouse_compiler', paths.file('compiler'))
    compiler = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compiler)
    config, amr = compiler.load_config(paths.file('layout'), paths.file('amr_dimensions'))
    if config['coordinates'].get('frame') != 'world' or config['coordinates'].get('units') != 'metres_radians':
        raise ValueError('Unsupported layout coordinate convention')
    manifest = json.loads(paths.file('generated_manifest').read_text())
    if digest(paths.file('layout')) != manifest['source_sha256']['config/warehouse_layout.json']:
        raise GraphStaleError('Compiled warehouse is stale relative to warehouse_layout.json')
    if digest(paths.file('building_model')) != manifest['files']['models/wh_building/model.sdf']:
        raise GraphStaleError('Compiled building model does not match its manifest')
    return config, amr, compiler


def wall_bounds(building_sdf):
    """Axis-aligned wall collision boxes from the compiled building (z ignored)."""
    walls = []
    for collision in ET.parse(building_sdf).getroot().iter('collision'):
        name = collision.get('name', '')
        if '_wall_' not in name:
            continue
        x, y, _, roll, pitch, yaw = (float(v) for v in collision.findtext('pose').split())
        sx, sy, _ = (float(v) for v in collision.findtext('geometry/box/size').split())
        if abs(roll)+abs(pitch)+abs(yaw) > 1e-9:
            raise ValueError('Rotated wall collisions are not supported: '+name)
        walls.append((x, y, sx, sy))
    if not walls:
        raise ValueError('No wall collisions in the compiled building model')
    return sorted(walls)


@dataclass(frozen=True)
class Corridor:
    id: str
    cx: float
    cy: float
    sx: float
    sy: float
    capacity: int

    @property
    def axis(self):
        return 0 if self.sx > self.sy else 1

    @property
    def width(self):
        return min(self.sx, self.sy)

    def extent(self):
        """Centreline interval along the long axis."""
        c, size = (self.cx, self.sx) if self.axis == 0 else (self.cy, self.sy)
        return c-size/2, c+size/2

    def point(self, s):
        return (s, self.cy) if self.axis == 0 else (self.cx, s)

    def along(self, p):
        return p[self.axis]

    def offset(self, p):
        return abs(p[1]-self.cy) if self.axis == 0 else abs(p[0]-self.cx)

    def interval(self, axis):
        c, size = (self.cx, self.sx) if axis == 0 else (self.cy, self.sy)
        return c-size/2, c+size/2


def corridors_from(config):
    result = []
    for a in config['navigation']['aisles']:
        sx, sy = a['size']
        if abs(sx-sy) < 1e-9:
            raise ValueError('Square aisle has no long axis: '+a['id'])
        result.append(Corridor(a['id'], *a['centre'], sx, sy, a['capacity']))
    return sorted(result, key=lambda c: c.id)


class Builder:
    def __init__(self, config, compiler, footprint, settings, walls):
        self.config, self.footprint, self.settings = config, footprint, settings
        self.clearance = settings['graph']['required_clearance_m']
        self.merge = settings['graph']['node_merge_tolerance_m']
        self.max_access = settings['graph'].get('max_access_length_m', 3.0)
        self.obstacles = sorted(compiler.static_bounds(config)) + list(walls)
        self.corridors = corridors_from(config)
        self.graph = Graph()
        self.on_corridor = {c.id: {} for c in self.corridors}
        self.stations, self.notes, self.layout_status = [], [], {}
        lateral = config['navigation']['lateral_clearance']
        if abs(lateral-self.clearance) > 1e-9:
            raise ValueError('required_clearance_m must equal layout navigation.lateral_clearance')
        # Opening needed for the padded robot plus clearance on both sides.
        self.passage = 2*(footprint.half_width+self.clearance)

    def layout_conflict(self, core, full):
        for rect in self.obstacles:
            if any(mv.shape_hits_rect(s, rect) for s in core):
                return 'invalid', f'robot footprint intersects static layout geometry at {rect[:2]}'
        for rect in self.obstacles:
            if any(mv.shape_hits_rect(s, rect) for s in full):
                return 'invalid', f'required clearance intersects static layout geometry at {rect[:2]}'
        return None

    def attach(self, corridor, point, node_id, node_type, entity=None):
        """Place a node on a corridor centreline, merging with an existing nearby point."""
        s = corridor.along(point)
        for existing, other in self.on_corridor[corridor.id].items():
            if abs(other-s) < self.merge:
                return existing
        if node_id not in self.graph.nodes:
            self.graph.add_node(Node(node_id, node_type, tuple(round(v, 6) for v in point), entity=entity))
        self.on_corridor[corridor.id][node_id] = s
        return node_id

    def junctions(self):
        horizontal = [c for c in self.corridors if c.axis == 0]
        vertical = [c for c in self.corridors if c.axis == 1]
        for h in horizontal:
            for v in vertical:
                hx0, hx1 = h.interval(0)
                vx0, vx1 = v.interval(0)
                gap = max(h.interval(1)[0]-v.interval(1)[1], v.interval(1)[0]-h.interval(1)[1])
                overlap = min(hx1, vx1)-max(hx0, vx0)
                if gap > 1e-6 or overlap <= 0:
                    continue
                if overlap < self.passage or not hx0+self.passage/2 <= v.cx <= hx1-self.passage/2:
                    self.notes.append(f'{h.id}/{v.id}: touching but opening {overlap:.2f} m < {self.passage:.2f} m; not connected')
                    continue
                node = f'{h.id}__{v.id}'
                self.graph.add_node(Node(node, 'INTERSECTION', (round(v.cx, 6), round(h.cy, 6))))
                self.on_corridor[h.id][node] = v.cx
                self.on_corridor[v.id][node] = h.cy
        for i, a in enumerate(self.corridors):
            for b in self.corridors[i+1:]:
                if a.axis == b.axis and all(min(a.interval(k)[1], b.interval(k)[1]) > max(a.interval(k)[0], b.interval(k)[0]) for k in (0, 1)):
                    self.notes.append(f'{a.id}/{b.id}: parallel overlap is not modelled; not connected')

    def on_centreline(self, point):
        for c in self.corridors:
            lo, hi = c.extent()
            if c.offset(point) < 1e-6 and lo <= c.along(point) <= hi:
                return c
        return None

    def area_nodes(self):
        for area in sorted(self.config['navigation']['open_areas'], key=lambda a: a['id']):
            corridor = self.on_centreline(tuple(area['centre']))
            if corridor is None:
                self.notes.append(f"{area['id']}: open area centre is not on a corridor centreline; not connected")
                continue
            self.attach(corridor, tuple(area['centre']), area['id'], 'OPEN_AREA', area['id'])
        for spawn in sorted(self.config['spawns'], key=lambda s: s['id']):
            if not spawn['active']:
                continue
            corridor = self.on_centreline(tuple(spawn['pose'][:2]))
            if corridor is None:
                self.notes.append(f"{spawn['id']}: spawn not on a corridor centreline; not connected")
                continue
            self.attach(corridor, tuple(spawn['pose'][:2]), spawn['id'], 'CORRIDOR_POINT', spawn['id'])

    def station_nodes(self):
        for station in sorted(self.config['stations'], key=lambda s: s['id']):
            source = 'goal_pose' if 'goal_pose' in station else 'pose'
            x, y, *_, yaw = station[source]
            dock = (float(x), float(y))
            self.graph.add_node(Node(station['id'], 'STATION', dock, yaw=float(yaw), entity=station['id']))
            candidates = []
            for c in self.corridors:
                lo, hi = c.extent()
                projection = c.point(min(max(c.along(dock), lo), hi))
                length = math.dist(projection, dock)
                if length < 1e-6:
                    continue
                core, full = mv.edge_shapes(projection, dock, self.footprint, self.clearance)
                conflict = self.layout_conflict(core, full)
                if conflict:
                    self.notes.append(f"{station['id']} via {c.id}: rejected access candidate ({conflict[1]})")
                    continue
                candidates.append((round(length, 6), c.id, c, projection))
            candidates.sort(key=lambda item: item[:2])
            chosen = [item for item in candidates if item[0] <= self.max_access] or candidates[:1]
            if not chosen:
                self.notes.append(f"{station['id']}: no collision-free access from any corridor")
            approaches = []
            for length, _, corridor, projection in chosen:
                approach = self.attach(corridor, projection, f"{station['id']}_via_{corridor.id}", 'CORRIDOR_POINT')
                self.access_edge(approach, station['id'], min(station['size']))
                approaches.append(approach)
            self.stations.append({'id': station['id'], 'label': station['label'], 'dock_node': station['id'],
                                  'dock_source': source, 'approach_nodes': approaches,
                                  'marking': {'centre': list(station['pose'][:2]), 'yaw': station['pose'][5],
                                              'size': list(station['size'])}})

    def access_edge(self, approach, dock, width):
        self.connect(approach, dock, 'station_access', None, width,
                     self.settings['cost']['station_access_penalty_m'])

    def connect(self, a, b, kind, corridor_id, width, penalty):
        pa, pb = self.graph.nodes[a].position, self.graph.nodes[b].position
        distance = round(math.dist(pa, pb), 6)
        connection = '--'.join(sorted((a, b)))
        one_way = {tuple(p) for p in self.settings['cost']['one_way']}
        directions = [(a, b), (b, a)]
        if (a, b) in one_way or (b, a) in one_way:
            directions = [d for d in directions if d in one_way]
        core, _ = mv.edge_shapes(pa, pb, self.footprint, self.clearance)
        zones = sorted(z['id'] for z in self.zones() if any(mv.shape_hits_rect(s, (*z['centre'], *z['size'])) for s in core))
        for source, target in directions:
            self.graph.add_edge(Edge(f'{source}->{target}', source, target, connection, kind, corridor_id, distance,
                                     round(self.settings['cost']['distance_weight']*distance+penalty, 6), width,
                                     'bidirectional' if len(directions) == 2 else 'one_way', tuple(zones)))

    def zones(self):
        nav = self.config['navigation']
        return ([{**z, 'kind': 'restricted'} for z in nav['restricted_zones']] +
                [{**z, 'kind': 'temporary_blocked'} for z in nav['temporary_blocked_zones']])

    def corridor_edges(self):
        penalties = self.settings['cost']['corridor_penalties_m']
        for c in self.corridors:
            ordered = sorted(self.on_corridor[c.id].items(), key=lambda item: (item[1], item[0]))
            if len(ordered) < 2:
                self.notes.append(f'{c.id}: fewer than two graph points; corridor not represented')
            for (a, _), (b, _) in zip(ordered, ordered[1:]):
                self.connect(a, b, 'corridor', c.id, c.width, penalties.get(c.id, 0.))
        for node in list(self.graph.nodes.values()):
            on = tuple(sorted(cid for cid, members in self.on_corridor.items() if node.id in members))
            self.graph.nodes[node.id] = Node(node.id, node.type, node.position, node.frame_id, node.yaw, node.entity, on)

    def layout_validation(self):
        for node in self.graph.nodes.values():
            core, full = mv.node_shapes(node.position, self.footprint, self.clearance)
            self.layout_status[node.id] = self.layout_conflict(core, full)
        for edge in self.graph.edges.values():
            if edge.connection in self.layout_status:
                continue
            pa, pb = self.graph.nodes[edge.source].position, self.graph.nodes[edge.target].position
            self.layout_status[edge.connection] = self.layout_conflict(*mv.edge_shapes(pa, pb, self.footprint, self.clearance))

    def build(self):
        self.junctions()
        self.area_nodes()
        self.station_nodes()
        self.corridor_edges()
        self.layout_validation()
        return self.graph


def map_validation(builder, occupancy, transform, settings):
    """Classify every node and connection in the map frame; layout conflicts dominate."""
    graph, fp, clearance = builder.graph, builder.footprint, builder.clearance
    reg = settings['registration']
    points = occupancy.occupied_points()
    point_residuals = registration.residuals(transform, points, builder.obstacles) if len(points) else np.array([])
    nodes, connections = {}, {}
    for node in sorted(graph.nodes.values(), key=lambda n: n.id):
        mx, my = transform.apply(*node.position)
        status, reason, detail = mv.classify(occupancy, *mv.node_shapes((mx, my), fp, clearance))
        near = np.hypot(points[:, 0]-mx, points[:, 1]-my) <= reg['local_radius_m'] if len(points) else np.array([], bool)
        local = float(np.median(point_residuals[near])) if near.sum() >= reg['local_min_cells'] else None
        if local is not None and local > reg['local_max_median_m'] and status != 'unmapped':
            status, reason = 'invalid', f'local registration residual {local:.3f} m exceeds {reg["local_max_median_m"]} m'
        if builder.layout_status[node.id]:
            status, reason = builder.layout_status[node.id]
        entry = {'status': status, 'reason': reason, 'cells': detail,
                 'map_position': [round(mx, 4), round(my, 4)],
                 'registration_residual_m': None if local is None else round(local, 4)}
        if node.yaw is not None:
            entry['map_yaw'] = round(registration.wrap(node.yaw+transform.yaw), 6)
        nodes[node.id] = entry
    for edge in sorted(graph.edges.values(), key=lambda e: e.id):
        if edge.connection in connections:
            continue
        pa = transform.apply(*graph.nodes[edge.source].position)
        pb = transform.apply(*graph.nodes[edge.target].position)
        status, reason, detail = mv.classify(occupancy, *mv.edge_shapes(pa, pb, fp, clearance))
        if builder.layout_status[edge.connection]:
            status, reason = builder.layout_status[edge.connection]
        elif status == 'validated':
            for n in (edge.source, edge.target):
                if nodes[n]['status'] != 'validated':
                    status, reason = nodes[n]['status'], f'endpoint {n}: {nodes[n]["reason"]}'
                    break
        connections[edge.connection] = {'status': status, 'reason': reason, 'cells': detail}
    return nodes, connections


def build_artifact(paths, record=None):
    """Return the complete versioned graph artifact for the current installed inputs."""
    settings = load_settings(paths.file('planner_config'))
    config, _, compiler = load_layout(paths)
    footprint = mv.Footprint.from_nav2(yaml.safe_load(paths.file('nav2_params').read_text()))
    occupancy = mv.OccupancyMap.from_yaml(paths.file('map_yaml'))
    record = record if record is not None else json.loads(paths.file('registration').read_text())
    transform = registration.evaluate(record, occupancy.version(), digest(paths.file('layout')))
    builder = Builder(config, compiler, footprint, settings, wall_bounds(paths.file('building_model')))
    graph = builder.build()
    errors, warnings = validate_structure(graph, [{'id': s['id'], 'dock_node': s['dock_node']} for s in builder.stations])
    nodes, connections = map_validation(builder, occupancy, transform, settings)
    for station in builder.stations:
        dock = station['dock_node']
        checks = [(a, nodes[a]['status'], connections['--'.join(sorted((a, dock)))]['status'])
                  for a in station['approach_nodes']]
        station['validated_approaches'] = [a for a, node, edge in checks if node == edge == 'validated']
        if nodes[dock]['status'] != 'validated':
            station['status'], station['reason'] = nodes[dock]['status'], 'dock: '+nodes[dock]['reason']
        elif station['validated_approaches']:
            station['status'], station['reason'] = 'validated', 'dock, approach and access lane validated'
        elif checks:
            a, node, edge = checks[0]
            station['status'] = node if node != 'validated' else edge
            station['reason'] = f'approach {a}: node {node}, access lane {edge}'
        else:
            station['status'], station['reason'] = 'invalid', 'no collision-free access candidate'
    validated = [e for e in graph.edges.values() if connections[e.connection]['status'] == 'validated']
    data = graph.to_dict()
    for n in data['nodes']:
        n.update(nodes[n['id']])
    for e in data['edges']:
        e.update({k: v for k, v in connections[e['connection']].items()})
    def counts(values):
        values = list(values)
        return {s: values.count(s) for s in ('validated', 'unmapped', 'blocked', 'invalid')}
    artifact = {
        'schema_version': ARTIFACT_SCHEMA, 'frame_id': 'world', 'topology_version': graph.topology_version(),
        'sources': {key: {'package': SOURCE_FILES[key][0], 'path': SOURCE_FILES[key][1], 'sha256': digest(paths.file(key))}
                    | ({'override': str(paths.overrides[key])} if key in paths.overrides else {})
                    for key in sorted(SOURCE_FILES)},
        'map': {'version': occupancy.version(), 'resolution': occupancy.resolution,
                'origin': list(occupancy.origin), 'width': occupancy.shape[1], 'height': occupancy.shape[0]},
        'registration': {'id': record['id'], 'parent_frame': 'world', 'child_frame': 'map',
                         'transform': record['transform'], 'valid': True, 'method': record['method'],
                         'residuals': record['residuals']},
        'robot': {'footprint': {'front_m': footprint.front, 'back_m': footprint.back, 'half_width_m': footprint.half_width},
                  'circumscribed_radius_m': round(footprint.circumscribed, 6), 'required_clearance_m': builder.clearance},
        'corridors': [{'id': c.id, 'centre': [c.cx, c.cy], 'size': [c.sx, c.sy], 'capacity': c.capacity} for c in builder.corridors],
        # Restricted zones apply when enabled; temporary blocks apply only in their scenario.
        'zones': [{'id': z['id'], 'kind': z['kind'], 'centre': z['centre'], 'size': z['size'],
                   'enabled': z.get('enabled'), 'scenario': z.get('scenario')} for z in sorted(builder.zones(), key=lambda z: z['id'])],
        'nodes': data['nodes'], 'edges': data['edges'], 'stations': builder.stations,
        'components': {'topology': components(graph), 'validated': [g for g in components(graph, validated) if len(g) > 1]},
        'structure': {'errors': errors, 'warnings': warnings, 'notes': builder.notes},
        'summary': {'nodes': len(graph.nodes), 'directed_edges': len(graph.edges), 'connections': len(connections),
                    'node_status': counts(v['status'] for v in nodes.values()),
                    'connection_status': counts(v['status'] for v in connections.values()),
                    'station_status': {s['id']: s['status'] for s in builder.stations}},
    }
    artifact['graph_version'] = canonical_hash(artifact)[:16]
    if errors:
        raise ValueError('Graph structure invalid: '+'; '.join(errors))
    return artifact


def load_artifact(paths, path=None):
    """Load the artifact and reject it unless every source still matches."""
    path = Path(path) if path else paths.artifact()
    try:
        artifact = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise GraphStaleError(f'Graph artifact unavailable: {path}') from exc
    if artifact.get('schema_version') != ARTIFACT_SCHEMA:
        raise GraphStaleError('Unsupported graph artifact schema')
    for key, source in artifact['sources'].items():
        if key not in SOURCE_FILES or digest(paths.file(key)) != source['sha256']:
            raise GraphStaleError(f'Graph artifact is stale: {key} changed; run build_graph')
    if canonical_hash({k: v for k, v in artifact.items() if k != 'graph_version'})[:16] != artifact['graph_version']:
        raise GraphStaleError('Graph artifact content does not match its version')
    if artifact['structure']['errors']:
        raise GraphStaleError('Graph artifact has structural errors')
    return artifact
