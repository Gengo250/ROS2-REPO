#!/usr/bin/env python3
"""Campus circulation graph from the scene's corridors, exported for the Nav2 Route Server.

Nodes: crossings of corridor centrelines and one access node per station. Edges: consecutive
nodes along each centreline (both directions) plus station accesses. Every edge is validated by
sweeping the robot footprint plus clearance against the scene geometry; failing edges are
dropped and reported. Costs: length, plus a penalty on capacity-1 rack aisles so main corridors are
preferred. Output: a GeoJSON graph in the MAP frame (through the verified registration) and a
JSON summary. Usage: build_campus_graph.py --registration campus_registration.json --output graph.geojson
"""
import argparse
import json
import math
from pathlib import Path
from atlas_scene.scene import Scene
from amr_lite_sim.geometry import StaticIndex, oriented_box

FOOTPRINT = (0.975+0.08, 0.70+0.08, 0.0125)   # Nav2 footprint incl. padding
CLEARANCE = 0.30
NARROW_PENALTY = 1.5                           # cost multiplier on capacity-1 aisles


def centreline(c):
    (cx, cy), (sx, sy) = c['centre'], c['size']
    return ((cx-sx/2, cy), (cx+sx/2, cy)) if sx >= sy else ((cx, cy-sy/2), (cx, cy+sy/2))


JUNCTION_REACH = 6.0   # a centreline may be extended this far to meet a perpendicular one (T-junction)


def along(c, p, reach=0.):
    """p lies on c's centreline span (extended by ``reach`` at both ends)."""
    (cx, cy), (sx, sy) = c['centre'], c['size']
    return abs(p[0]-cx) <= sx/2+reach if sx >= sy else abs(p[1]-cy) <= sy/2+reach


def swept_free(index, a, b, step=.2):
    length = math.dist(a, b)
    yaw = math.atan2(b[1]-a[1], b[0]-a[0])
    n = max(1, int(length/step))
    for i in range(n+1):
        x, y = a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n
        if index.collides(oriented_box(x, y, yaw, FOOTPRINT[0]+2*CLEARANCE, FOOTPRINT[1]+2*CLEARANCE, FOOTPRINT[2])):
            return False
    return True


def build(scene):
    corridors = scene.section('navigation')['corridors']
    index = StaticIndex(scene.band_polygons(0.035, 0.505))
    nodes, on_corridor = {}, {c['id']: [] for c in corridors}

    def node(p, kind, name=None):
        key = (round(p[0], 2), round(p[1], 2))
        for nid, n in nodes.items():
            if math.dist(n['xy'], key) < .3:
                return nid
        nid = name or f'n{len(nodes):03d}'
        nodes[nid] = {'xy': key, 'kind': kind}
        return nid
    lines = {c['id']: centreline(c) for c in corridors}
    for i, a in enumerate(corridors):
        (a0, a1) = lines[a['id']]
        for b in corridors[i+1:]:
            (b0, b1) = lines[b['id']]
            horizontal_a, horizontal_b = a0[1] == a1[1], b0[1] == b1[1]
            if horizontal_a == horizontal_b:
                continue
            p = (b0[0], a0[1]) if horizontal_a else (a0[0], b0[1])
            # Crossing or T-junction: the point is on one span and within reach of the other.
            if (along(a, p) and along(b, p, JUNCTION_REACH)) or (along(b, p) and along(a, p, JUNCTION_REACH)):
                nid = node(p, 'intersection')
                on_corridor[a['id']].append(nid)
                on_corridor[b['id']].append(nid)
    for c in corridors:   # dead ends keep their end points
        for end in lines[c['id']]:
            if not any(math.dist(nodes[n]['xy'], end) < 1. for n in on_corridor[c['id']]):
                on_corridor[c['id']].append(node(end, 'corridor_end'))
    stations = {}
    for s in scene.section('stations'):
        x, y = s['pose'][0], s['pose'][1]
        best = None
        for c in corridors:
            (p0, p1) = lines[c['id']]
            px, py = (min(max(x, p0[0]), p1[0]), p0[1]) if p0[1] == p1[1] else (p0[0], min(max(y, p0[1]), p1[1]))
            d = math.hypot(px-x, py-y)
            if best is None or d < best[0]:
                best = (d, c['id'], (px, py))
        sid = node((x, y), 'station', s['id'])
        stations[s['id']] = sid
        if best[0] > .3:
            access = node(best[2], 'access')
            on_corridor[best[1]].append(access)
            stations[s['id']+'__access'] = access
        else:
            on_corridor[best[1]].append(sid)
    edges, rejected = [], []
    capacity = {c['id']: c['capacity'] for c in corridors}

    def add(a, b, corridor):
        pa, pb = nodes[a]['xy'], nodes[b]['xy']
        if a == b:
            return
        if not swept_free(index, pa, pb):
            rejected.append({'source': a, 'target': b, 'corridor': corridor})
            return
        length = math.dist(pa, pb)
        cost = length*(NARROW_PENALTY if capacity.get(corridor, 2) == 1 else 1.)
        for s, t in ((a, b), (b, a)):
            edges.append({'id': len(edges), 'source': s, 'target': t, 'corridor': corridor, 'length': length, 'cost': cost})
    for cid, members in on_corridor.items():
        (p0, p1) = lines[cid]
        axis = 0 if p0[1] == p1[1] else 1
        # Ends that are not junctions stay as corridor_end nodes; junction nodes may lie on the
        # extension of the centreline, which the swept-footprint check then validates.
        ordered = sorted(set(members), key=lambda n: nodes[n]['xy'][axis])
        for a, b in zip(ordered, ordered[1:]):
            add(a, b, cid)
    for s in scene.section('stations'):
        if s['id']+'__access' in stations:
            add(stations[s['id']+'__access'], stations[s['id']], 'station_access')
    return nodes, edges, rejected, stations


def to_geojson(nodes, edges, registration):
    ids = {name: i for i, name in enumerate(nodes)}

    def to_map(p):
        c, s = math.cos(registration['yaw']), math.sin(registration['yaw'])
        return [c*p[0]-s*p[1]+registration['x'], s*p[0]+c*p[1]+registration['y']]
    features = [{'type': 'Feature', 'properties': {'id': ids[n], 'frame': 'map', 'name': n, 'kind': v['kind']},
                 'geometry': {'type': 'Point', 'coordinates': to_map(v['xy'])}} for n, v in nodes.items()]
    offset = len(nodes)
    for e in edges:
        features.append({'type': 'Feature', 'properties': {
            'id': offset+e['id'], 'startid': ids[e['source']], 'endid': ids[e['target']],
            'metadata': {'corridor': e['corridor'], 'length_m': round(e['length'], 3)},
            'cost': round(e['cost'], 3), 'overridable': False},
            'geometry': {'type': 'MultiLineString', 'coordinates': [[to_map(nodes[e['source']]['xy']), to_map(nodes[e['target']]['xy'])]]}})
    return {'type': 'FeatureCollection', 'name': 'atlas_campus_graph', 'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
            'date_generated': 'deterministic', 'features': features}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--registration', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    registration = json.loads(a.registration.read_text())
    if not registration.get('verified'):
        raise SystemExit('registration not verified')
    nodes, edges, rejected, stations = build(Scene.load())
    a.output.write_text(json.dumps(to_geojson(nodes, edges, registration['transform']), indent=1)+'\n')
    summary = {'nodes': len(nodes), 'directed_edges': len(edges), 'rejected_connections': rejected,
               'stations': {k: v for k, v in stations.items() if '__' not in k},
               'node_ids': {n: i for i, n in enumerate(nodes)}}
    a.output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=1)+'\n')
    print(json.dumps({k: summary[k] if k != 'node_ids' else len(summary[k]) for k in summary if k != 'stations'}))


if __name__ == '__main__':
    main()
