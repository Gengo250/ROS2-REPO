#!/usr/bin/env python3
"""Quality report for an approved or candidate occupancy map (offline, no ROS graph).

Combines: explored/known-free area, global and per-tile landmark residuals of the
verified registration (so a low median cannot hide a local distortion), loop-return
drift from a synchronized survey recording, and the graph coverage the map supports:
validated nodes/connections/stations, reachable station pairs and independent cycles.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import numpy as np
from amr_route_planner import coordinate_registration as reg
from amr_route_planner import graph_builder as gb
from amr_route_planner import map_validation as mv
from amr_route_planner.cost_model import CostModel, Restrictions, exclusions
from amr_route_planner.graph_model import Graph, components
from amr_route_planner.route_planner import search


def graph_metrics(artifact, settings):
    graph = Graph.from_dict(artifact)
    edges, nodes = exclusions(artifact, Restrictions())
    usable = [e for e in graph.edges.values() if e.id not in edges and e.source not in nodes and e.target not in nodes]
    connections = {e.connection: (e.source, e.target) for e in usable}
    vertices = {n for pair in connections.values() for n in pair}
    groups = components(graph, usable)
    # Independent undirected cycles: E - V + C over the validated subgraph.
    cycles = len(connections)-len(vertices)+len([g for g in groups if len(g) > 1]) if connections else 0
    stations = [s['id'] for s in artifact['stations'] if s['status'] == 'validated']
    model = CostModel.from_settings(settings)
    pairs = [(a, b) for a, b in itertools.permutations(stations, 2)
             if search(graph, a, b, model, 'astar', edges, nodes).found]
    return {'validated_nodes': artifact['summary']['node_status']['validated'],
            'validated_connections': artifact['summary']['connection_status']['validated'],
            'validated_stations': stations, 'reachable_station_pairs': len(pairs),
            'independent_validated_cycles': cycles, 'node_status': artifact['summary']['node_status'],
            'connection_status': artifact['summary']['connection_status'], 'station_status': artifact['summary']['station_status']}


def residual_tiles(points, residuals, transform, size=4.):
    world = np.array([transform.inverse().apply(*p) for p in points])
    tiles = []
    for y in np.arange(-10, 10, size):
        for x in np.arange(-15, 15, size):
            m = (world[:, 0] >= x) & (world[:, 0] < x+size) & (world[:, 1] >= y) & (world[:, 1] < y+size)
            if m.sum() >= 20:
                tiles.append({'x': float(x), 'y': float(y), 'cells': int(m.sum()), 'median_m': float(np.median(residuals[m]))})
    medians = sorted(t['median_m'] for t in tiles)
    return {'tile_size_m': size, 'tiles': tiles, 'worst_tile_median_m': medians[-1] if medians else None,
            'p90_tile_median_m': medians[int(.9*(len(medians)-1))] if medians else None,
            'tiles_over_0_20_m': sum(m > .2 for m in medians)}


def loop_return(recording):
    rows = json.loads(Path(recording).read_text())['samples']
    if len(rows) < 2:
        return None
    first, last = rows[0], rows[-1]

    def rel(a, b):
        c, s = math.cos(a[2]), math.sin(a[2])
        dx, dy = b[0]-a[0], b[1]-a[1]
        return [c*dx+s*dy, -s*dx+c*dy, math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))]
    est, gt = rel(first['slam'], last['slam']), rel(first['truth'], last['truth'])
    travelled = sum(math.dist(a['truth'][:2], b['truth'][:2]) for a, b in zip(rows, rows[1:]))
    errors = []
    for r in rows:
        e, g = rel(first['slam'], r['slam']), rel(first['truth'], r['truth'])
        errors.append(math.hypot(e[0]-g[0], e[1]-g[1]))
    errors.sort()
    return {'travelled_m': travelled, 'final_position_difference_m': math.hypot(est[0]-gt[0], est[1]-gt[1]),
            'final_yaw_difference_rad': abs(math.atan2(math.sin(est[2]-gt[2]), math.cos(est[2]-gt[2]))),
            'start_to_end_true_separation_m': math.hypot(gt[0], gt[1]),
            'error_p50_m': errors[len(errors)//2], 'error_p90_m': errors[int(.9*(len(errors)-1))], 'error_max_m': errors[-1]}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source-tree', action='store_true')
    p.add_argument('--map', type=Path, help='Candidate map YAML (default: approved)')
    p.add_argument('--registration', type=Path)
    p.add_argument('--graph', type=Path, help='Graph artifact built for this map')
    p.add_argument('--recording', type=Path, help='slam_diagnostics recording of the survey that built the map')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    root = gb.Paths.source_tree().source_root if a.source_tree else None
    paths = gb.Paths(root, {'map_yaml': a.map, 'registration': a.registration, 'graph': a.graph})
    settings = gb.load_settings(paths.file('planner_config'))
    occupancy = mv.OccupancyMap.from_yaml(paths.file('map_yaml'))
    record = json.loads(paths.file('registration').read_text())
    transform = reg.evaluate(record, occupancy.version(), gb.digest(paths.file('layout')))
    config, _, compiler = gb.load_layout(paths)
    landmarks = sorted(compiler.static_bounds(config))+gb.wall_bounds(paths.file('building_model'))
    points = occupancy.occupied_points()
    residuals = reg.residuals(transform, points, landmarks)
    cell = occupancy.resolution**2
    artifact = gb.load_artifact(paths)
    report = {'map_version': occupancy.version(), 'registration_id': record['id'], 'transform': record['transform'],
              'known_free_m2': float((occupancy.grid == mv.FREE).sum()*cell),
              'occupied_m2': float((occupancy.grid == mv.OCCUPIED).sum()*cell),
              'unknown_m2': float((occupancy.grid == mv.UNKNOWN).sum()*cell),
              'registration_global': reg.residual_summary(residuals),
              'registration_local': residual_tiles(points, residuals, transform),
              'graph': graph_metrics(artifact, settings),
              'loop_return': loop_return(a.recording) if a.recording else None}
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    brief = {k: report[k] for k in ('known_free_m2', 'registration_global', 'loop_return')}
    brief['local'] = {k: v for k, v in report['registration_local'].items() if k != 'tiles'}
    brief['graph'] = {k: v for k, v in report['graph'].items() if k not in ('node_status', 'connection_status')}
    print(json.dumps(brief, indent=2))


if __name__ == '__main__':
    main()
