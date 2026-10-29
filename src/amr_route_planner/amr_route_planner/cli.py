"""Offline graph tools: registration calibration, graph generation, route planning.

None of these commands start ROS or command the robot.
"""
import argparse
import json
from pathlib import Path
import sys
from PIL import Image, ImageDraw
from . import coordinate_registration as registration
from . import graph_builder as gb
from . import map_validation as mv

STATUS_COLOURS = {'validated': (0, 150, 70), 'unmapped': (240, 160, 0), 'blocked': (220, 0, 0), 'invalid': (150, 0, 150)}


def _paths(args):
    root = gb.Paths.source_tree().source_root if args.source_tree else args.source_root
    return gb.Paths(root, {'map_yaml': args.map, 'registration': args.registration})


def _common(parser):
    parser.add_argument('--source-tree', action='store_true', help='Read inputs from this workspace src/ instead of installed shares')
    parser.add_argument('--source-root', type=Path, help='Explicit src/ directory holding the packages')
    parser.add_argument('--map', type=Path, help='Evaluate a candidate map YAML instead of the approved map')
    parser.add_argument('--registration', type=Path, help='Registration record to use (default: package config)')


def calibrate(argv=None):
    p = argparse.ArgumentParser(description='Fit and verify the world → map registration for the approved map')
    _common(p)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(argv)
    paths = _paths(a)
    settings = gb.load_settings(paths.file('planner_config'))
    config, _, compiler = gb.load_layout(paths)
    occupancy = mv.OccupancyMap.from_yaml(paths.file('map_yaml'))
    landmarks = sorted(compiler.static_bounds(config)) + gb.wall_bounds(paths.file('building_model'))
    provenance = json.loads(paths.file('map_provenance').read_text())
    (wx, wy, wyaw), (mx, my, myaw) = provenance['initial_world_pose'], provenance['initial_map_pose']
    yaw = registration.wrap(myaw-wyaw)
    rotated = registration.SE2(0., 0., yaw).apply(wx, wy)
    prior = registration.SE2(mx-rotated[0], my-rotated[1], yaw)
    points = occupancy.occupied_points()
    fitted, score = registration.fit(points, landmarks, prior)
    thresholds = {k: settings['registration'][k] for k in
                  ('min_cells', 'max_median_m', 'min_inlier_fraction_0_20', 'max_prior_translation_m', 'max_prior_yaw_rad')}
    record = {
        'schema_version': registration.SCHEMA_VERSION, 'parent_frame': 'world', 'child_frame': 'map',
        'map_version': occupancy.version(),
        'map_files_sha256': {'yaml': gb.digest(paths.file('map_yaml')), 'image': gb.digest(paths.file('map_image'))},
        'layout_sha256': gb.digest(paths.file('layout')),
        'transform': fitted.as_dict(),
        'method': 'Robust truncated-mean (0.5 m) fit of real SLAM occupied cells to compiled static collision '
                  'geometry (walls, racks, pillars, fixed objects); grid search then coordinate descent.',
        'prior': {'source': 'amr_navigation/maps/provenance.json initial_world_pose → initial_map_pose',
                  'transform': prior.as_dict()},
        'residuals': registration.residual_summary(registration.residuals(fitted, points, landmarks)),
        'prior_residuals': registration.residual_summary(registration.residuals(prior, points, landmarks)),
        'objective_m': round(score, 6), 'landmarks': len(landmarks), 'thresholds': thresholds,
        'usage': 'Route geometry and visualization only; never published as TF or used to initialize AMCL.',
    }
    registration.accept(record, thresholds)
    a.output.write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps({k: record[k] for k in ('id', 'valid', 'reasons', 'transform', 'residuals', 'prior_residuals')}, indent=2))
    return 0 if record['valid'] else 1


def render(artifact, occupancy, path, route=None, scale=2):
    """Graph (and optionally a route) over the real occupancy map, map frame."""
    grid = occupancy.grid[::-1]
    pixels = Image.new('RGB', (grid.shape[1], grid.shape[0]))
    pixels.putdata([(255, 255, 255) if v == 0 else (0, 0, 0) if v == 100 else (190, 190, 190) for v in grid.flatten().tolist()])
    image = pixels.resize((grid.shape[1]*scale, grid.shape[0]*scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)
    ox, oy, _ = occupancy.origin
    h = grid.shape[0]

    def px(x, y):
        return ((x-ox)/occupancy.resolution*scale, (h-(y-oy)/occupancy.resolution)*scale)
    nodes = {n['id']: n for n in artifact['nodes']}
    for e in artifact['edges']:
        if e['source'] < e['target'] or e['direction'] == 'one_way':
            a, b = nodes[e['source']]['map_position'], nodes[e['target']]['map_position']
            draw.line([px(*a), px(*b)], fill=STATUS_COLOURS[e['status']], width=3)
    for n in artifact['nodes']:
        x, y = px(*n['map_position'])
        r = 7 if n['type'] == 'STATION' else 4
        draw.ellipse([x-r, y-r, x+r, y+r], fill=STATUS_COLOURS[n['status']], outline=(0, 0, 0))
        if n['type'] in ('STATION', 'OPEN_AREA'):
            draw.text((x+8, y-6), n['id'], fill=(0, 0, 120))
    if route:
        points = [px(*p) for p in route]
        draw.line(points, fill=(0, 90, 255), width=6)
        for x, y in points:
            draw.ellipse([x-6, y-6, x+6, y+6], outline=(0, 90, 255), width=3)
    image.save(path)


def build_graph(argv=None):
    p = argparse.ArgumentParser(description='Generate the versioned warehouse graph artifact')
    _common(p)
    p.add_argument('--output', type=Path, help='Artifact path (default: the package config directory)')
    p.add_argument('--report-dir', type=Path, help='Write the validation report and map overlay here')
    p.add_argument('--check', action='store_true', help='Fail unless the existing artifact equals a fresh build')
    a = p.parse_args(argv)
    paths = _paths(a)
    artifact = gb.build_artifact(paths)
    output = a.output or paths.artifact()
    text = json.dumps(artifact, indent=1, sort_keys=True)+'\n'
    if a.check:
        if not output.is_file() or output.read_text() != text:
            print('Graph artifact differs from a fresh deterministic build: '+str(output), file=sys.stderr)
            return 1
    else:
        output.write_text(text)
    if a.report_dir:
        a.report_dir.mkdir(parents=True, exist_ok=True)
        render(artifact, mv.OccupancyMap.from_yaml(paths.file('map_yaml')), a.report_dir/'graph-on-map.png')
        (a.report_dir/'graph-validation.json').write_text(json.dumps(
            {k: artifact[k] for k in ('graph_version', 'topology_version', 'summary', 'components', 'structure', 'stations')}
            | {'connections': sorted({e['connection']: {'status': e['status'], 'reason': e['reason']}
                                      for e in artifact['edges']}.items())}, indent=2)+'\n')
    print(json.dumps({'graph_version': artifact['graph_version'], **artifact['summary'],
                      'warnings': artifact['structure']['warnings'],
                      'rejected_or_unconnected_candidates': len(artifact['structure']['notes'])}, indent=2))
    return 0


EXIT = {'ok': 0, 'invalid_request': 2, 'no_route': 3, 'incompatible': 4}


def plan(argv=None):
    from .cost_model import Restrictions
    from .graph_builder import GraphStaleError
    from .route_planner import ALGORITHMS, RoutingError, load_context, plan as plan_route
    p = argparse.ArgumentParser(description='Plan a warehouse route without commanding the robot')
    _common(p)
    p.add_argument('--from', dest='origin', required=True, help='Station id or graph node id')
    p.add_argument('--to', dest='destination', required=True, help='Station id or graph node id')
    p.add_argument('--algorithm', choices=[*ALGORITHMS, 'both'], default='astar')
    p.add_argument('--scenario', help='Active warehouse scenario; activates its temporary blocked zones')
    p.add_argument('--block', action='append', default=[], metavar='CONNECTION', help='Close a connection (repeatable)')
    p.add_argument('--topology-only', action='store_true',
                   help='Ignore map validation to inspect topology; such a route is never executable')
    p.add_argument('--output', type=Path)
    p.add_argument('--render', type=Path, help='PNG of the route over the occupancy map')
    a = p.parse_args(argv)
    paths = _paths(a)
    try:
        artifact, transform, settings = load_context(paths)
    except (GraphStaleError, registration.RegistrationError, OSError, ValueError) as exc:
        print(f'Graph, map or registration incompatible: {exc}', file=sys.stderr)
        return EXIT['incompatible']
    connections = {e['connection'] for e in artifact['edges']}
    unknown = sorted(set(a.block)-connections)
    if unknown:
        print('Unknown connection(s): '+', '.join(unknown), file=sys.stderr)
        return EXIT['invalid_request']
    restrictions = Restrictions(a.scenario, frozenset(a.block), require_validated=not a.topology_only)
    reports = []
    try:
        for algorithm in (ALGORITHMS if a.algorithm == 'both' else [a.algorithm]):
            reports.append(plan_route(artifact, transform, settings, a.origin, a.destination, algorithm, restrictions)[0])
    except RoutingError as exc:
        print(json.dumps({'error': exc.code, 'message': str(exc)}, indent=2))
        return EXIT['invalid_request']
    if len(reports) == 2 and reports[0]['found'] and abs(reports[0]['cost']-reports[1]['cost']) > 1e-6:
        print('A* and Dijkstra disagree on the optimal cost', file=sys.stderr)
        return 1
    for r in reports:
        brief = {k: r.get(k) for k in ('algorithm', 'found', 'validation_status', 'distance_m', 'cost', 'expanded_states',
                                       'visited_nodes', 'planning_time_s', 'selected_nodes')}
        brief['objectives'] = [(w['node'], w['reason'], w['map']) for w in r.get('waypoints', [])]
        print(json.dumps(brief, indent=2))
    if a.output:
        a.output.write_text(json.dumps(reports if len(reports) > 1 else reports[0], indent=2)+'\n')
    if a.render and reports[0]['found']:
        render(artifact, mv.OccupancyMap.from_yaml(paths.file('map_yaml')), a.render, reports[0]['map_frame_path'])
    return EXIT['ok'] if reports[0]['found'] else EXIT['no_route']
