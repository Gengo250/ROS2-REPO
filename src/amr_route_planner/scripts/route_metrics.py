#!/usr/bin/env python3
"""Summarize measured metrics of one owned route integration run (no simulation access)."""
import argparse
import json
from pathlib import Path
import statistics


def median(values):
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def step_metrics(route):
    history = route['status_history']
    executing = [h['wall'] for h in history if h['status'] == 'EXECUTING']
    terminal = [h['wall'] for h in history if h['status'] in ('COMPLETED', 'FAILED', 'BLOCKED', 'CANCELED')]
    sims = [t['sim'] for t in route['trajectory'] if t.get('sim')]
    plans = [r for r in route['routes'] if r.get('found')]
    first = plans[0] if plans else {}
    actions = route.get('action_results', [])
    return {
        'final_status': route['final_status']['status'], 'failure_reason': route['final_status']['failure_reason'],
        'algorithm': first.get('algorithm'), 'route_nodes': first.get('selected_nodes'),
        'route_distance_m': first.get('distance_m'), 'route_cost': first.get('cost'),
        'expanded_states': first.get('expanded_states'), 'visited_nodes': first.get('visited_nodes'),
        'planning_time_s': first.get('planning_time_s'), 'objectives': len(first.get('waypoints', [])),
        'dispatched_routes': sum(1 for h in history if h['status'] == 'EXECUTING'),
        'nav2_goals_dispatched': len(executing),
        'execution_wall_s': (terminal[-1]-executing[0]) if executing and terminal else None,
        'execution_sim_s': (sims[-1]-sims[0]) if len(sims) > 1 else None,
        'travel_m': {frame: route.get(f'{frame}_travel_distance_m') for frame in ('map', 'odom', 'world')},
        'waypoint_or_action_failures': sum(1 for a in actions if a['status'] != 'SUCCEEDED'),
        'replans': len(route['replans']), 'replan_reasons': [r['reason'] for r in route['replans']],
        'blocked_connections': route['final_status']['blocked_connections'],
        'recoveries': route.get('recoveries'),
        'final_position_error_m': route.get('final_position_error_m'), 'final_yaw_error_rad': route.get('final_yaw_error_rad'),
        'final_ground_truth_error_m': route.get('final_ground_truth_error_m'),
        'corridor_conformance': route.get('corridor_conformance'),
        'registration_evaluation': route.get('registration_evaluation'),
        'plan_checks': len(route['plan_checks']), 'plan_points_outside_lanes': sum(c['outside'] for c in route['plan_checks']),
        'stopped_at_end': route.get('stopped'),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    run = a.run
    summary = {'run': run.name, 'report': json.loads((run/'report.json').read_text()), 'steps': []}
    for path in sorted(run.glob('route-*.json')):
        summary['steps'].append({'file': path.name, **step_metrics(json.loads(path.read_text()))})
    if (run/'contacts.json').exists():
        contacts = json.loads((run/'contacts.json').read_text())
        summary['collision_observation'] = {'scoped_amr_contact_samples': contacts['amr_contact_samples'],
                                            'coverage': contacts.get('coverage')}
    if (run/'sensors.json').exists():
        summary['real_time_factor'] = json.loads((run/'sensors.json').read_text()).get('rtf')
    if (run/'browser.json').exists():
        samples = json.loads((run/'browser.json').read_text())['samples']
        fps = [s['metrics'].get('fps') for s in samples if s.get('metrics')]
        summary['threejs'] = {'samples': len(samples), 'median_fps': median(fps), 'min_fps': min(f for f in fps if f) if any(fps) else None,
                              'overlay_visible_samples': sum(1 for s in samples if s['route'].get('visible')),
                              'overlay_statuses': sorted({s['route'].get('status') for s in samples if s['route'].get('status')})}
    text = json.dumps(summary, indent=2)
    if a.output:
        a.output.write_text(text+'\n')
    print(text)


if __name__ == '__main__':
    main()
