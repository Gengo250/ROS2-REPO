#!/usr/bin/env python3
"""Owned, supervised coverage extension of the approved SLAM map; produces a CANDIDATE.

Reuses Phase 4 infrastructure only: worker-free ``mapping_only`` simulation, SLAM
Toolbox resumed from the approved serialized pose graph, and ``mapping_jog.py``
legs (rotation, then bounded forward motion with a LiDAR stop). After each leg the
SLAM estimate is compared with independent Gazebo poses for EVALUATION; ground
truth never reaches SLAM or TF. The approved ``maps/warehouse.*`` is not modified.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

ROOT = Path(__file__).resolve().parents[3]
NAV = ROOT/'src/amr_navigation'
# World-frame survey corners (default: short loops closing on the main aisle). Recorded
# outcome: every automated route that travelled along a cross aisle (aliasing rack ends)
# or turned in place facing the north/south wall drifted 0.4–2.0 m and was rejected
# (survey-v4/v5/v6). Extend coverage with the operator-supervised Phase 4 procedure.
SURVEY = [(-11.0, 0.0), (8.1, 0.0), (8.1, 8.5), (3.9, 8.5), (3.9, -8.5), (8.1, -8.5), (8.1, 0.0),
          (-8.7, 0.0), (-8.7, 8.5), (-8.7, -8.5), (-8.7, 0.0), (-11.0, 0.0)]


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--posegraph', type=Path, default=ROOT/'docs/digital-twin/evidence-phase4/warehouse',
                   help='Serialized approved pose graph base path (no extension)')
    p.add_argument('--registration', type=Path, default=ROOT/'src/amr_route_planner/config/map_registration.json')
    p.add_argument('--max-drift', type=float, default=0.40, help='Abort when SLAM-vs-ground-truth error exceeds this (m)')
    p.add_argument('--domain', type=int, default=46)
    p.add_argument('--corners', type=json.loads, default=SURVEY, help='JSON list of [x, y] world corners')
    a = p.parse_args()
    survey = [tuple(c) for c in a.corners]
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    if not a.posegraph.with_suffix('.posegraph').is_file():
        raise SystemExit('Approved serialized pose graph missing: '+str(a.posegraph))
    reg = json.loads(a.registration.read_text())['transform']
    params = yaml.safe_load((NAV/'config/slam_params.yaml').read_text())
    params['slam_toolbox']['ros__parameters'].update(map_file_name=str(a.posegraph.resolve()), map_start_pose=[0., 0., 0.])
    (out/'resume-slam.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
    env = {**os.environ, 'ROS_DOMAIN_ID': str(a.domain), 'GZ_PARTITION': f'phase5_mapping_{os.getpid()}',
           'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    children, logs = [], []
    report = {'survey_world_corners': survey, 'registration_used_for_headings': reg, 'legs': [], 'passed': False}

    def launch(name, cmd):
        f = (out/(name+'.log')).open('w')
        logs.append(f)
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(proc)
        return proc

    def run(name, cmd, timeout):
        with (out/(name+'.log')).open('w') as f:
            return subprocess.run(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, timeout=timeout).returncode

    def drift(name):
        if run(name, ['python3', str(NAV/'scripts/observe_registration.py'), '--output', str(out/(name+'.json')),
                      '--duration', '6'], 60) != 0:
            return None
        rows = json.loads((out/(name+'.json')).read_text())['samples']
        if not rows:
            return None
        # Re-evaluate with the verified SE(2) registration instead of the +11 m evaluation offset.
        c, s = math.cos(reg['yaw']), math.sin(reg['yaw'])
        errors = [math.hypot(r['map'][0]-(c*r['world'][0]-s*r['world'][1]+reg['x']),
                             r['map'][1]-(s*r['world'][0]+c*r['world'][1]+reg['y'])) for r in rows]
        yaw = [math.atan2(math.sin(r['map'][2]-r['world'][2]-reg['yaw']), math.cos(r['map'][2]-r['world'][2]-reg['yaw'])) for r in rows]
        return {'samples': len(rows), 'position_error_m': sorted(errors)[len(errors)//2], 'yaw_error_rad': sorted(yaw)[len(yaw)//2]}

    try:
        sim = launch('simulation', ['ros2', 'launch', 'amr_simulation', 'warehouse.launch.py', 'gui:=false', 'mapping_only:=true'])
        launch('pose-adapter', ['ros2', 'run', 'digital_twin_gateway', 'simulation_pose_adapter'])
        time.sleep(25)
        if sim.poll() is not None:
            raise RuntimeError('Simulation exited')
        launch('slam', ['ros2', 'launch', 'amr_navigation', 'mapping.launch.py', 'slam_params_file:='+str(out/'resume-slam.yaml')])
        # Loading the serialized pose graph takes a variable time; wait for SLAM TF.
        for attempt in range(9):
            time.sleep(10)
            report['initial'] = drift(f'drift-initial-{attempt}')
            if report['initial'] is not None:
                break
        if report['initial'] is None or report['initial']['position_error_m'] > a.max_drift:
            raise RuntimeError('Resumed SLAM is not consistent with the approved map registration')
        for i, (start, end) in enumerate(zip(survey, survey[1:])):
            heading = math.atan2(end[1]-start[1], end[0]-start[0])+reg['yaw']
            distance = math.dist(start, end)
            code = run(f'leg-{i:02d}', ['python3', str(NAV/'scripts/mapping_jog.py'), '--heading', f'{heading:.6f}',
                                        '--distance', f'{distance:.3f}', '--output', str(out/f'leg-{i:02d}.json')], 600)
            leg = {'index': i, 'from_world': start, 'to_world': end, 'map_heading': heading, 'distance_m': distance,
                   'returncode': code, 'drift': drift(f'drift-{i:02d}')}
            report['legs'].append(leg)
            (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            if code != 0:
                raise RuntimeError(f'Survey leg {i} stopped; inspect leg-{i:02d}.log')
            if leg['drift'] is None or leg['drift']['position_error_m'] > a.max_drift:
                raise RuntimeError(f'Leg {i}: SLAM estimate drifted beyond {a.max_drift} m; candidate rejected')
        time.sleep(5)
        if run('save', ['bash', str(NAV/'scripts/save_map.sh'), str(out/'warehouse')], 120) != 0:
            raise RuntimeError('Map save failed')
        report['passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
        # Preserve the partial candidate for review; it is never promoted automatically.
        run('save-partial', ['bash', str(NAV/'scripts/save_map.sh'), str(out/'partial')], 120)
    finally:
        for child in reversed(children):
            stop(child)
        for f in logs:
            f.close()
        report['owned_processes_stopped'] = all(c.poll() is not None for c in children)
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'legs'} | {'legs': len(report['legs'])}, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
