#!/usr/bin/env python3
"""Rebuild a candidate map offline from a recorded survey bag (``extend_map_survey.py --bag``).

The recorded real sensor data (/scan, wheel-odometry /tf and /odom, /tf_static,
/clock) are played into SLAM Toolbox with the given parameters, which is SLAM
Toolbox's standard offline workflow. The sim-time drift recorder runs alongside it
(ground truth from the bag is evaluation only), and the occupancy map is saved.
Segment labels are copied by simulation stamp from the live survey's recording, so
the same per-segment drift analysis applies. Nothing is promoted automatically.
"""
import argparse
import bisect
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
NAV = ROOT/'src/amr_navigation'
TOPICS = ['/scan', '/tf', '/tf_static', '/odom', '/clock', '/digital_twin/simulation/poses']


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def relabel(recording, original):
    """Copy segment labels from the live survey's recording by simulation stamp."""
    live = json.loads(original.read_text())['samples']
    stamps = [r['sim'] for r in live]
    data = json.loads(recording.read_text())
    for row in data['samples']:
        i = bisect.bisect_left(stamps, row['sim'])
        near = [j for j in (i-1, i) if 0 <= j < len(live)]
        j = min(near, key=lambda k: abs(stamps[k]-row['sim']))
        row['label'] = live[j]['label'] if abs(stamps[j]-row['sim']) < .2 else ''
    recording.write_text(json.dumps(data)+'\n')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--slam-params', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--labels-from', type=Path, help='drift-recording.json of the live survey')
    p.add_argument('--rate', type=float, default=2.5, help='Playback rate relative to the recording wall clock')
    p.add_argument('--domain', type=int, default=66)
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy(a.slam_params, out/'slam-params.yaml')
    env = {**os.environ, 'ROS_DOMAIN_ID': str(a.domain), 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    report = {'bag': str(a.bag.resolve()), 'slam_params': str(a.slam_params), 'rate': a.rate, 'passed': False}
    children, logs = [], []

    def launch(name, cmd):
        f = (out/(name+'.log')).open('w')
        logs.append(f)
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(proc)
        return proc

    def run(name, cmd, timeout):
        with (out/(name+'.log')).open('w') as f:
            return subprocess.run(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, timeout=timeout).returncode

    try:
        launch('slam', ['ros2', 'launch', 'amr_navigation', 'mapping.launch.py', 'slam_params_file:='+str(out/'slam-params.yaml')])
        recorder = launch('recorder', ['python3', str(NAV/'scripts/slam_diagnostics.py'), '--output', str(out/'drift-recording.json')])
        time.sleep(8)
        started = time.monotonic()
        if run('play', ['ros2', 'bag', 'play', str(a.bag), '--rate', str(a.rate), '--topics', *TOPICS], 7200) != 0:
            raise RuntimeError('Bag playback failed; inspect play.log')
        report['playback_wall_s'] = time.monotonic()-started
        time.sleep(5)
        if run('save', ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(out/'warehouse'), '--ros-args',
                        '-p', 'use_sim_time:=true', '-p', 'map_subscribe_transient_local:=true',
                        '-p', 'save_map_timeout:=20.0'], 60) != 0:
            raise RuntimeError('Map save failed')
        stop(recorder)
        report['passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        for child in reversed(children):
            stop(child)
        for f in logs:
            f.close()
        report['owned_processes_stopped'] = all(c.poll() is not None for c in children)
    if (out/'drift-recording.json').exists():
        if a.labels_from:
            relabel(out/'drift-recording.json', a.labels_from)
        run('analysis', ['python3', str(NAV/'scripts/analyze_slam_diagnostics.py'), str(out/'drift-recording.json'),
                         '--output', str(out/'drift-analysis.json'), '--plot', str(out/'drift-errors.png')], 120)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
