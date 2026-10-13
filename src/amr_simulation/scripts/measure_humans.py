#!/usr/bin/env python3
"""Measure ground-truth displacement / simulation time in an isolated launch."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import time

from generate_warehouse import ROOT
from run_acceptance import simulation_processes, stop


def run(output, seconds, scenario, verify=False):
    assert not simulation_processes(), 'An existing simulation is running'
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(GZ_PARTITION='human_measure_'+str(os.getpid()),
                      ROS_DOMAIN_ID='94', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    from validate_runtime import Acceptance, distance, yaw, delta_angle
    import rclpy
    sim = observer = None
    with (output/'gazebo.log').open('w') as log:
        try:
            sim = subprocess.Popen(['ros2', 'launch', 'amr_simulation', 'warehouse.launch.py',
                                    'gui:=false', 'scenario:='+scenario], stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
            observer = Acceptance(scenario, output)
            observer.until(lambda: all(observer.counts[t] >= 3 for t in observer.topics)
                           and 'human_02' in observer.poses, 120)
            observer.wait(3)
            observer.history.clear()
            observer.rtf.clear()
            start, wall = observer.now(), time.monotonic()
            observer.wait(seconds)
            result = {'scenario': scenario, 'duration_sim_s': observer.now()-start,
                      'duration_wall_s': time.monotonic()-wall, 'humans': {},
                      'rtf_mean': statistics.mean(observer.rtf),
                      'history': observer.history, 'rtf_samples': observer.rtf}
            for name in ('human_01', 'human_02'):
                samples = [(a, b) for a, b in zip(observer.history, observer.history[1:])
                           if name in a and name in b and b['time'] > a['time']]
                speeds = [distance(a[name], b[name])/(b['time']-a['time']) for a, b in samples]
                total = sum(distance(a[name], b[name]) for a, b in samples)
                duration = sum(b['time']-a['time'] for a, b in samples)
                ordered = sorted(speeds)
                result['humans'][name] = {'distance_m': total, 'duration_s': duration,
                    'mean_m_s': total/duration, 'max_m_s': max(speeds),
                    'p95_m_s': ordered[int(.95*(len(ordered)-1))], 'speeds_m_s': speeds,
                    'max_yaw_rate_rad_s': max(abs(delta_angle(yaw(b[name]),yaw(a[name])))/(b['time']-a['time']) for a,b in samples),
                    'max_acceleration_m_s2': max(abs(v1-v0)/(b['time']-a['time']) for v0,v1,(a,b) in zip(speeds,speeds[1:],samples[1:]))}
            result['humans']['human_03'] = {'mean_m_s': 0, 'static': True}
            (output/'measurement.json').write_text(json.dumps(result, indent=2)+'\n')
            if verify:
                for name in ('human_01','human_02'):
                    human=next(h for h in observer.c['humans'] if h['id']==name)
                    override=json.loads((ROOT/'config/scenarios'/(scenario+'.json')).read_text())['human_overrides'].get(name,{})
                    target=override.get('speed',human.get('speed',observer.c['human_model']['motion']['speed']))
                    measured=result['humans'][name]
                    assert measured['max_m_s']<=1.4, measured
                    assert abs(measured['p95_m_s']-target)<.15, measured
                    assert measured['distance_m']>seconds*.2, measured
                    assert measured['max_yaw_rate_rad_s']<1.5, measured
                result['verification']='passed'
                (output/'measurement.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps({k:v for k,v in result.items() if k not in ('history','rtf_samples')}, indent=2))
        finally:
            if observer:
                observer.node.destroy_node()
                rclpy.shutdown()
            stop(sim)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=40)
    parser.add_argument('--scenario', default='normal')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    run(args.output, args.seconds, args.scenario, args.verify)
