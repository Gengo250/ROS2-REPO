#!/usr/bin/env python3
"""Effective differential-drive kinematics from synchronized odometry vs ground truth.

Input: slam_diagnostics.py recordings. Pure in-place rotations give the effective
wheel separation (odometry rotation / true rotation scales it linearly); straight
segments give the effective wheel radius. Ground truth is evaluation data only; the
result is a calibration constant, never a runtime input.
"""
import argparse
import json
import math
from pathlib import Path
import statistics


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('recordings', type=Path, nargs='+')
    p.add_argument('--configured-separation', type=float, required=True)
    p.add_argument('--configured-radius', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    rotations, straights = [], []
    for path in a.recordings:
        rows = [r for r in json.loads(path.read_text())['samples'] if r['label']]
        segments = {}
        for r in rows:
            segments.setdefault(r['label'], []).append(r)
        for label, seg in segments.items():
            if len(seg) < 5:
                continue
            true_turn = sum(wrap(b['truth'][2]-a_['truth'][2]) for a_, b in zip(seg, seg[1:]))
            odom_turn = sum(wrap(b['odom'][2]-a_['odom'][2]) for a_, b in zip(seg, seg[1:]))
            true_dist = sum(math.dist(a_['truth'][:2], b['truth'][:2]) for a_, b in zip(seg, seg[1:]))
            odom_dist = sum(math.dist(a_['odom'][:2], b['odom'][:2]) for a_, b in zip(seg, seg[1:]))
            if abs(true_turn) > 1.0 and true_dist < 0.05:
                # ω_odom = Δv / b_cfg and ω_true = Δv / b_eff  ⇒  b_eff = b_cfg · θ_odom / θ_true
                rotations.append({'recording': path.parent.name, 'label': label, 'true_rad': true_turn, 'odom_rad': odom_turn,
                                  'ratio': odom_turn/true_turn})
            elif true_dist > 3.0 and abs(true_turn) < 0.1:
                straights.append({'recording': path.parent.name, 'label': label, 'true_m': true_dist, 'odom_m': odom_dist,
                                  'ratio': odom_dist/true_dist})
    report = {'rotations': rotations, 'straights': straights, 'configured': {'wheel_separation': a.configured_separation,
                                                                             'wheel_radius': a.configured_radius}}
    if rotations:
        ratios = [r['ratio'] for r in rotations]
        report['effective_wheel_separation_m'] = a.configured_separation*statistics.median(ratios)
        report['rotation_ratio'] = {'median': statistics.median(ratios), 'min': min(ratios), 'max': max(ratios), 'n': len(ratios)}
    if straights:
        ratios = [r['ratio'] for r in straights]
        # ODom distance scales with radius: r_eff = r_cfg · d_true / d_odom
        report['effective_wheel_radius_m'] = a.configured_radius/statistics.median(ratios)
        report['straight_ratio'] = {'median': statistics.median(ratios), 'min': min(ratios), 'max': max(ratios), 'n': len(ratios)}
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('rotations', 'straights')}, indent=2))


if __name__ == '__main__':
    main()
