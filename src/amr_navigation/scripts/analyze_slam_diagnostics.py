#!/usr/bin/env python3
"""Per-segment SLAM vs odometry drift against ground truth, aligned at each segment start.

Relative motion Δ = P(t0)⁻¹·P(t) is compared between estimate and ground truth, so
no world/map registration enters the drift measurement.
"""
import argparse
import json
import math
from pathlib import Path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative(p0, p):
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dx, dy = p[0]-p0[0], p[1]-p0[1]
    return [c*dx+s*dy, -s*dx+c*dy, wrap(p[2]-p0[2])]


def errors(rows, key):
    out = []
    for r in rows:
        e, g = relative(rows[0][key], r[key]), relative(rows[0]['truth'], r['truth'])
        out.append((math.hypot(e[0]-g[0], e[1]-g[1]), wrap(e[2]-g[2])))
    return out


def summarize(rows):
    dist = sum(math.dist(a['truth'][:2], b['truth'][:2]) for a, b in zip(rows, rows[1:]))
    turn = sum(abs(wrap(b['truth'][2]-a['truth'][2])) for a, b in zip(rows, rows[1:]))
    result = {'samples': len(rows), 'sim_s': rows[-1]['sim']-rows[0]['sim'], 'distance_m': dist, 'rotation_rad': turn}
    for key in ('slam', 'odom'):
        e = errors(rows, key)
        pos = sorted(x[0] for x in e)
        result[key] = {'final_position_error_m': e[-1][0], 'final_yaw_error_rad': e[-1][1],
                       'max_position_error_m': pos[-1], 'p90_position_error_m': pos[int(.9*(len(pos)-1))],
                       'max_abs_yaw_error_rad': max(abs(x[1]) for x in e),
                       # Largest single-step disagreement: detects sudden scan-matching jumps.
                       'max_step_jump_m': max((abs(b[0]-a[0]) for a, b in zip(e, e[1:])), default=0.)}
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('recording', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--plot', type=Path)
    a = p.parse_args()
    rows = [r for r in json.loads(a.recording.read_text())['samples'] if r['label']]
    segments, order = {}, []
    for r in rows:
        if r['label'] not in segments:
            order.append(r['label'])
        segments.setdefault(r['label'], []).append(r)
    report = {'segments': {k: summarize(segments[k]) for k in order if len(segments[k]) > 2},
              'whole_run': summarize(rows) if len(rows) > 2 else None}
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    for k, v in report['segments'].items():
        print(f"{k:28s} dist {v['distance_m']:6.2f} m rot {v['rotation_rad']:5.2f} rad | SLAM final {v['slam']['final_position_error_m']:.3f} m "
              f"{v['slam']['final_yaw_error_rad']:+.3f} rad max {v['slam']['max_position_error_m']:.3f} jump {v['slam']['max_step_jump_m']:.3f} | "
              f"odom final {v['odom']['final_position_error_m']:.3f} m {v['odom']['final_yaw_error_rad']:+.3f} rad")
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(14, 8))
        travelled, total, last = [], 0., None
        for r in rows:
            if last is not None:
                total += math.dist(last, r['truth'][:2])
            last = r['truth'][:2]
            travelled.append(total)
        t0 = rows[0]['sim']
        for key, style in (('slam', '-'), ('odom', '--')):
            e = errors(rows, key)
            axes[0][0].plot(travelled, [x[0] for x in e], style, label=key)
            axes[1][0].plot(travelled, [x[1] for x in e], style, label=key)
            axes[0][1].plot([r['sim']-t0 for r in rows], [x[0] for x in e], style, label=key)
            axes[1][1].plot([r['sim']-t0 for r in rows], [x[1] for x in e], style, label=key)
        for ax in (axes[0][1], axes[1][1]):
            for k in order:
                ax.axvline(segments[k][0]['sim']-t0, color='grey', lw=.5)
                ax.text(segments[k][0]['sim']-t0, ax.get_ylim()[1], k, rotation=90, fontsize=6, va='top')
        axes[0][0].set(xlabel='ground-truth distance travelled (m)', ylabel='position error (m)', title='Whole run, aligned at start')
        axes[1][0].set(xlabel='ground-truth distance travelled (m)', ylabel='yaw error (rad)')
        axes[0][1].set(xlabel='simulation time (s)', ylabel='position error (m)')
        axes[1][1].set(xlabel='simulation time (s)', ylabel='yaw error (rad)')
        for ax in axes.flat:
            ax.grid(alpha=.3)
            ax.legend()
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)


if __name__ == '__main__':
    main()
