#!/usr/bin/env python3
"""Sample CPU and resident memory of the simulation/navigation processes (read-only, /proc).

Writes per-group CPU % (of one core) and RSS MiB every --period seconds until
--duration or SIGINT. Groups are matched on the command line.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

GROUPS = {'gazebo': ('gz sim', 'ruby'), 'slam_or_amcl': ('slam_toolbox', 'amcl'), 'nav2': ('nav2', 'controller_server', 'planner_server',
          'bt_navigator', 'behavior_server', 'smoother_server', 'velocity_smoother', 'waypoint_follower', 'lifecycle_manager'),
          'ros_gz_bridge': ('parameter_bridge',), 'gateway': ('digital_twin_gateway', 'uvicorn'), 'browser': ('chrome', 'chromium', 'headless_shell'),
          'route_and_guard': ('route_executor', 'execute_route', 'command_guard', 'navigation_command_guard')}
TICK = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def snapshot():
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            cmd = (Path('/proc', pid, 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace'))
            stat = Path('/proc', pid, 'stat').read_text().rsplit(')', 1)[1].split()
            rss = int(Path('/proc', pid, 'statm').read_text().split()[1])*PAGE
        except (OSError, IndexError, ValueError):
            continue
        group = next((g for g, keys in GROUPS.items() if any(k in cmd for k in keys)), None)
        if group:
            out[int(pid)] = (group, int(stat[11])+int(stat[12]), rss)
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--period', type=float, default=5.)
    p.add_argument('--duration', type=float, default=7200.)
    a = p.parse_args()
    running = [True]
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))
    signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))
    rows, before, t0 = [], snapshot(), time.monotonic()
    last = t0
    while running[0] and time.monotonic()-t0 < a.duration:
        time.sleep(a.period)
        now, current = time.monotonic(), snapshot()
        row = {'t': now-t0, 'groups': {}}
        for pid, (group, ticks, rss) in current.items():
            g = row['groups'].setdefault(group, {'cpu_percent': 0., 'rss_mib': 0.})
            if pid in before:
                g['cpu_percent'] += 100.*(ticks-before[pid][1])/TICK/(now-last)
            g['rss_mib'] += rss/2**20
        rows.append(row)
        before, last = current, now
    summary = {}
    for group in GROUPS:
        cpu = sorted(r['groups'][group]['cpu_percent'] for r in rows if group in r['groups'])
        mem = sorted(r['groups'][group]['rss_mib'] for r in rows if group in r['groups'])
        if cpu:
            summary[group] = {'cpu_percent_median': cpu[len(cpu)//2], 'cpu_percent_p90': cpu[int(.9*(len(cpu)-1))],
                              'rss_mib_median': mem[len(mem)//2], 'rss_mib_max': mem[-1]}
    a.output.write_text(json.dumps({'cores': os.cpu_count(), 'period_s': a.period, 'summary': summary, 'samples': rows}, indent=1)+'\n')
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
