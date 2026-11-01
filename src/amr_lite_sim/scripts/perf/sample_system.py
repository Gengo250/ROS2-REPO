#!/usr/bin/env python3
"""Sample per-process CPU/RSS (grouped), Intel GPU busy (RC6 residency), i915 per-client
render-engine busy time from /proc/<pid>/fdinfo, and nouveau runtime power state.
Read-only; writes one JSON report.
"""
import argparse
import json
import os
import signal
import time
from pathlib import Path

GROUPS = {
    'gazebo': ('gz sim', 'ruby'),
    'ros_gz_bridge': ('parameter_bridge',),
    'robot_state_publisher': ('robot_state_publisher',),
    'controller_spawner': ('controller_manager/spawner',),
    'nav2': ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'lifecycle_manager',
             'map_server', 'amcl'),
    'slam': ('slam_toolbox',),
    'command_guard': ('command_guard',),
    'gateway': ('digital_twin_gateway/lib', '/gateway', 'simulation_pose_adapter', 'uvicorn'),
    'lite_engine': ('lite_sim', 'amr_lite'),
    'browser': ('chrome', 'chromium', 'headless_shell'),
    'vite': ('vite',),
    'static_tf': ('static_transform_publisher',),
    'monitor': ('monitor_topics.py',),
}
TICK = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')
INTEL_GT = Path('/sys/class/drm/card1/gt/gt0')
NOUVEAU = Path('/sys/class/drm/card0/device/power/runtime_status')


def classify(cmd):
    for group, keys in GROUPS.items():
        if any(k in cmd for k in keys):
            return group
    return None


def gpu_busy_ns(pid):
    total = 0
    try:
        for fd in os.listdir(f'/proc/{pid}/fdinfo'):
            try:
                text = Path(f'/proc/{pid}/fdinfo/{fd}').read_text()
            except OSError:
                continue
            if 'drm-driver' not in text:
                continue
            for line in text.splitlines():
                if line.startswith('drm-engine-render:') or line.startswith('drm-engine-gfx:'):
                    total += int(line.split()[1])
    except OSError:
        pass
    return total


def snapshot():
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            cmd = Path('/proc', pid, 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            group = classify(cmd)
            if not group:
                continue
            stat = Path('/proc', pid, 'stat').read_text().rsplit(')', 1)[1].split()
            rss = int(Path('/proc', pid, 'statm').read_text().split()[1])*PAGE
        except (OSError, IndexError, ValueError):
            continue
        out[int(pid)] = (group, int(stat[11])+int(stat[12]), rss, gpu_busy_ns(pid))
    return out


def read_int(path):
    try:
        return int(path.read_text())
    except (OSError, ValueError):
        return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=30)
    p.add_argument('--period', type=float, default=2)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    rows = []
    running = [True]
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))
    signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))
    before, t_before = snapshot(), time.monotonic()
    rc6_before = read_int(INTEL_GT/'rc6_residency_ms')
    start = t_before
    cpu_total_before = sum(map(int, Path('/proc/stat').read_text().split('\n')[0].split()[1:]))
    idle_before = int(Path('/proc/stat').read_text().split('\n')[0].split()[4])
    while running[0] and time.monotonic()-start < a.duration:
        time.sleep(a.period)
        if not running[0]:
            break
        now, current = time.monotonic(), snapshot()
        dt = now-t_before
        row = {'t': now-start, 'groups': {}}
        for pid, (group, ticks, rss, gpu) in current.items():
            g = row['groups'].setdefault(group, {'cpu_percent': 0., 'rss_mib': 0., 'intel_gpu_percent': 0.})
            if pid in before:
                g['cpu_percent'] += 100.*(ticks-before[pid][1])/TICK/dt
                g['intel_gpu_percent'] += 100.*(gpu-before[pid][3])/1e9/dt
            g['rss_mib'] += rss/2**20
        rc6 = read_int(INTEL_GT/'rc6_residency_ms')
        if rc6 is not None and rc6_before is not None:
            row['intel_gpu_busy_percent'] = max(0., 100.*(1-(rc6-rc6_before)/1000./dt))
        row['intel_gpu_freq_mhz'] = read_int(INTEL_GT/'rps_act_freq_mhz')
        try:
            row['nouveau_runtime_status'] = NOUVEAU.read_text().strip()
        except OSError:
            pass
        line = Path('/proc/stat').read_text().split('\n')[0].split()
        total, idle = sum(map(int, line[1:])), int(line[4])
        row['system_cpu_percent'] = 100.*(1-(idle-idle_before)/max(1, total-cpu_total_before))
        cpu_total_before, idle_before = total, idle
        mem = dict(l.split(':') for l in Path('/proc/meminfo').read_text().splitlines())
        row['system_mem_used_mib'] = (int(mem['MemTotal'].split()[0])-int(mem['MemAvailable'].split()[0]))/1024
        rows.append(row)
        before, t_before, rc6_before = current, now, rc6
    summary = {}
    for group in GROUPS:
        values = [r['groups'][group] for r in rows if group in r['groups']]
        if values:
            summary[group] = {k: {'mean': sum(v[k] for v in values)/len(values), 'max': max(v[k] for v in values)}
                              for k in ('cpu_percent', 'rss_mib', 'intel_gpu_percent')}
    for key in ('system_cpu_percent', 'intel_gpu_busy_percent', 'intel_gpu_freq_mhz', 'system_mem_used_mib'):
        values = [r[key] for r in rows if r.get(key) is not None]
        if values:
            summary[key] = {'mean': sum(values)/len(values), 'max': max(values)}
    summary['nouveau_runtime_status'] = sorted({r.get('nouveau_runtime_status') for r in rows} - {None})
    summary['cpu_cores'] = os.cpu_count()
    with open(a.output, 'w') as f:
        json.dump({'summary': summary, 'samples': rows}, f, indent=1)


if __name__ == '__main__':
    main()
