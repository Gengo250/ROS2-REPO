#!/usr/bin/env python3
"""Run simultaneous Nav2 missions for several AMRs through the Gateway (same path as the browser).

Each ``robot=destination`` pair is sent with POST /api/v1/missions at the same time; the engine
state (authoritative) is then polled at 5 Hz until every mission ends. Reported per robot: status,
simulated duration, travelled distance, mean speed, recoveries, final goal error (ground truth), and
whether any other robot moved while it was idle (command isolation). Reported for the fleet:
minimum centre distance between robots, contact events (robot–robot, robot–structure, people),
protective stops and the RTF range. A sequence of rounds is given as several --round arguments.
Usage: run_fleet_missions.py --round amr_01=dock_05 amr_02=pick_south_w2 amr_03=charger_01 --output r.json
"""
import argparse
import itertools
import json
import math
import time
import urllib.request


def call(api, method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    request = urllib.request.Request(api+path, data=data, method=method, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read() or b'{}')


def run_round(api, pairs, timeout):
    _, world = call(api, 'GET', '/api/v1/world')
    seen = max((e['seq'] for e in world['events']), default=-1)
    amrs = {e['id']: e for e in world['entities'] if e['type'] == 'amr'}
    start = {r: (amrs[r]['x'], amrs[r]['y']) for r in amrs}
    stations = {s['id']: s for s in json.loads(urllib.request.urlopen(api+'/api/v1/world/scene', timeout=10).read())['stations']}
    result = {'missions': {}, 'idle_robots_moved_m': {}}
    for robot, destination in pairs:
        status, reply = call(api, 'POST', '/api/v1/missions', {'robot_id': robot, 'destination': destination})
        result['missions'][robot] = {'destination': destination, 'http': status, 'accepted': reply.get('accepted'),
                                     'reason': reply.get('reason'), 'distance_m': 0., 'trajectory': []}
    active = {r for r, m in result['missions'].items() if m['accepted']}
    idle = set(amrs)-{r for r, _ in pairs}
    t0_wall, t0_sim, last, min_gap, rtfs, events = time.monotonic(), world['simulation']['sim_time'], {}, math.inf, [], []
    while active and time.monotonic()-t0_wall < timeout:
        time.sleep(.2)
        _, world = call(api, 'GET', '/api/v1/world')
        rtfs.append(world['simulation']['real_time_factor'])
        poses = {e['id']: e for e in world['entities'] if e['type'] == 'amr'}
        for a, b in itertools.combinations(sorted(poses), 2):
            min_gap = min(min_gap, math.hypot(poses[a]['x']-poses[b]['x'], poses[a]['y']-poses[b]['y']))
        for robot, entry in result['missions'].items():
            e = poses[robot]
            if robot in last:
                entry['distance_m'] += math.hypot(e['x']-last[robot][0], e['y']-last[robot][1])
            last[robot] = (e['x'], e['y'])
            if len(entry['trajectory']) == 0 or world['simulation']['sim_time']-entry['trajectory'][-1][0] >= 1.:
                entry['trajectory'].append([round(world['simulation']['sim_time'], 2), round(e['x'], 3), round(e['y'], 3)])
        for robot in idle:
            e = poses[robot]
            result['idle_robots_moved_m'][robot] = round(max(result['idle_robots_moved_m'].get(robot, 0.),
                                                             math.hypot(e['x']-start[robot][0], e['y']-start[robot][1])), 4)
        for e in world['events']:
            if e['seq'] > seen:
                seen = e['seq']
                events.append(e)
        for r in list(active):
            mission = next(x for x in world['robots'] if x['id'] == r)['mission']
            if mission and mission['status'] in ('succeeded', 'failed', 'canceled', 'rejected'):
                entry = result['missions'][r]
                goal = stations[entry['destination']]['pose']
                e = poses[r]
                entry.update(status=mission['status'], detail=mission['detail'], recoveries=mission['recoveries'],
                             sim_seconds=round(world['simulation']['sim_time']-t0_sim, 1),
                             goal_error_m=round(math.hypot(e['x']-goal[0], e['y']-goal[1]), 3))
                entry['mean_speed_m_s'] = round(entry['distance_m']/max(entry['sim_seconds'], 1e-6), 3)
                active.discard(r)
    for r in active:
        result['missions'][r]['status'] = 'timeout'
    kinds = {}
    for e in events:
        kinds[e['kind']] = kinds.get(e['kind'], 0)+1
    contacts = [e for e in events if e['kind'] == 'contact']
    result.update(min_robot_centre_distance_m=round(min_gap, 3), events=kinds,
                  robot_robot_contacts=[e for e in contacts if e['other'].startswith('amr_')],
                  contacts=contacts, rtf_min=min(rtfs) if rtfs else None, rtf_max=max(rtfs) if rtfs else None,
                  wall_seconds=round(time.monotonic()-t0_wall, 1))
    for entry in result['missions'].values():
        entry['distance_m'] = round(entry['distance_m'], 2)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--round', nargs='+', action='append', required=True, metavar='ROBOT=DESTINATION')
    p.add_argument('--api', default='http://127.0.0.1:8000')
    p.add_argument('--timeout', type=float, default=900)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    rounds = []
    for spec in a.round:
        pairs = [tuple(item.split('=', 1)) for item in spec]
        result = run_round(a.api, pairs, a.timeout)
        summary = {r: (m.get('status'), m.get('sim_seconds'), m.get('distance_m'), m.get('goal_error_m')) for r, m in result['missions'].items()}
        print(json.dumps({'round': spec, 'missions': summary, 'min_gap_m': result['min_robot_centre_distance_m'],
                          'contacts': len(result['contacts']), 'idle_moved': result['idle_robots_moved_m']}))
        rounds.append(result)
    with open(a.output, 'w') as out:
        json.dump({'rounds': rounds}, out, indent=1)


if __name__ == '__main__':
    main()
