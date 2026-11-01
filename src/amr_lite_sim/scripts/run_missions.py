#!/usr/bin/env python3
"""Drive the AMR through station missions via the Gateway (the same path as the browser).

For each destination: POST /api/v1/missions, then poll /api/v1/world until the mission ends,
recording wall/sim duration, travelled distance (engine pose), mean speed, field events and
contacts. Optional --obstacle-after/--obstacle places a user obstacle on the robot's planned
path mid-mission to exercise detection, slow-down/stop and replanning.
Usage: run_missions.py dock_05 charger_01 ... --output report.json [--obstacle pallet --obstacle-after 6]
"""
import argparse
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


def main():
    p = argparse.ArgumentParser()
    p.add_argument('destinations', nargs='+')
    p.add_argument('--api', default='http://127.0.0.1:8000')
    p.add_argument('--output', required=True)
    p.add_argument('--timeout', type=float, default=600)
    p.add_argument('--obstacle', choices=['cone', 'box', 'pallet'])
    p.add_argument('--obstacle-after', type=float, default=8., help='seconds into each mission')
    p.add_argument('--obstacle-ahead', type=float, default=5., help='metres ahead along the plan')
    p.add_argument('--obstacle-at', type=float, nargs=2, help='explicit world position instead of the plan look-ahead')
    a = p.parse_args()
    results = []
    for destination in a.destinations:
        # The snapshot carries a ring of recent events: count only those newer than this mission.
        _, world = call(a.api, 'GET', '/api/v1/world')
        seen = max((e['seq'] for e in world['events']), default=-1)
        status, reply = call(a.api, 'POST', '/api/v1/missions', {'destination': destination})
        entry = {'destination': destination, 'http': status, 'accepted': reply.get('accepted'), 'reason': reply.get('reason')}
        if not reply.get('accepted'):
            results.append(entry)
            print(json.dumps(entry))
            continue
        start_wall, start_sim, last, distance, speeds, events, final = time.monotonic(), None, None, 0., [], [], None
        trajectory = []
        placed = None
        while time.monotonic()-start_wall < a.timeout:
            time.sleep(.2)
            _, world = call(a.api, 'GET', '/api/v1/world')
            robot = next(e for e in world['entities'] if e['id'] == 'amr_01')
            sim = world['simulation']['sim_time']
            start_sim = start_sim if start_sim is not None else sim
            if last is not None:
                distance += math.dist(last, (robot['x'], robot['y']))
            last = (robot['x'], robot['y'])
            trajectory.append((round(time.monotonic()-start_wall, 2), round(robot['x'], 3), round(robot['y'], 3), round(robot['speed'], 3), robot['state']))
            speeds.append(robot['speed'])
            for e in world['events']:
                if e['kind'] in ('protective_field', 'contact') and e['seq'] > seen and e not in events:
                    events.append(e)
            if a.obstacle and placed is None and time.monotonic()-start_wall > a.obstacle_after:
                path = world['robots'][0]['path']
                if a.obstacle_at:
                    path = [list(a.obstacle_at)]
                if path:
                    # The plan point a.obstacle_ahead metres ahead of the robot.
                    nearest = min(range(len(path)), key=lambda i: math.dist(path[i], last)) if not a.obstacle_at else 0
                    walked, target = 0., path[-1]
                    for q0, q1 in zip(path[nearest:], path[nearest+1:]):
                        walked += math.dist(q0, q1)
                        if walked >= a.obstacle_ahead:
                            target = q1
                            break
                    _, reply = call(a.api, 'POST', '/api/v1/obstacles', {'kind': a.obstacle, 'x': target[0], 'y': target[1]})
                    placed = {'at': target, 'accepted': reply.get('accepted'), 'reason': reply.get('reason'),
                              'id': (reply.get('obstacle') or {}).get('id'), 'mission_time_s': time.monotonic()-start_wall}
            mission = world['robots'][0]['mission']
            if mission and mission['status'] not in ('pending', 'active'):
                final = mission
                break
        wall, sim = time.monotonic()-start_wall, world['simulation']['sim_time']-start_sim
        goal = final['goal'] if final else None
        entry.update({'status': final['status'] if final else 'timeout', 'detail': final['detail'] if final else None,
                      'recoveries': final['recoveries'] if final else None, 'wall_s': wall, 'sim_s': sim,
                      'distance_m': distance, 'mean_speed_wall_m_s': distance/wall if wall else 0.,
                      'max_speed_m_s': max(speeds, default=0.), 'final_position_error_m': math.dist(last, goal[:2]) if goal else None,
                      'protective_field_events': sum(1 for e in events if e['kind'] == 'protective_field'),
                      'contacts': [e for e in events if e['kind'] == 'contact'], 'obstacle': placed,
                      'rtf': world['simulation']['real_time_factor']})
        if placed and placed.get('accepted'):
            after = [t for t in trajectory if t[0] >= placed['mission_time_s']]
            ox, oy = placed['at']
            placed['min_centre_distance_m'] = min((math.dist((x, y), (ox, oy)) for _, x, y, _, _ in after), default=None)
            placed['stopped_samples'] = sum(1 for t in after if abs(t[3]) < .02)
        entry['trajectory'] = trajectory
        if placed and placed.get('id'):
            call(a.api, 'DELETE', f"/api/v1/obstacles/{placed['id']}")
        results.append(entry)
        print(json.dumps({k: entry[k] for k in ('destination', 'status', 'wall_s', 'sim_s', 'distance_m', 'mean_speed_wall_m_s',
                                                 'final_position_error_m', 'protective_field_events', 'recoveries')}))
        time.sleep(2)
    json.dump(results, open(a.output, 'w'), indent=1)


if __name__ == '__main__':
    main()
