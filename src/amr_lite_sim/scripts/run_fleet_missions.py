#!/usr/bin/env python3
"""Run simultaneous Nav2 missions for several AMRs through the Gateway (same path as the browser).

Each ``robot=destination`` pair is sent with POST /api/v1/missions at the same time; the engine
state (authoritative) is then polled at 5 Hz until every mission ends. Reported per robot: status,
simulated duration, travelled distance, mean speed, recoveries, final goal error (ground truth), and
whether any other robot moved while it was idle (command isolation). Reported for the fleet:
minimum centre distance between robots, contact events (robot–robot, robot–structure, people),
protective stops and the RTF range. A sequence of rounds is given as several --round arguments.
Usage: run_fleet_missions.py --round amr_01=dock_05 amr_02=pick_south_w2 amr_03=charger_01 --output r.json

Fleet-coordination scenarios (--scenario NAME, repeatable; one continuous recording per scenario):
missions are sent, replaced and cancelled through the same API, and the engine state, the mission
states and the Gateway's station/corridor reservations are recorded at 4 Hz: trajectories, Nav2
plans (sampled), mission status transitions, station ownership and queue changes, the minimum
distance between robot *footprints* (collision boxes, from engine poses; physical separation, not
only the absence of contact events), contact events, time spent in the engine's blocked state,
recoveries, failures and the RTF. Each scenario ends with explicit checks; a queued robot never
counts as arrived. Forklifts are recorded too: their state/reason transitions, restarts within 6 m
of a robot (stop-and-go) and the minimum robot–forklift footprint gap. Scenarios: see SCENARIOS below.
Usage: run_fleet_missions.py --scenario shared_destination --scenario reassign --output s.json
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
    scene = json.loads(urllib.request.urlopen(api+'/api/v1/world/scene', timeout=10).read())
    stations = {s['id']: s for s in scene['stations']}
    FORKLIFT_BOX.update({f['id']: (f['length'], f['width'], f['centre_offset']) for f in scene.get('agents', {}).get('forklifts', ())})
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
            if mission and mission['status'] in ('succeeded', 'failed', 'canceled', 'rejected', 'blocked'):
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


# ---------------------------------------------------------------------------------- scenarios
LENGTH, WIDTH, OFFSET = 0.975, 0.70, 0.0125          # AMR collision box (amr_dimensions.json, as the engine)
DRIVING = ('pending', 'active', 'recovering')
ENDED = ('succeeded', 'failed', 'canceled', 'rejected', 'blocked')
CHARGER_AISLE_ENTRANCE = (63.4, 22.0)


FORKLIFT_BOX = {}                                     # id → (length, width, centre offset), from the scene
CHARGER_AISLE = (61.8, 65.0, 22.0, 46.0)               # navigation.corridors charger_aisle (x0, x1, y0, y1)


def box(e, dims=None):
    length, width, offset = dims or FORKLIFT_BOX.get(e['id']) or (LENGTH, WIDTH, OFFSET)
    c, s = math.cos(e['yaw']), math.sin(e['yaw'])
    cx, cy = e['x']+c*offset, e['y']+s*offset
    return [(cx+c*dx-s*dy, cy+s*dx+c*dy) for dx, dy in ((length/2, width/2), (-length/2, width/2), (-length/2, -width/2), (length/2, -width/2))]


def _inside(p, poly):
    x, y, c = p[0], p[1], False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i+1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < (x2-x1)*(y-y1)/(y2-y1)+x1:
            c = not c
    return c


def _seg(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy or 1.)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def footprint_gap(a, b):
    pa, pb = box(a), box(b)
    if any(_inside(p, pb) for p in pa) or any(_inside(p, pa) for p in pb):
        return 0.
    return min(min(_seg(p, q[i], q[(i+1) % 4]) for p in p_ for i in range(4)) for p_, q in ((pa, pb), (pb, pa)))


class Recorder:
    """Polls /api/v1/world at 4 Hz and keeps everything the scenario report needs."""

    def __init__(self, api, scenario):
        self.api, self.scenario = api, scenario
        _, world = call(api, 'GET', '/api/v1/world')
        self.seen = max((e['seq'] for e in world['events']), default=0)
        self.t0_sim = world['simulation']['sim_time']
        self.samples, self.transitions, self.reservations, self.events, self.plans = [], [], [], [], {}
        self.last_status, self.last_stations, self.last_sim, self.world = {}, None, None, world
        self.min_gap = {'m': math.inf}
        self.blocked_s, self.rtfs, self.trajectories, self.steps = {}, [], {}, []
        self.expected = {}       # robot → id of the mission this scenario sent last
        self.forklift = {'min_gap': {'m': math.inf}, 'transitions': [], 'restarts_near_robot': {}, 'last': {}, 'moving': {}}

    def poll(self):
        _, w = call(self.api, 'GET', '/api/v1/world')
        self.world = w
        sim = w['simulation']['sim_time']
        if sim is None:
            return w
        amrs = {e['id']: e for e in w['entities'] if e['type'] == 'amr'}
        robots = {r['id']: r for r in w['robots']}
        if w['simulation']['real_time_factor'] is not None:
            self.rtfs.append(w['simulation']['real_time_factor'])
        ids = sorted(amrs)
        gaps = {}
        for i, a in enumerate(ids):
            for b in ids[i+1:]:
                g = footprint_gap(amrs[a], amrs[b])
                gaps[f'{a}-{b}'] = round(g, 3)
                if g < self.min_gap['m']:
                    self.min_gap = {'m': g, 'pair': f'{a}-{b}', 'sim': round(sim-self.t0_sim, 2),
                                    'poses': {r: [round(amrs[r][k], 3) for k in ('x', 'y', 'yaw')] for r in (a, b)}}
        dt = sim-self.last_sim if self.last_sim is not None else 0.
        self.last_sim = sim
        for r, e in amrs.items():
            if e['state'] == 'blocked':
                self.blocked_s[r] = self.blocked_s.get(r, 0.)+dt
            traj = self.trajectories.setdefault(r, [])
            if not traj or sim-self.t0_sim-traj[-1][0] >= 1.:
                traj.append([round(sim-self.t0_sim, 2), round(e['x'], 3), round(e['y'], 3), round(e['yaw'], 3), round(e['speed'], 3)])
            m = robots[r]['mission']
            key = (m['id'], m['status'], m.get('waiting_for'), m.get('blocking_robot'), m.get('queue_position')) if m else None
            if key != self.last_status.get(r):
                self.last_status[r] = key
                if m:
                    self.transitions.append({'sim': round(sim-self.t0_sim, 2), 'robot': r, 'mission': m['id'], 'destination': m['destination'],
                                             'status': m['status'], 'waiting_for': m.get('waiting_for'), 'waiting_on': m.get('waiting_on'),
                                             'blocking_robot': m.get('blocking_robot'), 'queue_position': m.get('queue_position'),
                                             'recoveries': m['recoveries'], 'pose': [round(e['x'], 2), round(e['y'], 2)]})
            path = robots[r]['path']
            if path and (r not in self.plans or self.plans[r][-1]['path'][-1] != [round(v, 2) for v in path[-1]] or
                         sim-self.t0_sim-self.plans[r][-1]['sim'] >= 10.):
                self.plans.setdefault(r, []).append({'sim': round(sim-self.t0_sim, 2), 'path': [[round(v, 2) for v in p] for p in path[::8]+[path[-1]]]})
        self._forklifts(w, amrs, sim)
        stations = {'stations': w.get('stations', []), 'corridors': w.get('corridors', [])}
        if stations != self.last_stations:
            self.last_stations = stations
            self.reservations.append({'sim': round(sim-self.t0_sim, 2), **stations})
        for e in w['events']:
            if e['seq'] > self.seen:
                self.seen = e['seq']
                self.events.append(e)
        self.samples.append({'sim': round(sim-self.t0_sim, 2), 'rtf': w['simulation']['real_time_factor'], 'gaps': gaps})
        return w

    def _forklifts(self, w, amrs, sim):
        fk = self.forklift
        for f in (e for e in w['entities'] if e['type'] == 'forklift'):
            key = (f['state'], f.get('reason'))
            if key != fk['last'].get(f['id']):
                fk['last'][f['id']] = key
                fk['transitions'].append({'sim': round(sim-self.t0_sim, 2), 'forklift': f['id'], 'state': key[0], 'reason': key[1],
                                          'pose': [round(f['x'], 2), round(f['y'], 2), round(f['yaw'], 2)]})
            gaps = {r: footprint_gap(a, f) for r, a in amrs.items()}
            nearest = min(gaps.items(), key=lambda kv: kv[1]) if gaps else (None, math.inf)
            if nearest[1] < fk['min_gap']['m']:
                fk['min_gap'] = {'m': nearest[1], 'pair': f"{nearest[0]}-{f['id']}", 'sim': round(sim-self.t0_sim, 2)}
            moving = f['speed'] > .06
            if moving and fk['moving'].get(f['id']) is False and nearest[1] < 6.:
                fk['restarts_near_robot'][f['id']] = fk['restarts_near_robot'].get(f['id'], 0)+1
            fk['moving'][f['id']] = moving

    def forklift_entity(self, fid):
        return next(e for e in self.world['entities'] if e['id'] == fid)

    def mission(self, robot):
        """The robot's mission in the latest world state, only once it is the one this scenario sent
        (the first snapshots after a POST may still carry the previous mission)."""
        m = next(r for r in self.world['robots'] if r['id'] == robot)['mission']
        if robot in self.expected and (m is None or m['id'] != self.expected[robot]):
            return None
        return m

    def amr(self, robot):
        return next(e for e in self.world['entities'] if e['id'] == robot)

    def sim(self):
        return self.world['simulation']['sim_time']-self.t0_sim

    def wait(self, condition, timeout, label):
        """Poll until ``condition(self)`` (wall timeout); returns True when it held."""
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.poll()
            if condition(self):
                self.steps.append({'step': label, 'sim': round(self.sim(), 2), 'ok': True})
                return True
            time.sleep(.25)
        self.steps.append({'step': label, 'sim': round(self.sim(), 2), 'ok': False})
        return False

    def send(self, pairs):
        out = {}
        for robot, destination in pairs.items():
            m = self.mission(robot)
            if m and m['status'] in DRIVING:
                out[robot+':cancel'] = call(self.api, 'DELETE', f'/api/v1/missions/{robot}')[1].get('accepted')
                self.wait(lambda s, r=robot: (s.mission(r) or {}).get('status') not in DRIVING, 30, f'{robot} previous goal cancelled')
            status, reply = call(self.api, 'POST', '/api/v1/missions', {'robot_id': robot, 'destination': destination})
            if reply.get('mission'):
                self.expected[robot] = reply['mission']['id']
            out[robot] = {'destination': destination, 'http': status, 'accepted': reply.get('accepted'), 'reason': reply.get('reason'),
                          'status': (reply.get('mission') or {}).get('status')}
        self.steps.append({'step': 'send', 'sim': round(self.sim(), 2), 'requests': out})
        self.wait(lambda s: all(s.mission(r) is not None for r in pairs if r in s.expected), 10, 'world state shows the new missions')
        return out

    def cancel(self, robot):
        status, reply = call(self.api, 'DELETE', f'/api/v1/missions/{robot}')
        self.steps.append({'step': f'cancel {robot}', 'sim': round(self.sim(), 2), 'http': status, 'accepted': reply.get('accepted'),
                           'reason': reply.get('reason')})
        self.poll()
        return reply

    def report(self, checks):
        contacts = [e for e in self.events if e['kind'] == 'contact']
        robot_contacts = [e for e in contacts if e['other'].startswith('amr_')]
        statuses = {}
        for t in self.transitions:
            statuses.setdefault(t['robot'], []).append(t['status'])
        checks = dict(checks)
        checks['no_robot_robot_contact'] = not robot_contacts
        checks['footprints_never_overlap'] = self.min_gap['m'] > 0.
        return {'scenario': self.scenario, 'passed': all(checks.values()), 'checks': checks,
                'min_footprint_gap': {**self.min_gap, 'm': round(self.min_gap['m'], 3)},
                'contacts': contacts, 'robot_robot_contacts': robot_contacts,
                'blocked_seconds': {r: round(v, 1) for r, v in self.blocked_s.items()},
                'recoveries': {r: max((t['recoveries'] for t in self.transitions if t['robot'] == r), default=0) for r in statuses},
                'failures': [t for t in self.transitions if t['status'] in ('failed', 'blocked', 'rejected')],
                'rtf_min': min(self.rtfs, default=None), 'rtf_mean': round(sum(self.rtfs)/len(self.rtfs), 3) if self.rtfs else None,
                'sim_seconds': round(self.sim(), 1), 'steps': self.steps, 'transitions': self.transitions,
                'status_sequences': statuses, 'reservations': self.reservations, 'events': self.events,
                'trajectories': self.trajectories, 'plans': self.plans, 'gap_samples': self.samples[::4],
                'forklifts': {'min_robot_gap': {**self.forklift['min_gap'], 'm': round(self.forklift['min_gap']['m'], 3)},
                              'restarts_near_robot': self.forklift['restarts_near_robot'],
                              'transitions': self.forklift['transitions']}}


def ended(robots):
    return lambda s: all((s.mission(r) or {}).get('status') in ENDED for r in robots)


def status_is(robot, *statuses):
    return lambda s: (s.mission(robot) or {}).get('status') in statuses


def near(robot, point, distance):
    return lambda s: math.hypot(s.amr(robot)['x']-point[0], s.amr(robot)['y']-point[1]) < distance


def at_station(s, robot, station, stations, radius=1.5):
    p = stations[station]['pose']
    return math.hypot(s.amr(robot)['x']-p[0], s.amr(robot)['y']-p[1]) < radius


def scenario_shared_destination(rec, stations, timeout):
    """All three AMRs to charger_01: one owner drives, two wait in the queue (and stay put)."""
    start = {r: (rec.amr(r)['x'], rec.amr(r)['y']) for r in ('amr_01', 'amr_02', 'amr_03')}
    rec.send({'amr_01': 'charger_01', 'amr_02': 'charger_01', 'amr_03': 'charger_01'})
    queued_at_send = {r: (rec.mission(r) or {}).get('status') for r in ('amr_02', 'amr_03')}
    arrived = rec.wait(status_is('amr_01', *ENDED), timeout, 'owner amr_01 mission ended')
    rec.wait(lambda s: False, 15, 'settle 15 s (queued robots must not move)')
    w = rec.world
    st = next((x for x in w.get('stations', []) if x['id'] == 'charger_01'), {})
    inside = [r for r in ('amr_01', 'amr_02', 'amr_03') if at_station(rec, r, 'charger_01', stations)]
    moved = {r: round(math.hypot(rec.amr(r)['x']-start[r][0], rec.amr(r)['y']-start[r][1]), 3) for r in ('amr_02', 'amr_03')}
    return {'owner_succeeded': arrived and (rec.mission('amr_01') or {}).get('status') == 'succeeded',
            'others_queued_on_request': queued_at_send == {'amr_02': 'waiting', 'amr_03': 'waiting'},
            'others_still_waiting_for_station': all((rec.mission(r) or {}).get('status') == 'waiting' and rec.mission(r)['waiting_for'] == 'station'
                                                    for r in ('amr_02', 'amr_03')),
            'queue_is_fifo': list(st.get('queue', [])) == ['amr_02', 'amr_03'],
            'exactly_one_robot_on_station': inside == ['amr_01'],
            'queued_robots_did_not_move': all(v < .3 for v in moved.values())}, {'moved_m': moved, 'station': st}


def scenario_reassign(rec, stations, timeout):
    """From the shared-destination state: new, different destinations for all three."""
    targets = {'amr_01': 'pick_north_e1', 'amr_02': 'dock_03', 'amr_03': 'receiving_01'}
    replies = rec.send(targets)
    done = rec.wait(ended(targets), timeout, 'all reassigned missions ended')
    w = rec.world
    return {'all_accepted': all(replies[r]['accepted'] for r in targets),
            'all_succeeded': done and all((rec.mission(r) or {}).get('status') == 'succeeded' for r in targets),
            'all_at_their_new_station': all(at_station(rec, r, d, stations, .6) for r, d in targets.items()),
            'charger_01_queue_cleared': not any(x['id'] == 'charger_01' and x['queue'] for x in w.get('stations', []))}, {}


def scenario_head_on(rec, stations, timeout):
    """Head-on through the charger aisle (capacity 1, 3.2 m): amr_01 drives south out of it (from
    charger_04 to dock_07) while amr_02 arrives from the cross aisle to enter it (to charger_02).
    amr_02 starts from production_w: parking it on a pick station puts it in a worker's picking aisle
    (a person-AMR standoff there is outside this robot-robot scenario)."""
    setup = rec.send({'amr_01': 'charger_04', 'amr_02': 'production_w'})
    rec.wait(ended(('amr_01', 'amr_02')), timeout, 'setup: amr_01 at charger_04, amr_02 at production_w')
    ok_setup = all((rec.mission(r) or {}).get('status') == 'succeeded' for r in ('amr_01', 'amr_02'))
    rec.send({'amr_02': 'charger_02'})
    rec.wait(near('amr_02', CHARGER_AISLE_ENTRANCE, 17.), timeout, 'amr_02 17 m from the charger aisle entrance')
    rec.send({'amr_01': 'dock_07'})
    rec.wait(ended(('amr_01', 'amr_02')), timeout, 'both missions ended')
    held = [t for t in rec.transitions if t['robot'] == 'amr_02' and t['status'] == 'waiting' and t['waiting_for'] in ('corridor', 'robot')]
    amr_01_south = any(p[2] < 22. and p[0] > next((x['sim'] for x in rec.steps if x['step'].startswith('amr_02 17 m')), 0)
                       for p in rec.trajectories['amr_01'])
    return {'setup_succeeded': ok_setup, 'amr_01_left_through_the_south_entrance': amr_01_south,
            'amr_02_held_before_the_aisle': bool(held),
            'both_succeeded': all((rec.mission(r) or {}).get('status') == 'succeeded' for r in ('amr_01', 'amr_02'))}, \
        {'holds': held, 'setup': setup}


def scenario_three_missions(rec, stations, timeout):
    """Three different destinations at once (the Phase 7 fleet round)."""
    targets = {'amr_01': 'dock_05', 'amr_02': 'pick_south_w2', 'amr_03': 'charger_02'}
    rec.send(targets)
    done = rec.wait(ended(targets), timeout, 'all missions ended')
    return {'all_succeeded': done and all((rec.mission(r) or {}).get('status') == 'succeeded' for r in targets)}, {}


def scenario_occupied_station(rec, stations, timeout):
    """amr_01 parked at dock_05; amr_02 sent there waits without approaching; amr_01 leaves; amr_02 goes."""
    if not at_station(rec, 'amr_01', 'dock_05', stations):
        rec.send({'amr_01': 'dock_05'})
        rec.wait(ended(('amr_01',)), timeout, 'setup: amr_01 parked at dock_05')
    start = (rec.amr('amr_02')['x'], rec.amr('amr_02')['y'])
    rec.send({'amr_02': 'dock_05'})
    rec.wait(lambda s: False, 20, 'amr_02 waits 20 s')
    m = rec.mission('amr_02') or {}
    waited = m.get('status') == 'waiting' and m.get('waiting_for') == 'station' and m.get('blocking_robot') == 'amr_01'
    moved = math.hypot(rec.amr('amr_02')['x']-start[0], rec.amr('amr_02')['y']-start[1])
    rec.send({'amr_01': 'dock_07'})
    left = rec.wait(lambda s: not at_station(s, 'amr_01', 'dock_05', stations), timeout, 'amr_01 left dock_05')
    dispatched_after_leaving = rec.wait(status_is('amr_02', *DRIVING, *ENDED), 60, 'amr_02 dispatched')
    done = rec.wait(ended(('amr_01', 'amr_02')), timeout, 'both missions ended')
    first_drive = next((t for t in rec.transitions if t['robot'] == 'amr_02' and t['status'] in DRIVING and t['destination'] == 'dock_05'), None)
    left_at = next((x['sim'] for x in rec.steps if x['step'] == 'amr_01 left dock_05'), None)
    return {'amr_02_waited_for_occupied_station': waited, 'amr_02_did_not_approach': moved < .3,
            'amr_01_left': left, 'amr_02_dispatched_only_after_amr_01_left': dispatched_after_leaving and first_drive is not None
            and left_at is not None and first_drive['sim'] >= left_at-0.5,
            'both_succeeded': done and all((rec.mission(r) or {}).get('status') == 'succeeded' for r in ('amr_01', 'amr_02'))}, \
        {'amr_02_moved_while_waiting_m': round(moved, 3), 'first_drive': first_drive}


def scenario_cancellation(rec, stations, timeout):
    """(charger_04: a station no robot occupies in the scenario sequence.) Queued cancel, cancel of the active owner (while it drives at cruise speed) with a waiter behind
    it, and no goal left driving after a cancel."""
    rec.send({'amr_01': 'charger_04', 'amr_03': 'charger_04'})
    queued = (rec.mission('amr_03') or {}).get('status') == 'waiting'
    start = (rec.amr('amr_01')['x'], rec.amr('amr_01')['y'])
    rec.wait(lambda s: math.hypot(s.amr('amr_01')['x']-start[0], s.amr('amr_01')['y']-start[1]) > 2. and s.amr('amr_01')['speed'] > .4,
             timeout, 'amr_01 driving at cruise speed')
    rec.cancel('amr_03')                                      # queued mission cancelled
    canceled_queued = rec.wait(lambda s: (s.mission('amr_03') or {}).get('status') == 'canceled' and not any(
        x['id'] == 'charger_04' and 'amr_03' in x['queue'] for x in s.world.get('stations', [])), 10, 'amr_03 canceled and out of the queue')
    rec.send({'amr_03': 'charger_04'})                        # waits again behind the owner
    requeued = (rec.mission('amr_03') or {}).get('status') == 'waiting'
    speed_at_cancel = rec.amr('amr_01')['speed']
    rec.cancel('amr_01')                                      # active owner cancelled while driving
    owner_canceled = rec.wait(status_is('amr_01', 'canceled'), 30, 'amr_01 canceled')
    rec.wait(lambda s: abs(s.amr('amr_01')['speed']) < .02, 20, 'amr_01 stopped')
    stop_pose = (rec.amr('amr_01')['x'], rec.amr('amr_01')['y'])
    rec.wait(lambda s: False, 10, '10 s after the stop')
    drift = math.hypot(rec.amr('amr_01')['x']-stop_pose[0], rec.amr('amr_01')['y']-stop_pose[1])
    waiter_went = rec.wait(status_is('amr_03', *DRIVING, *ENDED), 30, 'amr_03 dispatched after the owner cancel')
    done = rec.wait(ended(('amr_03',)), timeout, 'amr_03 mission ended')
    return {'second_robot_queued': queued, 'queued_cancel_removes_from_queue': canceled_queued, 'requeued_behind_owner': requeued,
            'owner_canceled_while_driving': owner_canceled and speed_at_cancel > .4, 'no_goal_keeps_driving_after_cancel': drift < .05,
            'waiter_dispatched_after_owner_cancel': waiter_went,
            'waiter_succeeded': done and (rec.mission('amr_03') or {}).get('status') == 'succeeded'}, \
        {'speed_at_cancel_m_s': round(speed_at_cancel, 3), 'drift_after_stop_m': round(drift, 3)}


def scenario_single_mission(rec, stations, timeout):
    """Single-robot regression: one mission of amr_01."""
    rec.send({'amr_01': 'dock_05'})
    done = rec.wait(ended(('amr_01',)), timeout, 'mission ended')
    return {'succeeded': done and (rec.mission('amr_01') or {}).get('status') == 'succeeded',
            'at_station': at_station(rec, 'amr_01', 'dock_05', stations, .6)}, {}


def in_charger_aisle_mouth(f):
    """The forklift's body overlaps the charger aisle (the only way out of it)."""
    xs, ys = zip(*box(f))
    x0, x1, y0, y1 = CHARGER_AISLE
    return max(xs) > x0 and min(xs) < x1 and max(ys) > y0 and min(ys) < y1


def scenario_forklift_parked_reassign(rec, stations, timeout):
    """The EMP-1 / amr_01 report: amr_01 parked at charger_01 when EMP-1 reaches its U-turn at the
    charger aisle mouth; then pick_south_e1, cancelled after 6 s of simulation, and dock_05. Checks: the
    forklift never stands in the aisle mouth while held, the robot gets out and arrives, no
    stop-and-go (one restart per encounter), robot–forklift gap ≥ the 0.3 m vehicle margin."""
    if not at_station(rec, 'amr_01', 'charger_01', stations, .6):
        rec.send({'amr_01': 'charger_01'})
        rec.wait(ended(('amr_01',)), timeout, 'amr_01 parked at charger_01')
    held = {'since': None, 'mouth': False}

    def emp_held(s):
        f = s.forklift_entity('EMP-1')
        if f['speed'] < .06 and math.hypot(f['x']-63.4, f['y']-24.) < 6.:
            held['mouth'] |= in_charger_aisle_mouth(f)
            held['since'] = held['since'] if held['since'] is not None else s.sim()
            return s.sim()-held['since'] >= 35.
        held['since'] = None
        return False
    waited = rec.wait(emp_held, timeout, 'EMP-1 held by the parked robot for 35 s')
    rec.forklift['restarts_near_robot'].clear()
    rec.send({'amr_01': 'pick_south_e1'})
    t0 = rec.sim()
    rec.wait(lambda s: s.sim()-t0 >= 6., 120, '6 s of simulation on the way to pick_south_e1')
    rec.cancel('amr_01')
    rec.wait(lambda s: (s.mission('amr_01') or {}).get('status') not in DRIVING, 30, 'pick_south_e1 cancelled')
    rec.send({'amr_01': 'dock_05'})
    done = rec.wait(ended(('amr_01',)), timeout, 'dock_05 ended')
    forklift_contacts = [e for e in rec.events if e['kind'] == 'contact' and e['other'].startswith('EMP')]
    return {'emp1_held_by_parked_robot': waited,
            'emp1_never_held_in_aisle_mouth': not held['mouth'],
            'dock_05_succeeded': done and (rec.mission('amr_01') or {}).get('status') == 'succeeded',
            'at_dock_05': at_station(rec, 'amr_01', 'dock_05', stations, .6),
            'no_forklift_contact': not forklift_contacts,
            'forklift_gap_above_margin': rec.forklift['min_gap']['m'] >= .3,
            'emp1_at_most_one_restart_per_encounter': rec.forklift['restarts_near_robot'].get('EMP-1', 0) <= 1}, \
        {'vehicle_wait_events': [e for e in rec.events if e['kind'] in ('vehicle_wait', 'vehicle_resumed')]}


SCENARIOS = {'shared_destination': scenario_shared_destination, 'reassign': scenario_reassign, 'head_on': scenario_head_on,
             'three_missions': scenario_three_missions, 'occupied_station': scenario_occupied_station,
             'cancellation': scenario_cancellation, 'single_mission': scenario_single_mission,
             'forklift_parked_reassign': scenario_forklift_parked_reassign}


def run_scenario(api, name, timeout, checkpoint):
    scene = json.loads(urllib.request.urlopen(api+'/api/v1/world/scene', timeout=10).read())
    stations = {s['id']: s for s in scene['stations']}
    FORKLIFT_BOX.update({f['id']: (f['length'], f['width'], f['centre_offset']) for f in scene.get('agents', {}).get('forklifts', ())})
    rec = Recorder(api, name)
    checks, extra = SCENARIOS[name](rec, stations, timeout)
    if checkpoint:
        import subprocess
        subprocess.run(checkpoint.format(name=name), shell=True, timeout=120)
    report = rec.report(checks)
    report['extra'] = extra
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--round', nargs='+', action='append', metavar='ROBOT=DESTINATION')
    p.add_argument('--scenario', action='append', choices=sorted(SCENARIOS))
    p.add_argument('--api', default='http://127.0.0.1:8000')
    p.add_argument('--timeout', type=float, default=900)
    p.add_argument('--checkpoint', default='', help="shell command run at the end of each scenario ({name} = scenario)")
    p.add_argument('--output', required=True)
    a = p.parse_args()
    if not a.round and not a.scenario:
        p.error('give --round and/or --scenario')
    rounds, scenarios = [], []
    for spec in a.round or []:
        pairs = [tuple(item.split('=', 1)) for item in spec]
        result = run_round(a.api, pairs, a.timeout)
        summary = {r: (m.get('status'), m.get('sim_seconds'), m.get('distance_m'), m.get('goal_error_m')) for r, m in result['missions'].items()}
        print(json.dumps({'round': spec, 'missions': summary, 'min_gap_m': result['min_robot_centre_distance_m'],
                          'contacts': len(result['contacts']), 'idle_moved': result['idle_robots_moved_m']}))
        rounds.append(result)
    for name in a.scenario or []:
        report = run_scenario(a.api, name, a.timeout, a.checkpoint)
        print(json.dumps({'scenario': name, 'passed': report['passed'], 'checks': report['checks'],
                          'min_footprint_gap_m': report['min_footprint_gap']['m'], 'robot_contacts': len(report['robot_robot_contacts']),
                          'rtf_mean': report['rtf_mean'], 'sim_s': report['sim_seconds']}), flush=True)
        scenarios.append(report)
        with open(a.output, 'w') as out:
            json.dump({'rounds': rounds, 'scenarios': scenarios}, out, indent=1)
    with open(a.output, 'w') as out:
        json.dump({'rounds': rounds, 'scenarios': scenarios}, out, indent=1)


if __name__ == '__main__':
    main()
