"""Fleet traffic coordination for the AMR missions: stations, narrow corridors, persistent conflicts.

Pure decision logic (no ROS, no asyncio): the Gateway's mission commands feed it the latest engine
state (backend-authoritative poses, simulation time), the missions and the robots' Nav2 plans, and
execute the actions it returns. It never commands a velocity: every action is a Nav2 goal sent,
cancelled (hold, block) or sent again (resume), so each robot's command guard keeps sole ownership
of its cmd_vel.

Stations (scene ``stations``) are single-capacity resources:
  * one owner at a time (the robot whose mission to it is under way);
  * a bounded FIFO queue of waiting missions (deterministic, no starvation, no barging);
  * a station stays occupied while any robot's footprint is inside its occupancy disc, even after
    its mission succeeded; it is free only when that robot has physically left;
  * a circular wait (A waits for B's station while B waits for A's) is broken by dispatching the
    highest-priority robot of the cycle; it leaves, which frees the next one's station.

Narrow corridors (scene ``navigation.corridors`` with capacity 1, e.g. rack aisles, charger aisle):
a robot whose Nav2 plan enters one within ``lookahead`` metres of path claims it if no *driving*
robot is inside or has claimed it; otherwise its goal is held (cancelled) before the entrance and
sent again once the corridor is clear. Parked robots do not hold anyone (waiting for them would
never end; the planner routes around them with the fleet costmap layer).

Persistent conflicts: a driving robot that makes no progress for ``stall_seconds`` of simulation
time (so the real-time factor does not matter) with another AMR within ``conflict_gap`` of its
footprint (or in contact) is in a robot conflict; a brief stop never qualifies. Two driving robots
blocking each other: the lower-priority one yields (held until the other has passed) up to
``max_yields`` times per mission; if the winner still cannot pass the yielded robot the roles are
swapped once; if that fails too, or a robot is stuck behind a parked one for ``block_seconds``,
the robots are stopped and reported blocked instead of retrying unsafe motion.

Priority: older mission first (creation time), then robot id. Every rule is deterministic.
"""
from dataclasses import dataclass, field
import math

DRIVING = ('pending', 'active', 'recovering')
LENGTH, WIDTH, OFFSET = 0.975, 0.70, 0.0125          # AMR collision box (amr_dimensions.json, as the engine)


@dataclass(frozen=True)
class Corridor:
    id: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    capacity: int

    @classmethod
    def from_scene(cls, c):
        (cx, cy), (sx, sy) = c['centre'], c['size']
        return cls(c['id'], cx-sx/2, cx+sx/2, cy-sy/2, cy+sy/2, int(c.get('capacity', 1)))

    def contains(self, x, y, margin=0.):
        return self.x_min-margin <= x <= self.x_max+margin and self.y_min-margin <= y <= self.y_max+margin


@dataclass
class Hold:
    kind: str                # 'corridor' | 'robot'
    on: str                  # corridor id or robot id
    blocker: str | None      # robot the hold waits for
    since: float             # simulation time
    swapped: bool = False


@dataclass
class Progress:
    mission: str
    x: float
    y: float
    since: float
    best: float | None


@dataclass
class Waiting:
    """Why a mission waits (station queue or a hold), for the mission state shown to users."""
    kind: str                # 'station' | 'corridor' | 'robot'
    on: str
    blocker: str | None = None
    position: int | None = None


def box(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    cx, cy = x+c*OFFSET, y+s*OFFSET
    return [(cx+c*dx-s*dy, cy+s*dx+c*dy) for dx, dy in ((LENGTH/2, WIDTH/2), (-LENGTH/2, WIDTH/2),
                                                        (-LENGTH/2, -WIDTH/2), (LENGTH/2, -WIDTH/2))]


def _inside(p, poly):
    x, y, c = p[0], p[1], False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i+1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < (x2-x1)*(y-y1)/(y2-y1)+x1:
            c = not c
    return c


def _segment_distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy or 1.)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def footprint_gap(a, b):
    """Distance between two AMR collision boxes (0 when they touch or overlap)."""
    pa, pb = box(a.x, a.y, a.yaw), box(b.x, b.y, b.yaw)
    if any(_inside(p, pb) for p in pa) or any(_inside(p, pa) for p in pb):
        return 0.
    return min(min(_segment_distance(p, q[i], q[(i+1) % 4]) for p in p_ for i in range(4))
               for p_, q in ((pa, pb), (pb, pa)))


class Traffic:
    def __init__(self, stations, corridors=(), occupancy_radius=1.5, queue_limit=4, lookahead=6.,
                 stall_seconds=12., block_seconds=30., conflict_gap=1.0, clear_gap=2.0, min_hold=2.,
                 max_yields=2, plan_max_age=5.):
        """``stations``: id → (x, y, yaw) world pose; ``corridors``: Corridor list (only capacity 1 is gated).
        ``occupancy_radius``: robot centre within this distance of the station pose = station occupied
        (circumscribed radius 0.6 m of the box + 0.9 m clearance)."""
        self.stations = {k: tuple(v[:2]) for k, v in stations.items()}
        self.corridors = [c for c in corridors if c.capacity == 1]
        self.occupancy_radius, self.queue_limit, self.lookahead = occupancy_radius, queue_limit, lookahead
        self.stall_seconds, self.block_seconds = stall_seconds, block_seconds
        self.conflict_gap, self.clear_gap, self.min_hold = conflict_gap, clear_gap, min_hold
        self.max_yields, self.plan_max_age = max_yields, plan_max_age
        self.owners, self.queues, self.claims = {}, {}, {}
        self.holds, self.progress, self.yields = {}, {}, {}

    # ------------------------------------------------------------------ stations
    def occupants(self, station, amrs):
        sx, sy = self.stations[station]
        return sorted(r for r, e in amrs.items() if math.hypot(e.x-sx, e.y-sy) < self.occupancy_radius)

    def request(self, robot, station, amrs):
        """Atomic reservation decision for a new mission: ('dispatch', None), ('queue', Waiting) or
        ('reject', reason). Dispatch only when nobody owns the station, nobody waits for it and no
        other robot is inside its occupancy disc."""
        self.withdraw(robot)
        if station not in self.stations:
            return 'reject', f'unknown destination {station!r}'
        queue = self.queues.setdefault(station, [])
        others = [o for o in self.occupants(station, amrs) if o != robot]
        if self.owners.get(station) in (None, robot) and not queue and not others:
            self.owners[station] = robot
            return 'dispatch', None
        if len(queue) >= self.queue_limit:
            return 'reject', f'station {station} queue full ({self.queue_limit})'
        queue.append(robot)
        return 'queue', self._station_waiting(robot, station, amrs)

    def _station_waiting(self, robot, station, amrs):
        queue = self.queues.get(station, [])
        owner = self.owners.get(station)
        occupants = [o for o in self.occupants(station, amrs) if o != robot]
        blocker = owner if owner not in (None, robot) else (occupants[0] if occupants else (queue[0] if queue and queue[0] != robot else None))
        return Waiting('station', station, blocker, queue.index(robot)+1 if robot in queue else None)

    def withdraw(self, robot):
        """Forget every reservation, claim, hold and progress record of ``robot`` (mission cancelled,
        replaced or ended). Its physical presence is still respected through the engine poses."""
        for queue in self.queues.values():
            if robot in queue:
                queue.remove(robot)
        for table in (self.owners, self.claims):
            for key in [k for k, v in table.items() if v == robot]:
                del table[key]
        self.holds.pop(robot, None)
        self.progress.pop(robot, None)

    mission_ended = withdraw

    def priority(self, robot, missions):
        m = missions.get(robot)
        return (m.created_at.timestamp() if m else math.inf, robot)

    def _cycle(self, head, station, amrs, missions):
        """Robots of a circular station wait through ``head`` (empty when there is none)."""
        chain, robot, target = [head], head, station
        for _ in range(len(amrs)+1):
            occupants = [o for o in self.occupants(target, amrs) if o != robot]
            if not occupants:
                return []
            nxt = occupants[0]
            if nxt == head:
                return chain
            m = missions.get(nxt)
            if nxt in chain or not m or m.status != 'waiting' or nxt not in self.queues.get(m.destination, []):
                return []
            chain.append(nxt)
            robot, target = nxt, m.destination
        return []

    # ------------------------------------------------------------------ corridors
    def _inside(self, e):
        return {c.id for c in self.corridors if c.contains(e.x, e.y)}

    def _approaching(self, e, points):
        """First capacity-1 corridor (not already containing the robot) that the plan enters within
        ``lookahead`` metres of path from the point nearest to the robot."""
        if not points:
            return None
        inside = self._inside(e)
        start = min(range(len(points)), key=lambda i: (points[i][0]-e.x)**2+(points[i][1]-e.y)**2)
        travelled, (px, py) = 0., (e.x, e.y)
        for x, y in points[start:]:
            travelled += math.hypot(x-px, y-py)
            px, py = x, y
            if travelled > self.lookahead:
                return None
            for c in self.corridors:
                if c.id not in inside and c.contains(x, y):
                    return c.id
        return None

    # ------------------------------------------------------------------ one coordination step
    def step(self, sim, entities, missions, plans):
        """One decision round. ``entities``: engine entities; ``missions``: robot → current Mission;
        ``plans``: robot → (age seconds, world points). Returns (actions, waiting):
        actions: ('dispatch', robot) | ('hold', robot, Waiting) | ('resume', robot) | ('block', robot, Waiting)
        waiting: robot → Waiting for every mission queued for a station."""
        amrs = {e.id: e for e in entities if e.type == 'amr'}
        actions = []
        status = {r: m.status for r, m in missions.items() if m is not None}
        prio = lambda r: self.priority(r, missions)
        driving = lambda r: status.get(r) in DRIVING and r not in self.holds

        # Stale bookkeeping: anything whose mission is no longer open is released.
        for robot in [r for r in self.holds if status.get(r) != 'waiting']:
            self.holds.pop(robot)
        for station, owner in list(self.owners.items()):
            m = missions.get(owner)
            if not m or m.destination != station or m.status not in DRIVING+('waiting',) or owner in self.queues.get(station, []):
                del self.owners[station]
        for station, queue in self.queues.items():
            queue[:] = [r for r in queue if status.get(r) == 'waiting' and missions[r].destination == station and r not in self.holds]

        # 1. Stations: the queue head goes when the station is free (strict), or to break a cycle.
        for station, queue in sorted(self.queues.items()):
            if not queue or self.owners.get(station):
                continue
            head = queue[0]
            if [o for o in self.occupants(station, amrs) if o != head]:
                cycle = self._cycle(head, station, amrs, missions)
                if not cycle or min(cycle, key=prio) != head:
                    continue
            queue.pop(0)
            self.owners[station] = head
            actions.append(('dispatch', head))
        waiting = {r: self._station_waiting(r, s, amrs) for s, q in self.queues.items() for r in q}
        dispatched = {a[1] for a in actions}

        # 2. Narrow corridors: claim before entering, or hold before the entrance.
        inside = {r: self._inside(e) for r, e in amrs.items()}
        for c in list(self.claims):
            r = self.claims[c]
            if not (driving(r) or (r in self.holds and self.holds[r].kind == 'corridor')):
                del self.claims[c]
        candidates = [r for r in amrs if (driving(r) and r not in dispatched) or (r in self.holds and self.holds[r].kind == 'corridor')]
        for robot in sorted(candidates, key=prio):
            e, plan = amrs[robot], plans.get(robot)
            held = self.holds.get(robot)
            fresh = plan is not None and (held is not None or plan[0] <= self.plan_max_age)
            target = self._approaching(e, plan[1]) if fresh else None
            for c, r in list(self.claims.items()):
                if r == robot and c != target and c not in inside[robot]:
                    del self.claims[c]
            if target is None:
                if held:
                    if sim-held.since >= self.min_hold:
                        self.holds.pop(robot)
                        actions.append(('resume', robot))
                continue
            # Inside robots with an open mission (driving, or held there) will drive through it.
            blockers = sorted(o for o in amrs if o != robot and target in inside[o] and (driving(o) or o in self.holds))
            claimant = self.claims.get(target)
            if claimant not in (None, robot) and claimant not in blockers:
                blockers.append(claimant)
            # Two robots each held for the other's corridor: the higher-priority one goes.
            blockers = [o for o in blockers if not (o in self.holds and self.holds[o].blocker == robot and prio(robot) < prio(o))]
            if blockers:
                if not held:
                    info = Waiting('corridor', target, blockers[0])
                    self.holds[robot] = Hold('corridor', target, blockers[0], sim)
                    actions.append(('hold', robot, info))
                continue
            self.claims[target] = robot
            if held and sim-held.since >= self.min_hold:
                self.holds.pop(robot)
                actions.append(('resume', robot))

        # 3. Robot-to-robot yields: release when the robot yielded to has passed or stopped driving.
        for robot, h in list(self.holds.items()):
            if h.kind != 'robot' or sim-h.since < self.min_hold:
                continue
            winner = amrs.get(h.on)
            if winner is None or not driving(h.on) or footprint_gap(winner, amrs[robot]) > self.clear_gap:
                self.holds.pop(robot)
                actions.append(('resume', robot))

        # 4. Persistent conflicts (simulation time; a brief stop never qualifies).
        acted = {a[1] for a in actions}
        for robot in sorted(amrs, key=prio):
            m = missions.get(robot)
            if not driving(robot) or robot in acted:
                if not driving(robot):
                    self.progress.pop(robot, None)
                continue
            e = amrs[robot]
            p = self.progress.get(robot)
            remaining = m.distance_remaining_m
            if p is None or p.mission != m.id:
                self.progress[robot] = Progress(m.id, e.x, e.y, sim, remaining)
                continue
            if math.hypot(e.x-p.x, e.y-p.y) > .3 or (remaining is not None and p.best is not None and remaining < p.best-.3):
                self.progress[robot] = Progress(m.id, e.x, e.y, sim, remaining if p.best is None or remaining is None else min(remaining, p.best))
                continue
            stalled = sim-p.since
            if stalled < self.stall_seconds:
                continue
            blocker = self._blocker(e, amrs)
            if blocker is None:
                continue                          # not a robot conflict: Nav2's own recovery applies
            other = self.holds.get(blocker)
            if driving(blocker):
                q = self.progress.get(blocker)
                if q is None or sim-q.since < self.stall_seconds/2:
                    continue                      # the other robot is still moving: temporary
                loser, winner = sorted((robot, blocker), key=prio)[::-1]
                if self.yields.get(missions[loser].id, 0) >= self.max_yields:
                    actions += self._block((robot, blocker), sim, amrs, f'{robot}+{blocker}')
                else:
                    self.yields[missions[loser].id] = self.yields.get(missions[loser].id, 0)+1
                    self.holds[loser] = Hold('robot', winner, winner, sim)
                    self.progress.pop(winner, None)
                    actions.append(('hold', loser, Waiting('robot', winner, winner)))
                acted |= {robot, blocker}
            elif other is not None and other.kind == 'robot' and other.on == robot and sim-other.since >= self.stall_seconds:
                # The robot that yielded is in the way of the one it yielded to: swap once.
                if other.swapped:
                    actions += self._block((robot, blocker), sim, amrs, f'{robot}+{blocker}')
                else:
                    self.holds.pop(blocker)
                    actions.append(('resume', blocker))
                    self.holds[robot] = Hold('robot', blocker, blocker, sim, swapped=True)
                    self.progress.pop(blocker, None)
                    actions.append(('hold', robot, Waiting('robot', blocker, blocker)))
                acted |= {robot, blocker}
            elif stalled >= self.block_seconds:
                actions += self._block((robot,), sim, amrs, blocker)
                acted.add(robot)
        return actions, waiting

    def _blocker(self, e, amrs):
        contact = (e.reason or '').removeprefix('contact:') if (e.reason or '').startswith('contact:') else None
        if contact in amrs:
            return contact
        near = [(footprint_gap(e, o), r) for r, o in amrs.items() if r != e.id]
        near = [(g, r) for g, r in near if g < self.conflict_gap]
        return min(near)[1] if near else None

    def _block(self, robots, sim, amrs, blocker):
        out = []
        for robot in robots:
            self.holds.pop(robot, None)
            self.progress.pop(robot, None)
            other = next((r for r in robots if r != robot), blocker)
            out.append(('block', robot, Waiting('robot', other, other)))
        return out

    # ------------------------------------------------------------------ observability
    def describe(self, entities):
        """Station ownership, occupancy and queues, and corridor claims (non-empty entries only)."""
        amrs = {e.id: e for e in entities if e.type == 'amr'}
        stations = []
        for station in sorted(self.stations):
            owner, queue, occupants = self.owners.get(station), tuple(self.queues.get(station, ())), tuple(self.occupants(station, amrs))
            if owner or queue or occupants:
                stations.append(dict(id=station, owner=owner, occupants=occupants, queue=queue))
        corridors = []
        for c in self.corridors:
            occupants = tuple(sorted(r for r, e in amrs.items() if c.contains(e.x, e.y)))
            waiting = tuple(sorted(r for r, h in self.holds.items() if h.kind == 'corridor' and h.on == c.id))
            claimed = self.claims.get(c.id)
            if claimed or occupants or waiting:
                corridors.append(dict(id=c.id, claimed_by=claimed, occupants=occupants, waiting=waiting))
        return stations, corridors
