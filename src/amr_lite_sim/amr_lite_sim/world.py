"""The lite world: robots, workers, forklifts, user obstacles and sensors. No ROS here.

One ``step(dt)`` advances everything by a fixed simulation step. The ROS node owns timing,
publication and services; this module is deterministic for a given seed and command log.
"""
from dataclasses import dataclass, field
import math
import numpy as np
from .agents import Forklift, Traffic, VehicleView, Worker
from .drive import DiffDrive, DriveState
from .geometry import StaticIndex, box_of, disc, oriented_box, overlap, polygon_segments, circle_overlaps_polygon
from .lidar import SegmentSet, scan

HUMAN_BAND = (0.05, 1.8)
# Safety field of the AMR (simulated safety scanner). Reported for every object; enforced for
# people only: safe stop in the protective field, reduced speed in the warning field.
AMR_FIELD_BRAKING = 0.5      # m/s², the drive's deceleration limit
AMR_FIELD_GAP = 0.3
AMR_WARNING_EXTRA = 1.0
AMR_SAFETY_BRAKING = 1.5     # m/s², safe stop, above the 0.5 m/s² motion profile
AMR_WARNING_SPEED = 0.3      # m/s with a person in the warning field
ROTATION_SAFETY_MARGIN = 0.15  # m beyond the circumscribed radius


@dataclass
class Obstacle:
    id: str
    kind: str
    x: float
    y: float
    yaw: float
    length: float
    width: float
    height: float
    origin: str = 'user'
    polygon: np.ndarray = None
    radius: float = 0.

    def __post_init__(self):
        if self.kind == 'cone':
            self.polygon = disc(self.x, self.y, .15, 10)
        else:
            self.polygon = oriented_box(self.x, self.y, self.yaw, self.length, self.width)
        self.radius = math.hypot(self.length, self.width)/2


@dataclass
class Robot:
    id: str
    drive: DriveState
    spawn: tuple
    state: str = 'stopped'
    reason: str = ''
    field_state: int = 0
    field_length: float = 0.
    contact_with: str = ''
    safety_stop: str = ''
    odom: list = field(default_factory=lambda: [0., 0., 0.])

    def view(self, model):
        d = self.drive
        return VehicleView(self.id, d.x, d.y, d.yaw, d.v, footprint(model, d.x, d.y, d.yaw),
                           predicted_samples(model, d), False, None)


def footprint(model, x, y, yaw, inflate=0.):
    return oriented_box(x, y, yaw, model.length+2*inflate, model.width+2*inflate, model.footprint_offset)


def predicted_samples(model, d, horizon=3., step=.35):
    """Constant-twist look-ahead of an AMR, so forklifts can negotiate crossings with it."""
    out, s, x, y, yaw = [], 0., d.x, d.y, d.yaw
    speed = abs(d.v)
    if speed < .05:
        poly = footprint(model, x, y, yaw)
        return [(0., (x, y, yaw), poly, box_of(poly))]
    dt = step/speed
    t = 0.
    while t <= horizon:
        poly = footprint(model, x, y, yaw)
        out.append((s, (x, y, yaw), poly, box_of(poly)))
        x, y, yaw = DiffDrive.advance(x, y, yaw, d.v, d.w, dt)
        s += step
        t += dt
    return out


class World:
    def __init__(self, scene, model, profile='operations', seed=7):
        self.scene, self.model, self.profile = scene, model, profile
        self.rng = np.random.default_rng(seed)
        self.time = 0.
        self.elapsed_ns = 0  # exact simulation time; ``time`` is derived, never accumulated
        self.drive = DiffDrive(model)
        interior = scene.section('building')['interior']
        self.interior = (interior['x_min'], interior['x_max'], interior['y_min'], interior['y_max'])
        self.body_static = StaticIndex(scene.band_polygons(model.ground_clearance, model.height))
        self.human_static = StaticIndex(scene.band_polygons(*HUMAN_BAND))
        self.scan_static = {spec.frame_id: SegmentSet(*polygon_segments(scene.slice_polygons(spec.mount_z)))
                            for spec in (model.front, model.top)}
        self.robots = {}
        for spawn in scene.section('spawns'):
            if spawn['active']:
                x, y, _, _, _, yaw = map(float, spawn['pose'])
                self.robots[spawn['robot_id']] = Robot(spawn['robot_id'], DriveState(x, y, yaw), (x, y, yaw))
        agents = scene.section('agents')
        self.workers = []
        self.forklifts = []
        if profile == 'operations':
            self.workers = [Worker(w, agents['acts'], self.rng) for w in agents['workers']]
            lanes = scene.section('lanes')
            fleet_rank = {'EMP-1': 8, 'EMP-2': 9, 'EMP-3': 10}  # ranks in the original fleet table
            self.forklifts = [Forklift(f, lanes[f['loop']]['points'], fleet_rank.get(f['id'], 8), self.rng)
                              for f in agents['forklifts']]
        elif profile not in ('mapping', 'empty'):
            raise ValueError('profile must be operations, mapping or empty')
        self.traffic = Traffic(self.forklifts, StaticIndex(scene.band_polygons(0.05, 2.0)))
        self.obstacles = {}
        self.obstacle_seq = 0
        self.catalogue = scene.section('obstacle_catalogue')
        self.events = []
        self.event_seq = 0
        self.agent_accumulator = 0.
        self.agent_period = 0.02

    # ------------------------------------------------------------------ events
    def emit(self, kind, entity='', other='', detail='', x=0., y=0.):
        self.event_seq += 1
        self.events.append({'seq': self.event_seq, 'time': self.time, 'kind': kind, 'entity': entity,
                            'other': other, 'detail': detail, 'x': x, 'y': y})

    def drain_events(self):
        events, self.events = self.events, []
        return events

    # ------------------------------------------------------------------ commands
    def command(self, robot_id, v, w):
        d = self.robots[robot_id].drive
        d.command, d.command_time = (v, w), self.time

    def add_obstacle(self, kind, x, y, yaw=0.):
        spec = self.catalogue.get(kind)
        if spec is None:
            return None, f'unknown obstacle kind {kind!r}'
        x0, x1, y0, y1 = self.interior
        if not (x0+.3 < x < x1-.3 and y0+.3 < y < y1-.3):
            return None, 'outside the building'
        candidate = Obstacle('', kind, x, y, yaw, spec['length'], spec['width'], spec['height'])
        if self.body_static.collides(candidate.polygon, .04):
            return None, 'occupied by a permanent structure'
        if any(overlap(candidate.polygon, o.polygon, .04) for o in self.obstacles.values()):
            return None, 'another object is already there'
        for r in self.robots.values():
            if overlap(candidate.polygon, footprint(self.model, r.drive.x, r.drive.y, r.drive.yaw), .25):
                return None, f'robot {r.id} is there'
        if any(overlap(candidate.polygon, f.obb, .25) for f in self.forklifts):
            return None, 'a vehicle is there'
        if any(w.on_floor and circle_overlaps_polygon(w.x, w.y, .45, candidate.polygon) for w in self.workers):
            return None, 'a person is there'
        self.obstacle_seq += 1
        candidate.id = f'obstacle_{self.obstacle_seq:04d}'
        self.obstacles[candidate.id] = candidate
        self.emit('obstacle_added', candidate.id, '', kind, x, y)
        return candidate, ''

    def remove_obstacle(self, identifier=''):
        ids = [identifier] if identifier else list(self.obstacles)
        removed = 0
        for i in ids:
            o = self.obstacles.pop(i, None)
            if o:
                removed += 1
                self.emit('obstacle_removed', o.id, '', o.kind, o.x, o.y)
        return removed

    # ------------------------------------------------------------------ simulation
    def vehicle_views(self):
        views = [r.view(self.model) for r in self.robots.values()]
        views += [f.view() for f in self.forklifts]
        return views

    def _robot_contact(self, robot, poly):
        if self.body_static.collides(poly):
            return 'structure'
        for o in self.obstacles.values():
            if overlap(poly, o.polygon):
                return o.id
        for w in self.workers:
            if w.on_floor and circle_overlaps_polygon(w.x, w.y, Worker.RADIUS, poly):
                return w.id
        for f in self.forklifts:
            if overlap(poly, f.obb):
                return f.id
        for other in self.robots.values():
            if other is not robot and overlap(poly, footprint(self.model, other.drive.x, other.drive.y, other.drive.yaw)):
                return other.id
        return ''

    def _rotation_unsafe(self, r, w):
        """Simulated safety-scanner rotation stop: no in-place turn with a person inside the swept disc."""
        d = r.drive
        reach = math.hypot(self.model.length/2+abs(self.model.footprint_offset), self.model.width/2)+ROTATION_SAFETY_MARGIN
        return next((k.id for k in self.workers if k.on_floor and math.hypot(k.x-d.x, k.y-d.y) < reach+Worker.RADIUS), '')

    def _step_robot(self, r, dt):
        d = r.drive
        v, w = self.drive.velocities(d, self.time, dt)
        v, w = self._personnel_safety(r, v, w, dt)
        person = self._rotation_unsafe(r, w) if abs(v) < .05 and abs(w) > 1e-3 else ''
        if person:
            if r.reason != f'rotation_stop:{person}':
                self.emit('protective_stop', r.id, person, 'in-place rotation refused: person inside the swept area', d.x, d.y)
            d.v = d.w = 0.
            r.state, r.reason = 'stopped', f'rotation_stop:{person}'
            self._evaluate_field(r)
            return
        x, y, yaw = DiffDrive.advance(d.x, d.y, d.yaw, v, w, dt)
        contact = ''
        if abs(v) > 1e-6 or abs(w) > 1e-6:
            contact = self._robot_contact(r, footprint(self.model, x, y, yaw))
        previous_v = d.v
        if contact:
            # Simplified collision: advance to the last collision-free fraction of the step
            # (the robot ends touching), then stop. Only motion away from the body releases it.
            low, high = 0., 1.
            for _ in range(10):
                mid = (low+high)/2
                px, py, pyaw = DiffDrive.advance(d.x, d.y, d.yaw, v, w, dt*mid)
                if self._robot_contact(r, footprint(self.model, px, py, pyaw)):
                    high = mid
                else:
                    low = mid
            if low > 0:
                d.x, d.y, d.yaw = DiffDrive.advance(d.x, d.y, d.yaw, v, w, dt*low)
            if r.contact_with != contact:
                self.emit('contact', r.id, contact, 'motion blocked by contact', d.x, d.y)
            r.contact_with = contact
            d.v = d.w = 0.
            r.state, r.reason = 'blocked', f'contact:{contact}'
        else:
            if r.contact_with:
                self.emit('released', r.id, r.contact_with, 'contact cleared', d.x, d.y)
            r.contact_with = ''
            left, right = self.drive.wheel_rates(v, w)
            d.left_angle += left*dt
            d.right_angle += right*dt
            d.x, d.y, d.yaw, d.v, d.w = x, y, yaw, v, w
            moving = abs(v) > .01 or abs(w) > .02
            r.state, r.reason = ('moving', '') if moving else ('stopped', '')
            if r.safety_stop:
                r.reason = f'protective_stop:{r.safety_stop}'
        d.accel = (d.v-previous_v)/dt
        self._evaluate_field(r)

    def _field_zone(self, r, length):
        d = r.drive
        front = self.model.footprint_offset+self.model.length/2
        c, s = math.cos(d.yaw), math.sin(d.yaw)
        return oriented_box(d.x+c*(front+length/2), d.y+s*(front+length/2), d.yaw, length, self.model.width+.2)

    def _person_in(self, r, length):
        poly = self._field_zone(r, length)
        box = box_of(poly)
        reach = self.model.length+length+Worker.RADIUS
        return next((k.id for k in self.workers if k.on_floor and abs(k.x-r.drive.x) < reach and abs(k.y-r.drive.y) < reach
                     and circle_overlaps_polygon(k.x, k.y, Worker.RADIUS, poly, box)), '')

    def _personnel_safety(self, r, v, w, dt):
        """Simulated safety scanner for people, independent of navigation: a person in the
        protective field (sized for the current speed) triggers a safe stop; it resets once the
        standstill warning field is clear, so the restart cannot oscillate. A person in the warning
        field limits the speed. Only forward motion is limited; turning has its own stop."""
        d = r.drive
        if r.safety_stop and not self._person_in(r, AMR_FIELD_GAP+AMR_WARNING_EXTRA):
            r.safety_stop = ''
        if v <= 0.:
            return v, w
        speed = max(d.v, v)
        protective = speed*speed/(2*AMR_FIELD_BRAKING)+AMR_FIELD_GAP
        if not r.safety_stop:
            person = self._person_in(r, protective)
            if person:
                r.safety_stop = person
                self.emit('protective_stop', r.id, person, 'person inside the protective field: safe stop', d.x, d.y)
        if r.safety_stop:
            limit = max(0., d.v-AMR_SAFETY_BRAKING*dt)
        elif self._person_in(r, protective+AMR_WARNING_EXTRA):
            limit = max(AMR_WARNING_SPEED, d.v-AMR_FIELD_BRAKING*dt)
        else:
            return v, w
        if v > limit:
            v, w = limit, w*limit/v                 # keep the commanded curvature
        return v, w

    def _evaluate_field(self, r):
        d = r.drive
        speed = max(d.v, 0.)
        protective = speed*speed/(2*AMR_FIELD_BRAKING)+AMR_FIELD_GAP
        warning = protective+AMR_WARNING_EXTRA

        def occupied(poly):
            if self.body_static.collides(poly):
                return True
            if any(overlap(poly, o.polygon) for o in self.obstacles.values()):
                return True
            box = box_of(poly)
            if any(w.on_floor and circle_overlaps_polygon(w.x, w.y, Worker.RADIUS, poly, box+[-.3, -.3, .3, .3])
                   for w in self.workers):
                return True
            return any(overlap(poly, f.obb) for f in self.forklifts)
        state = 2 if occupied(self._field_zone(r, protective)) else 1 if occupied(self._field_zone(r, warning)) else 0
        if state == 2 and r.field_state != 2 and speed > .05:
            self.emit('protective_field', r.id, '', 'object inside the protective field', d.x, d.y)
        r.field_state, r.field_length = state, protective

    def step(self, dt):
        for r in self.robots.values():
            self._step_robot(r, dt)
        self.agent_accumulator += dt
        if self.agent_accumulator >= self.agent_period-1e-9:
            h, self.agent_accumulator = self.agent_accumulator, 0.
            views = self.vehicle_views()
            obstacles = list(self.obstacles.values())
            dynamic = [(o.polygon, box_of(o.polygon)) for o in obstacles]
            dynamic += [(v.obb, box_of(v.obb)) for v in views]
            blocked = self._worker_blocked(dynamic)
            for w in self.workers:
                w.update(h, views, self.workers, obstacles, lambda x, y, w=w: blocked(x, y, w.x, w.y))
                if w.turned_back:
                    w.turned_back = False
                    self.emit('turned_back', w.id, '', 'waited for a vehicle and turned back', w.x, w.y)
            self.traffic.step(h, views, self.workers, obstacles)
        self.elapsed_ns += int(round(dt*1e9))
        self.time = self.elapsed_ns/1e9

    def _worker_blocked(self, dynamic):
        """Disc test against structure and the dynamic bodies prepared for this agent step."""
        radius = Worker.RADIUS+.05

        def blocked(x, y, px, py):
            if self.human_static.circle_collides(x, y, Worker.RADIUS):
                return True
            for poly, box in dynamic:
                if circle_overlaps_polygon(x, y, radius, poly, box):
                    # Already inside the margin (a vehicle stopped against the person): only steps
                    # away from the body are allowed, so nobody stays pinned.
                    if not circle_overlaps_polygon(px, py, radius, poly, box):
                        return True
                    cx, cy = poly.mean(0)
                    if math.hypot(x-cx, y-cy) <= math.hypot(px-cx, py-cy):
                        return True
            return False
        return blocked

    # ------------------------------------------------------------------ sensing
    def sensor_pose(self, robot, spec):
        d = robot.drive
        c, s = math.cos(d.yaw), math.sin(d.yaw)
        return d.x+c*spec.mount_x-s*spec.mount_y, d.y+s*spec.mount_x+c*spec.mount_y, d.yaw+spec.mount_yaw

    def dynamic_segments(self, robot, spec):
        polys = []
        for w in self.workers:
            if w.on_floor:
                polys.append(disc(w.x, w.y, Worker.RADIUS, 10))
        polys += [f.obb for f in self.forklifts]
        for o in self.obstacles.values():
            if o.height >= spec.mount_z:
                polys.append(o.polygon if o.kind != 'cone' or spec.mount_z < .3 else disc(o.x, o.y, .1, 10))
        for other in self.robots.values():
            if other is not robot and spec.mount_z < self.model.height-.05:
                polys.append(footprint(self.model, other.drive.x, other.drive.y, other.drive.yaw))
        return polygon_segments(polys)

    def scan(self, robot_id, spec):
        robot = self.robots[robot_id]
        x, y, yaw = self.sensor_pose(robot, spec)
        a, b = self.dynamic_segments(robot, spec)
        return scan(spec, x, y, yaw, self.scan_static[spec.frame_id], a, b, self.rng)

    def odometry(self, robot_id):
        """Pose of base_link in odom (odom = spawn pose, as diff_drive_controller starts)."""
        r = self.robots[robot_id]
        sx, sy, syaw = r.spawn
        dx, dy = r.drive.x-sx, r.drive.y-sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        return c*dx-s*dy, s*dx+c*dy, math.atan2(math.sin(r.drive.yaw-syaw), math.cos(r.drive.yaw-syaw))
