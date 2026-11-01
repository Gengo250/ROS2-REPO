"""Workers and forklifts of the ATLAS campus, simulated in the backend (world frame).

Ported from the ATLAS prototype (updateWorker, arriveEnd, the vehicle traffic step) with
its constants. ATLAS works in WebGL x/z with z south; here every vector is in the ROS
world frame, where the same yaw values point the same way. LIVE differences: workers never
step into structure, obstacles or vehicles, and drop no automatic parcels (temporary
objects are user-placed only).
"""
from dataclasses import dataclass, field
import math
import numpy as np
from .geometry import box_of, circle_overlaps_polygon, first_overlap, oriented_box, overlap, wrap

TAU = 2*math.pi


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def smooth(e0, e1, x):
    k = clamp((x-e0)/(e1-e0), 0., 1.)
    return k*k*(3-2*k)


@dataclass
class VehicleView:
    """What workers and forklifts know about any vehicle (Nav2 AMR or forklift)."""
    id: str
    x: float
    y: float
    yaw: float
    v: float
    obb: np.ndarray
    samples: list
    is_forklift: bool
    agent: object = None


class Worker:
    RADIUS = 0.25
    STUCK_LIMIT = 10.

    def __init__(self, spec, acts, rng):
        self.id, self.role = spec['id'], spec['role']
        self.level_z = float(spec['level_z'])
        self.path = [(float(p[0]), float(p[1])) for p in spec['path']]
        self.acts_at_end, self.face = spec['acts'], spec['face']
        self.act_specs = acts
        init = spec['initial']
        self.x, self.y, self.yaw = float(init['x']), float(init['y']), float(init['yaw'])
        self.wp, self.dir = init['waypoint'], init['direction']
        self.vmax, self.carry = float(spec['max_speed']), bool(spec['carry'])
        self.side_sign = spec['appearance']['side_sign']
        self.rng = rng
        self.v = self.side = self.wait_t = self.ignore = self.look = self.stuck_t = 0.
        self.waiting = False
        self.turned_back = False
        self.act = None
        self.queue = []
        self.act_face = self.yaw
        self.act_goal = (self.x, self.y)
        self.reason = ''

    @property
    def on_floor(self):
        return self.level_z < 1

    def state(self):
        if self.act:
            return 'working' if self.act['ready'] else 'walking'
        if self.waiting:
            return 'waiting'
        return 'walking' if self.v > .1 else 'idle'

    def activity(self):
        if not self.act:
            return '', 0.
        return self.act['type'], clamp(self.act['t']/self.act['dur'], 0., 1.) if self.act['ready'] else 0.

    def _turn_toward(self, target, dt, rate):
        err = wrap(target-self.yaw)
        self.yaw = wrap(self.yaw+clamp(err*min(1., dt*7), -rate*dt, rate*dt))
        return err

    def _start_act(self):
        kind = self.queue.pop(0)
        self.act = {'type': kind, 't': 0., 'dur': self.act_specs[kind]['dur']*self.rng.uniform(.85, 1.2),
                    'ready': False, 'done': False}

    def _arrive_end(self):
        end = 0 if self.wp == 0 else 1
        queue = [t for t in self.acts_at_end[end] if not (t == 'pick' and self.carry) and not (t == 'drop' and not self.carry)]
        face = self.face[end]
        self.act_face, self.act_goal = self.yaw, (self.x, self.y)
        if face == 'side':
            self.act_face = round((self.yaw+self.side_sign*math.pi/2)/(math.pi/2))*(math.pi/2)
            self.side_sign *= -1
            self.act_goal = (self.x+math.cos(self.act_face)*.72, self.y+math.sin(self.act_face)*.72)
        elif face is not None:
            self.act_face = face
        self.queue = queue or ['idle']
        self._start_act()

    def update(self, dt, vehicles, workers, obstacles, blocked):
        """``blocked(x, y)`` tells whether a disc at (x, y) would touch structure or a vehicle."""
        if dt <= 0:
            return
        prev_yaw = self.yaw
        self.waiting = False
        self.reason = ''
        look_target = 0.
        if self.act:
            act = self.act
            if not act['ready']:
                dx, dy = self.act_goal[0]-self.x, self.act_goal[1]-self.y
                d = math.hypot(dx, dy)
                if d > .04:
                    sp = min(.75, d*2.6)
                    nx, ny = self.x+dx/d*min(sp*dt, d), self.y+dy/d*min(sp*dt, d)
                    if self.on_floor and blocked(nx, ny):
                        # The task spot is occupied (e.g. by an AMR): work from here.
                        self.act_goal = (self.x, self.y)
                        self.v = 0.
                    else:
                        self.x, self.y, self.v = nx, ny, sp
                else:
                    self.v = max(0., self.v-3*dt)
                err = self._turn_toward(self.act_face, dt, 3.4)
                if d < .06 and abs(err) < .1:
                    act['ready'] = True
            else:
                act['t'] += dt
                self.v = max(0., self.v-3*dt)
                self._turn_toward(self.act_face, dt, 1.5)
                if not act['done'] and act['t'] > act['dur']*.5:
                    act['done'] = True
                    if act['type'] == 'pick':
                        self.carry = True
                    elif act['type'] == 'drop':
                        self.carry = False
                if act['t'] >= act['dur']:
                    if self.queue:
                        self._start_act()
                    else:
                        self.act = None
                        if self.wp == len(self.path)-1:
                            self.dir, self.wp = -1, self.wp-1
                        else:
                            self.dir, self.wp = 1, self.wp+1
        else:
            end = self.wp == 0 or self.wp == len(self.path)-1
            tx0, ty0 = self.path[self.wp]
            px0, py0 = self.path[self.wp-self.dir]
            sl = math.hypot(tx0-px0, ty0-py0) or 1.
            # Right-hand perpendicular of the walked segment (ATLAS lateral offset).
            perp_x, perp_y = (ty0-py0)/sl, -(tx0-px0)/sl
            tx, ty = tx0+perp_x*self.side, ty0+perp_y*self.side
            dx, dy = tx-self.x, ty-self.y
            dist = math.hypot(dx, dy)
            err = self._turn_toward(math.atan2(dy, dx), dt, 3.4)
            fx, fy = math.cos(self.yaw), math.sin(self.yaw)
            vt = self.vmax*(.85 if self.carry else 1.)*clamp(math.cos(err)*1.25, .08, 1.)
            vt = min(vt, .2+dist*.8) if end else min(vt, self.vmax*.62+dist*.35)
            side_t, threat = 0., None
            if self.ignore <= 0 and self.on_floor:
                for vh in vehicles:
                    rx, ry = vh.x-self.x, vh.y-self.y
                    d = math.hypot(rx, ry)
                    if d > 8:
                        continue
                    length = 1.9 if vh.is_forklift else 1.1
                    al, lat = rx*fx+ry*fy, abs(-rx*fy+ry*fx)
                    if vh.v > .25:
                        if -.3 < al < 4.2+length and lat < 1.3+length*.4:
                            self.waiting, threat = True, vh
                        vfx, vfy = math.cos(vh.yaw), math.sin(vh.yaw)
                        to_me = (-rx*vfx-ry*vfy)/(d or 1.)
                        if d < 5+length and to_me > .72 and abs(rx*vfy-ry*vfx) < 1.5:
                            self.waiting, threat = True, vh
                    elif 0 < al < 1.7+length and lat < 1.1+length*.3:
                        side_t = 1.5
            for other in workers:
                if other is self or abs(other.level_z-self.level_z) > 1:
                    continue
                rx, ry = other.x-self.x, other.y-self.y
                if abs(rx) > 2.2 or abs(ry) > 2.2:
                    continue
                al, lat = rx*fx+ry*fy, abs(-rx*fy+ry*fx)
                if 0 < al < 1.6 and lat < .75:
                    side_t = max(side_t, .8)
                    vt *= .7
            if self.on_floor:
                for ob in obstacles:
                    rx, ry = ob.x-self.x, ob.y-self.y
                    if abs(rx) > 3 or abs(ry) > 3:
                        continue
                    al, lat = rx*fx+ry*fy, abs(-rx*fy+ry*fx)
                    if -.2 < al < 1.8 and lat < ob.radius+.45:
                        side_t = max(side_t, ob.radius+.8)
            if self.waiting:
                vt = 0.
                self.wait_t += dt
                self.reason = 'vehicle'
                if self.wait_t > 6:
                    self.ignore, self.wait_t = 3., 0.
            else:
                self.wait_t = max(0., self.wait_t-dt*2)
            self.side += (side_t-self.side)*(1-math.exp(-dt*(2.6 if side_t > self.side else .7)))
            self.v += clamp(vt-self.v, -3.6*dt, 1.5*dt)
            nx, ny = self.x+fx*self.v*dt, self.y+fy*self.v*dt
            hit = self.on_floor and self.v > 0 and blocked(nx, ny)
            if hit and side_t > 0:
                # LIVE: stepping aside for a stopped vehicle or an object. Blocked on the way
                # (often by the vehicle itself), step straight sideways up to the side-step or
                # the structure, then wait there so the vehicle can pass.
                self.v = 0.
                self.waiting, self.reason = True, 'blocked'
                offset = (self.x-px0)*perp_x+(self.y-py0)*perp_y
                mx, my = self.x+perp_x*.5*dt, self.y+perp_y*.5*dt
                if offset < side_t-.05 and not blocked(mx, my):
                    self.x, self.y = mx, my
            elif hit:
                # LIVE: never walk through. A task beside a rack can leave the person off
                # the walked line, facing the rack end; slide toward a point 0.5 m ahead on
                # the line, and stop (shrinking the side-step) only if that is blocked too.
                sx, sy = tx0-px0, ty0-py0
                u = clamp(((self.x-px0)*sx+(self.y-py0)*sy)/(sl*sl), 0., 1.)
                ax, ay = px0+sx*u+sx/sl*.5-self.x, py0+sy*u+sy/sl*.5-self.y
                ad = math.hypot(ax, ay) or 1.
                mx, my = self.x+ax/ad*self.v*dt, self.y+ay/ad*self.v*dt
                if blocked(mx, my):
                    self.v, self.side = 0., self.side*.5
                    self.waiting, self.reason = True, 'blocked'
                else:
                    self.x, self.y = mx, my
            else:
                self.x, self.y = nx, ny
            # Bounded wait (as HumanSystem in the Gazebo worlds): a person held up by a
            # vehicle for STUCK_LIMIT turns back along the walked route, so a head-on meeting
            # in a narrow aisle cannot deadlock.
            self.stuck_t = self.stuck_t+dt if self.waiting else max(0., self.stuck_t-dt)
            if self.stuck_t > self.STUCK_LIMIT and len(self.path) > 1:
                self.wp, self.dir = self.wp-self.dir, -self.dir
                self.stuck_t = self.side = self.wait_t = 0.
                self.turned_back, self.reason = True, 'turned back'
            if dist < (.25 if end else .8):
                if end:
                    self._arrive_end()
                else:
                    self.wp += self.dir
            if threat:
                look_target = clamp(wrap(math.atan2(threat.y-self.y, threat.x-self.x)-self.yaw), -1.3, 1.3)
            else:
                look_target = clamp(err*.75, -.9, .9)
        self.ignore -= dt
        self.yaw_rate = wrap(self.yaw-prev_yaw)/dt
        self.look += (look_target-self.look)*(1-math.exp(-dt*4))


@dataclass
class Lane:
    points: list
    lengths: list = field(default_factory=list)

    def __init__(self, points):
        self.points = [(float(x), float(y)) for x, y in points]
        self.lengths = []
        self.__post_init__()
    total: float = 0.

    def __post_init__(self):
        for j, a in enumerate(self.points):
            b = self.points[(j+1) % len(self.points)]
            self.lengths.append(math.hypot(b[0]-a[0], b[1]-a[1]))
        self.total = sum(self.lengths)

    def at(self, distance):
        d = distance % self.total
        for j, length in enumerate(self.lengths):
            if d <= length:
                a, b = self.points[j], self.points[(j+1) % len(self.points)]
                u = min(1., d/max(length, 1e-3))
                return a[0]+(b[0]-a[0])*u, a[1]+(b[1]-a[1])*u
            d -= length
        return self.points[0]


class Forklift:
    """ATLAS vehicle on a one-way lane: protective field, yielding, following, back-off, detours."""

    def __init__(self, spec, lane, rank, rng):
        self.id, self.lane, self.rank, self.rng = spec['id'], Lane(lane), rank, rng
        self.length, self.width, self.cx = spec['length'], spec['width'], spec['centre_offset']
        self.accel, self.brake, self.emerg = spec['acceleration'], spec['braking'], spec['emergency_braking']
        self.a_lat, self.gap, self.k = spec['lateral_acceleration'], spec['gap'], spec['curvature_window']
        self.speed = spec['speed']
        self.d = spec['start_distance']
        self.v = 0.
        self.detour = None
        self.backoff = None
        self.samples = []
        self.prot, self.look, self.field = .5, 3., 0
        self.why, self.ref, self.yield_to = None, None, None
        self.wait_t = self.emerg_t = self.boost_t = self.next_plan = 0.
        self.replans = 0
        self.detour_fail = False
        self.curve = False
        self.pose = self.pose_at(self.d)
        self.obb = self.footprint(self.pose)

    @property
    def is_forklift(self):
        return True

    def offset_at(self, distance):
        t = self.detour
        if not t:
            return 0.
        return t['o']*min(smooth(t['d0'], t['d0']+t['ramp'], distance), 1-smooth(t['d1'], t['d1']+t['ramp'], distance))

    def pose_at(self, distance):
        ax, ay = self.lane.at(distance-self.k)
        bx, by = self.lane.at(distance+self.k)
        yaw = math.atan2(by-ay, bx-ax)
        c, s = math.cos(yaw), math.sin(yaw)
        o = self.offset_at(distance)
        if self.detour:
            d_o = (self.offset_at(distance+.3)-self.offset_at(distance-.3))/.6
            yaw = math.atan2(s-c*d_o, c+s*d_o)
        # Positive offset is to the right of the travel direction.
        return ((ax+bx)/2+s*o, (ay+by)/2-c*o, yaw)

    def footprint(self, pose):
        return oriented_box(pose[0], pose[1], pose[2], self.length, self.width, self.cx)

    def plan_samples(self):
        self.prot = self.v*self.v/(2*self.brake)+self.gap
        self.look = max(self.prot+2.4, 3.4)
        out, s = [], 0.
        while s <= self.look+1e-6:
            p = self.pose_at(self.d+s)
            poly = self.footprint(p)
            out.append((s, p, poly, box_of(poly)))
            s += .35
        self.samples = out

    def view(self):
        return VehicleView(self.id, self.pose[0], self.pose[1], self.pose[2], self.v, self.obb,
                           self.samples, True, self)

    def priority(self):
        return (100 if self.boost_t > 0 else 0)+2+self.rank*.001


def obb_hit(a, b, margin=0.):
    return first_overlap([a], [box_of(a)], [b], [box_of(b)], margin) is not None


def circle_hits_box(box, x, y, radius):
    return circle_overlaps_polygon(x, y, radius, box)


class Traffic:
    """Forklift traffic step; the Nav2 AMR takes part only as an observed vehicle."""

    def __init__(self, forklifts, static_index):
        self.forklifts = forklifts
        self.static = static_index

    def _scan_hazards(self, r, vehicles, workers, obstacles):
        smp = r.samples
        polys = [x[2] for x in smp]
        boxes = np.array([x[3] for x in smp])
        lo, hi = boxes[:, :2].min(0), boxes[:, 2:].max(0)
        best = None

        def consider(hit, kind, refs):
            nonlocal best
            if hit is not None and (best is None or hit[0] < best[0]):
                best = (hit[0], kind, refs[hit[1]])
        ids = self.static.query_ids(lo-.1, hi+.1)
        if len(ids):
            statics = [self.static.polygons[i] for i in ids]
            consider(first_overlap(polys, boxes, statics, np.hstack([self.static.lo[ids], self.static.hi[ids]]), .08),
                     'static', statics)
        if obstacles:
            consider(first_overlap(polys, boxes, [o.polygon for o in obstacles], [box_of(o.polygon) for o in obstacles], .25),
                     'obj', obstacles)
        others = [q for q in vehicles if q.agent is not r]
        if others:
            consider(first_overlap(polys, boxes, [q.obb for q in others], [box_of(q.obb) for q in others], .3), 'veh', others)
        for w in workers:
            if not w.on_floor or w.x < lo[0]-1.5 or w.x > hi[0]+1.5 or w.y < lo[1]-1.5 or w.y > hi[1]+1.5:
                continue
            ax, ay = w.x+math.cos(w.yaw)*w.v*.8, w.y+math.sin(w.yaw)*w.v*.8
            for i, poly in enumerate(polys):
                if best is not None and i >= best[0]:
                    break
                if circle_hits_box(poly, w.x, w.y, .75) or (w.v > .1 and circle_hits_box(poly, ax, ay, .6)):
                    best = (i, 'person', w)
                    break
        return best

    @staticmethod
    def _future_conflict(r, q):
        if len(r.samples) < 2 or len(q.samples) < 2:
            return None
        a, b = r.samples[1:], q.samples[1:]
        hit = first_overlap([x[2] for x in a], [x[3] for x in a], [x[2] for x in b], [x[3] for x in b], .15)
        return None if hit is None else (hit[0]+1, hit[1]+1)

    def _rear_clear(self, r, dist, vehicles, workers, obstacles):
        s = -.35
        while s >= -dist-1e-6:
            f = r.footprint(r.pose_at(r.d+s))
            if self.static.collides(f, .05):
                return False
            if any(obb_hit(f, o.polygon, .15) for o in obstacles):
                return False
            if any(q.agent is not r and obb_hit(f, q.obb, .15) for q in vehicles):
                return False
            if any(w.on_floor and circle_hits_box(f, w.x, w.y, .6) for w in workers):
                return False
            s -= .35
        return True

    def _backoff_step(self, r, dt, vehicles, workers, obstacles):
        b = r.backoff
        r.why, r.ref, r.field, r.yield_to = 'backoff', b['from'], 1, b['from']
        if b['left'] > 0 and self._rear_clear(r, .7, vehicles, workers, obstacles):
            r.v = max(r.v-1.5*dt, -.4)
            b['left'] += r.v*dt
        else:
            r.v = min(0., r.v+2*dt)
            b['left'] = min(b['left'], 0.)
            b['hold'] -= dt
        if b['hold'] <= 0:
            r.backoff = None

    def _plan_detour(self, r, hit_s, ob_poly, vehicles, obstacles):
        base = r.d
        radius = np.abs(ob_poly-ob_poly.mean(0)).max()*1.42
        ramp = clamp(hit_s-.2, 2.6, 4.5)
        d1 = base+hit_s+2*radius+r.length+.6
        keep = r.detour
        for o in (1.6, -1.6, 2.05, -2.05, 2.7, -2.7):
            r.detour = {'d0': base, 'd1': d1, 'o': o, 'ramp': ramp}
            ok, s = True, 0.
            while s <= d1+ramp-base and ok:
                f = r.footprint(r.pose_at(base+s))
                if self.static.collides(f, .15) or any(obb_hit(f, ob.polygon, .32) for ob in obstacles):
                    ok = False
                elif s < 10:
                    for q in vehicles:
                        if q.agent is r:
                            continue
                        if obb_hit(f, q.obb, .25) or any(obb_hit(f, qs[2], .2) for qs in q.samples):
                            ok = False
                            break
                s += .18
            if ok:
                return True
        r.detour = keep
        return False

    def _decide(self, r, vehicles, workers, obstacles, dt):
        if r.backoff:
            self._backoff_step(r, dt, vehicles, workers, obstacles)
            return
        lim, why, ref = math.inf, None, None
        if r.emerg_t > 0:
            r.emerg_t -= dt
            lim, why = 0., 'emerg'
        hazard = self._scan_hazards(r, vehicles, workers, obstacles)
        if hazard:
            s = r.samples[hazard[0]][0]
            if s < lim:
                lim, why, ref = s, hazard[1], hazard[2]
        # Crossings and merges: whoever arrives later yields; ties go to priority.
        y_to, y_lim = None, math.inf
        for q in vehicles:
            if q.agent is r or not q.samples or (q.agent is not None and q.agent.yield_to is r):
                continue
            reach = r.look+(q.agent.look if q.agent else 3.)+6
            if (q.x-r.pose[0])**2+(q.y-r.pose[1])**2 > reach*reach:
                continue
            fc = self._future_conflict(r, q)
            if not fc:
                continue
            s_a, s_b = r.samples[fc[0]][0], q.samples[fc[1]][0]
            q_wait = q.agent.wait_t if q.agent else 0.
            t_a = s_a/max(r.v, .5)
            t_b = math.inf if (q.v < .05 and q_wait > .6) else s_b/max(q.v, .5)
            q_prio = q.agent.priority() if q.agent else 1.
            give = t_a > t_b+.35 or (abs(t_a-t_b) <= .35 and r.priority() < q_prio)
            if r.yield_to is q.agent and t_b < math.inf and r.priority() < 100:
                give = True
            if give:
                limit = max(0., s_a-.45)
                if limit < y_lim:
                    y_lim, y_to = limit, q
        if y_to and y_lim < lim:
            lim, why, ref = y_lim, 'yield', y_to
        if why in ('yield', 'veh') and ref is not None and math.cos(wrap(ref.yaw-r.pose[2])) > .8 and ref.v > .05:
            why = 'follow'
        r.yield_to = y_to.agent if y_to else None
        # Target speed: cruise, curve limit, braking profile to the stopping point.
        vt = r.speed
        r.curve = False
        sm = r.samples
        for i in range(1, len(sm)):
            k = abs(wrap(sm[i][1][2]-sm[i-1][1][2]))/.35
            if k > .08:
                vc = math.sqrt(r.a_lat/k)
                allowed = math.sqrt(vc*vc+2*r.brake*sm[i-1][0])
                if allowed < vt:
                    vt, r.curve = allowed, True
        if lim < math.inf:
            available = lim-(0 if why == 'yield' else r.gap)
            vs = math.sqrt(2*r.brake*available) if available > 0 else 0.
            vt = min(vt, vs)
        dv = vt-r.v
        r.v += min(dv, r.accel*dt) if dv > 0 else max(dv, -r.emerg*(2 if why == 'emerg' else 1)*dt)
        r.v = max(r.v, 0.)
        stopped = r.v < .06
        r.wait_t = r.wait_t+dt if stopped and lim < math.inf else 0.
        if r.wait_t > 12 and r.boost_t <= 0:
            r.boost_t = 5.
        r.boost_t -= dt
        if (why == 'veh' and ref is not None and r.wait_t > 1.5 and ref.agent is not None and
                ((ref.agent.why == 'veh' and ref.agent.ref is r.view_ref and r.priority() < ref.agent.priority())
                 or (r.wait_t > 9 and ref.agent.why in ('veh', 'yield') and ref.agent.wait_t > 3))
                and self._rear_clear(r, 1.2, vehicles, workers, obstacles)):
            r.backoff, r.wait_t = {'left': 3., 'hold': 2.6, 'from': ref}, 0.
        # User object on the lane: plan a lateral detour while there is room.
        r.next_plan -= dt
        if r.v > .4 or (r.wait_t > 20 and r.replans >= 3):
            r.replans = 0
        if why in ('obj', 'static') and r.detour and r.wait_t > 1.5:
            r.detour, r.next_plan = None, 0.
        if why == 'obj' and not r.detour and r.next_plan <= 0 and (r.wait_t > .8 or lim > 4.8):
            r.next_plan = 1.2
            r.detour_fail = not self._plan_detour(r, lim, ref.polygon, vehicles, obstacles)
            if r.detour_fail and lim < 2.2 and r.replans < 3 and self._rear_clear(r, 2.2, vehicles, workers, obstacles):
                r.replans += 1
                r.backoff, r.next_plan = {'left': 2.6, 'hold': .4, 'from': None}, 0.
        if not r.detour and why != 'obj':
            r.detour_fail = False
        r.why, r.ref = why, ref
        r.field = 2 if (why == 'emerg' or (lim < math.inf and lim <= r.prot+.05 and why not in ('follow', 'yield'))) \
            else 1 if (lim < math.inf or (r.curve and r.v < r.speed*.6)) else 0

    def step(self, dt, vehicles, workers, obstacles):
        """``vehicles`` holds VehicleViews for every forklift and every AMR."""
        if dt <= 0:
            return
        n = math.ceil(dt/.034)
        h = dt/n
        for _ in range(n):
            for r in self.forklifts:
                r.plan_samples()
            views = {id(v.agent): v for v in vehicles if v.agent is not None}
            for r in self.forklifts:
                r.view_ref = views.get(id(r))
            for r in self.forklifts:
                self._decide(r, vehicles, workers, obstacles, h)
            for r in self.forklifts:
                r.d += r.v*h
                r.pose = r.pose_at(r.d)
                r.obb = r.footprint(r.pose)
                if r.detour and r.d > r.detour['d1']+r.detour['ramp']:
                    r.detour = None
                view = views.get(id(r))
                if view is not None:
                    view.x, view.y, view.yaw, view.v, view.obb, view.samples = \
                        r.pose[0], r.pose[1], r.pose[2], r.v, r.obb, r.samples

    @staticmethod
    def state(r):
        if r.backoff or r.v < -.05:
            return 'reversing', r.why or ''
        if r.v < .06:
            return 'stopped', r.why or ''
        if r.why or r.curve:
            return 'slowing', r.why or 'curve'
        return 'moving', ''
