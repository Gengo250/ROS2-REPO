"""Scene configurations for synthetic captures, driven by the lite engine itself (amr_lite_sim.World).

A *session* is one engine world: a seed (workers' and forklifts' behaviour), a warm-up, user
objects placed through the engine's own validated ``add_obstacle``, a lighting mode, and a
sequence of frames. Between frames the world is stepped (people walk and work, forklifts drive);
for each frame one robot is the camera, placed either
 * ``targeted``: at a sampled distance and bearing from an object (person, forklift, another AMR,
   a user object or a floor pallet/table box) and turned roughly towards it, or
 * ``driving``: driven by velocity commands through the engine (collisions, safety scanner),
and its camera is rendered from the engine's exact entity states. Every robot pose is checked
collision-free with the engine's own collision test; nothing is drawn that the engine does not hold.
"""
from dataclasses import dataclass
import hashlib
import math
import numpy as np
from amr_lite_sim.world import World, footprint

FIELDS = ('free', 'warning', 'protective')


def session_seed(seed, index):
    """Deterministic per-session seed from the generation seed."""
    return int.from_bytes(hashlib.sha256(f'{seed}:{index}'.encode()).digest()[:4], 'little')


def entity_json(world):
    """World entities as the world-state v1 fields the renderer uses (same as the Gateway's)."""
    out = []
    for e in world.entity_states():
        e = dict(e)
        e.setdefault('activity', None)
        e.setdefault('activity_progress', 0.)
        e.setdefault('carrying', False)
        e.setdefault('look', 0.)
        e['field_state'] = FIELDS[e.get('field_state', 0)] if 0 <= e.get('field_state', 0) < 3 else 'protective'
        e['field_length'] = float(e.get('field_length', 0.))
        e['reason'] = e.get('reason') or None
        out.append({k: (float(v) if isinstance(v, (np.floating,)) else v) for k, v in e.items()})
    return out


def obstacle_json(world):
    return [dict(id=o.id, kind=o.kind, origin=o.origin, x=o.x, y=o.y, yaw=o.yaw, length=o.length, width=o.width, height=o.height)
            for o in world.obstacles.values()]


@dataclass
class View:
    robot_id: str
    mode: str            # targeted | driving
    target: str | None   # targeted object id
    target_kind: str | None


class Session:
    def __init__(self, scene, model, config, seed, robots, populate=True):
        """``populate``: warm up, scatter the robots and place random objects (dataset sessions);
        False leaves the engine's initial world for scripted scenarios."""
        self.config, self.seed, self.robots = config, seed, list(robots)
        self.rng = np.random.default_rng(seed)
        self.world = World(scene, model, 'operations', seed=seed)
        self.scene = scene
        self.step = config['engine_step']
        self.night = bool(self.rng.random() < config['night_fraction'])
        self.anchors = self._anchors()
        solids = scene.raw['solids']
        self.static_targets = [(s['id'], s['tag'], s['x'], s['y']) for s in solids if s['tag'] in ('Palete no chão', 'Mesa de inspeção')]
        if not populate:
            return
        self.advance(self.rng.uniform(*config['warmup_seconds']))
        for robot in self.world.robots:
            self.place_free(robot)
        self.place_obstacles(int(self.rng.integers(config['obstacles_per_session'][0], config['obstacles_per_session'][1]+1)))

    # ------------------------------------------------------------------ world helpers
    def _anchors(self):
        """Floor regions robots may stand in: the scene's navigation corridors (centre, size)."""
        return [(c['centre'], c['size']) for c in self.scene.raw['navigation']['corridors']]

    def advance(self, seconds):
        for _ in range(int(round(seconds/self.step))):
            self.world.step(self.step)

    def free(self, robot_id, x, y, yaw):
        """Engine collision test for a robot pose (structure, objects, people, vehicles, robots)."""
        w = self.world
        x0, x1, y0, y1 = w.interior
        if not (x0+1 < x < x1-1 and y0+1 < y < y1-1):
            return False
        poly = footprint(w.model, x, y, yaw, inflate=.08)
        return not w._robot_contact(w.robots[robot_id], poly)

    def teleport(self, robot_id, x, y, yaw):
        d = self.world.robots[robot_id].drive
        d.x, d.y, d.yaw, d.v, d.w = x, y, yaw, 0., 0.
        d.command = (0., 0.)

    def place_free(self, robot_id, tries=200):
        for _ in range(tries):
            (cx, cy), (sx, sy) = self.anchors[self.rng.integers(len(self.anchors))]
            x, y = cx+self.rng.uniform(-sx/2, sx/2), cy+self.rng.uniform(-sy/2, sy/2)
            yaw = self.rng.uniform(-math.pi, math.pi)
            if self.free(robot_id, x, y, yaw):
                self.teleport(robot_id, x, y, yaw)
                return True
        return False

    def place_obstacles(self, count, tries=400):
        kinds, weights = zip(*self.config['obstacle_kinds'].items())
        weights = np.array(weights)/sum(weights)
        placed = 0
        for _ in range(tries):
            if placed >= count:
                break
            # Near where people and robots are: worker paths, corridors and stations.
            choice = self.rng.random()
            if choice < .4:
                worker = self.scene.raw['agents']['workers'][self.rng.integers(len(self.scene.raw['agents']['workers']))]
                px, py = worker['path'][self.rng.integers(len(worker['path']))]
            elif choice < .8:
                (px, py), (sx, sy) = self.anchors[self.rng.integers(len(self.anchors))]
                px, py = px+self.rng.uniform(-sx/2, sx/2), py+self.rng.uniform(-sy/2, sy/2)
            else:
                station = self.scene.raw['stations'][self.rng.integers(len(self.scene.raw['stations']))]
                px, py = station['pose'][0], station['pose'][1]
            x, y = px+self.rng.uniform(-3, 3), py+self.rng.uniform(-3, 3)
            kind = kinds[self.rng.choice(len(kinds), p=weights)]
            obstacle, _ = self.world.add_obstacle(kind, float(x), float(y), float(self.rng.uniform(-math.pi, math.pi)))
            placed += obstacle is not None
        return placed

    # ------------------------------------------------------------------ views
    def target_candidates(self, camera):
        w = self.world
        # (id, x, y, radius): the camera distance is measured from the object's outline.
        out = {'person': [(k.id, k.x, k.y, .3) for k in w.workers if k.on_floor],
               'forklift': [(f.id, f.pose[0], f.pose[1], 2.4) for f in w.forklifts],
               'amr': [(r.id, r.drive.x, r.drive.y, .6) for r in w.robots.values() if r.id != camera],
               'obstacle': [(o.id, o.x, o.y, o.radius) for o in w.obstacles.values()],
               'static': [(sid, x, y, 1.6) for sid, _, x, y in self.static_targets]}
        return {k: v for k, v in out.items() if v}

    def targeted(self, camera):
        candidates = self.target_candidates(camera)
        kinds = [k for k in self.config['targets'] if k in candidates]
        p = np.array([self.config['targets'][k] for k in kinds], float)
        kind = kinds[self.rng.choice(len(kinds), p=p/p.sum())]
        if kind == 'amr':
            # Another AMR in view: move it somewhere free first (robots are seen in many places).
            other = candidates['amr'][self.rng.integers(len(candidates['amr']))][0]
            self.place_free(other)
            candidates = self.target_candidates(camera)
        tid, tx, ty, radius = candidates[kind][self.rng.integers(len(candidates[kind]))]
        lo, hi = self.config['target_distance_m']
        for _ in range(120):
            d = radius+math.exp(self.rng.uniform(math.log(lo), math.log(hi)))
            bearing = self.rng.uniform(-math.pi, math.pi)
            x, y = tx+d*math.cos(bearing), ty+d*math.sin(bearing)
            yaw = math.atan2(ty-y, tx-x)+float(np.clip(self.rng.normal(0, self.config['heading_jitter_rad']/2),
                                                        -self.config['heading_jitter_rad'], self.config['heading_jitter_rad']))
            if self.free(camera, x, y, yaw):
                self.teleport(camera, x, y, yaw)
                return View(camera, 'targeted', tid, kind)
        self.place_free(camera)
        return View(camera, 'targeted', None, None)

    def driving(self, camera, seconds):
        """Drive the camera robot through the engine for ``seconds`` (moving-camera frames)."""
        v, w = self.rng.uniform(.3, .6), self.rng.uniform(-.25, .25)
        robot = self.world.robots[camera]
        for _ in range(int(round(seconds/self.step))):
            if robot.state == 'blocked':
                v, w = 0., self.rng.choice([-.8, .8])
            self.world.command(camera, v, w)
            self.world.step(self.step)
        return View(camera, 'driving', None, None)

    def next_view(self):
        camera = self.robots[self.rng.integers(len(self.robots))]
        gap = self.rng.uniform(*self.config['seconds_between_frames'])
        if self.rng.random() < self.config['view_modes']['targeted']:
            self.advance(gap)
            return self.targeted(camera)
        return self.driving(camera, gap+self.rng.uniform(.5, 2.5))

    def state(self):
        return {'entities': entity_json(self.world), 'obstacles': obstacle_json(self.world), 'sim_time': self.world.time}
