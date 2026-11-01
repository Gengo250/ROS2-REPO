"""Thread-safe latest-value store for world-state v1. Ages use monotonic wall time.

ROS callbacks hand in already-normalized plain values; the asyncio publisher builds one
immutable snapshot per tick. When the engine goes quiet, the last values are kept and
marked stale/offline; nothing is extrapolated or invented.
"""
from collections import deque
from datetime import datetime, timezone
import math
from threading import Lock
from time import monotonic, time
from uuid import uuid4
from .models import Connection, RobotNavigation, Simulation, WorldEntity, WorldEvent, WorldObstacle, WorldState

FIELDS = ('free', 'warning', 'protective')


def utcnow():
    return datetime.now(timezone.utc)


class WorldStore:
    def __init__(self, settings, registration=None, timer=monotonic, wall_clock=utcnow, wall_time=time):
        self.settings, self.registration = settings, registration
        self.timer, self.wall_clock, self.wall_time = timer, wall_clock, wall_time
        self.lock = Lock()
        self.session_id = str(uuid4())
        self.sequence = 0
        self.started = timer()
        self.ros_initialized = False
        self.engine = None          # (receipt, dict)
        self.events = deque(maxlen=50)
        self.last_event_seq = 0
        self.engine_session = None
        self.plans = {}             # robot id → (receipt, world points, omitted reason)
        self.interlock = None       # (receipt, message)
        self.missions = {}

    def set_ros_initialized(self, value):
        with self.lock:
            self.ros_initialized = value

    def availability(self, receipt, now):
        if not self.ros_initialized:
            return 'offline'
        if receipt is None:
            return 'unknown' if now-self.started < self.settings.offline_seconds else 'offline'
        age = now-receipt
        if age >= self.settings.offline_seconds:
            return 'offline'
        return 'stale' if age >= self.settings.stale_seconds else 'online'

    # ------------------------------------------------------------------ inputs (ROS thread)
    def update_engine(self, state):
        """``state``: dict with session_id, sim_time, rtf, target_rtf, paused, overruns, scene_sha256,
        entities (list of dicts), obstacles (list of dicts)."""
        entities = tuple(WorldEntity(**e) for e in state['entities'])
        obstacles = tuple(WorldObstacle(**o) for o in state['obstacles'])
        with self.lock:
            if self.engine_session and state['session_id'] != self.engine_session:
                self.events.clear()
                self.last_event_seq = 0
            self.engine_session = state['session_id']
            self.engine = (self.timer(), {**state, 'entities': entities, 'obstacles': obstacles})

    def add_event(self, event):
        with self.lock:
            if event['seq'] <= self.last_event_seq:
                return
            self.last_event_seq = event['seq']
            self.events.append(WorldEvent(**event))

    def update_plan(self, robot_id, points, frame_id):
        """Nav2 plan in the map frame → world frame through the verified registration."""
        if self.registration is None:
            world, reason = (), 'world/map registration unavailable'
        elif frame_id != 'map':
            world, reason = (), f'unsupported plan frame {frame_id!r}'
        else:
            step = max(1, math.ceil(len(points)/256))
            sampled = points[::step]
            if points and sampled[-1] != points[-1]:
                sampled = sampled[:255]+[points[-1]]
            world, reason = tuple(self.registration.map_to_world(x, y) for x, y in sampled), None
        with self.lock:
            self.plans[robot_id] = (self.timer(), world, reason)

    def update_interlock(self, message):
        with self.lock:
            self.interlock = (self.timer(), message[:256])

    def update_mission(self, mission):
        with self.lock:
            self.missions[mission.robot_id] = mission

    def mission(self, robot_id):
        with self.lock:
            return self.missions.get(robot_id)

    # ------------------------------------------------------------------ output (asyncio thread)
    def snapshot(self):
        with self.lock:
            now = self.timer()
            receipt, engine = self.engine if self.engine else (None, None)
            simulation_availability = self.availability(receipt, now)
            if engine is None:
                simulation = Simulation(backend='none', status='unknown' if simulation_availability == 'unknown' else 'offline',
                                        availability=simulation_availability, engine_session_id=None, sim_time=None,
                                        real_time_factor=None, target_real_time_factor=None, overruns=0,
                                        scene_sha256=None, age_seconds=None)
            else:
                status = {'online': 'paused' if engine['paused'] else 'running', 'stale': 'stalled',
                          'offline': 'offline', 'unknown': 'unknown'}[simulation_availability]
                simulation = Simulation(backend='lite', status=status, availability=simulation_availability,
                                        engine_session_id=engine['session_id'], sim_time=engine['sim_time'],
                                        real_time_factor=engine['rtf'], target_real_time_factor=engine['target_rtf'],
                                        overruns=engine['overruns'], scene_sha256=engine['scene_sha256'],
                                        age_seconds=now-receipt)
            interlock_receipt = self.interlock[0] if self.interlock else None
            navigation_availability = self.availability(interlock_receipt, now)
            robots = []
            for e in (engine['entities'] if engine else ()):
                if e.type != 'amr':
                    continue
                plan = self.plans.get(e.id)
                robots.append(RobotNavigation(
                    id=e.id, interlock=self.interlock[1] if self.interlock else None,
                    interlock_ready=bool(self.interlock and self.interlock[1] == 'ready' and navigation_availability == 'online'),
                    interlock_availability=navigation_availability,
                    path=plan[1] if plan else (), path_age_seconds=now-plan[0] if plan else None, path_frame='world',
                    path_omitted_reason=plan[2] if plan else None, mission=self.missions.get(e.id)))
            self.sequence += 1
            return WorldState(
                session_id=self.session_id, sequence=self.sequence, timestamp=self.wall_clock(),
                server_wall_time=self.wall_time(), simulation=simulation,
                connection=Connection(gateway='online', ros='online' if self.ros_initialized else 'offline',
                                      simulation=simulation_availability, navigation=navigation_availability,
                                      registration='verified' if self.registration else 'missing'),
                entities=engine['entities'] if engine else (), robots=tuple(robots),
                obstacles=engine['obstacles'] if engine else (), events=tuple(self.events))


def field_name(value):
    return FIELDS[value] if 0 <= value < len(FIELDS) else 'protective'
