"""Small thread-safe latest-value store. Timeouts use monotonic wall time."""
from datetime import datetime, timezone
from threading import Lock
from time import monotonic
from uuid import uuid4
from .models import ConnectionStatus, Robot, TopicStatus, TwinState, World, TwinStateV2, DynamicEntity, SimulationState
from .models import NavigationDiagnostic, NavigationStatus


def utcnow():
    return datetime.now(timezone.utc)


class StateManager:
    def __init__(self, settings, timer=monotonic, wall_clock=utcnow):
        self.settings, self.timer, self.wall_clock = settings, timer, wall_clock
        self.lock = Lock()
        self.started = timer()
        self.session_id = str(uuid4())
        self.sequence = 0
        self.ros_initialized = False
        self.robots = {}
        self.entities = {}
        self.entity_receipts = {}
        self.simulation_epoch = 0
        self.pose_time = None
        self.latest_v2 = None
        topics = [settings.clock_topic, settings.tf_topic, settings.tf_static_topic]
        topics += [t for r in settings.robots for t in (r.odom_topic, r.joint_states_topic)]
        topics.append(settings.simulation_pose_topic)
        self.topics = dict.fromkeys(topics)
        self.simulation_time = None
        self.clock_progress = None
        self.latest = None
        self.navigation = {}

    def update_navigation(self, message):
        with self.lock:
            for status in message.status:
                if status.name not in ('navigation/interlock', 'navigation/action'):
                    continue
                self.navigation[status.name] = (self.timer(), self.wall_clock(),
                    int(status.level), status.message[:1024],
                    {v.key[:64]: v.value[:2048] for v in status.values[:16]})

    def navigation_snapshot(self):
        with self.lock:
            now = self.timer()
            return NavigationStatus(diagnostics=tuple(NavigationDiagnostic(
                name=name, level=level, message=message, values=values,
                received_at=timestamp, age_seconds=max(0., now-receipt),
                availability=self.availability(receipt, now))
                for name, (receipt, timestamp, level, message, values) in self.navigation.items()))

    def set_ros_initialized(self, value):
        with self.lock:
            self.ros_initialized = value

    def _touch(self, topic):
        old = self.topics[topic]
        self.topics[topic] = (self.timer(), self.wall_clock(), old[2] + 1 if old else 1)

    def touch(self, topic):
        with self.lock:
            self._touch(topic)

    def update_robot(self, robot):
        with self.lock:
            self.robots[robot.id] = robot
            self._touch(robot.pose_source)

    def update_entities(self, entities):
        with self.lock:
            if not entities:
                return
            stamp = entities[0].source_timestamp
            if self.pose_time is not None and stamp < self.pose_time:
                self.simulation_epoch += 1
                self.entities.clear()
                self.entity_receipts.clear()
            self.pose_time = stamp
            self._touch(self.settings.simulation_pose_topic)
            for entity in entities:
                previous = self.entities.get(entity.id)
                # Repeated frozen poses are not evidence of simulation progress.
                if previous is None or entity.source_timestamp != previous.source_timestamp:
                    self.entities[entity.id] = entity
                    self.entity_receipts[entity.id] = self.timer()

    def update_clock(self, value):
        with self.lock:
            if self.simulation_time is not None and value < self.simulation_time:
                self.simulation_epoch += 1
                self.entities.clear()
                self.entity_receipts.clear()
                self.pose_time = None
            # The first sample alone does not prove that the simulation is running.
            if self.simulation_time is not None and value != self.simulation_time:
                self.clock_progress = self.timer()
            self.simulation_time = value
            self._touch(self.settings.clock_topic)

    def availability(self, received, now, static=False):
        if not self.ros_initialized:
            return 'offline'
        if received is None:
            return 'unknown' if now - self.started < self.settings.offline_seconds else 'offline'
        if static:
            return 'online'
        age = now - received
        if age >= self.settings.offline_seconds:
            return 'offline'
        return 'stale' if age >= self.settings.stale_seconds else 'online'

    def publish(self, clients=0, measured_hz=0):
        with self.lock:
            now, timestamp = self.timer(), self.wall_clock()
            simulation = self.availability(self.clock_progress, now)
            robots = []
            for config in self.settings.robots:
                record = self.topics[config.odom_topic]
                status = self.availability(record[0] if record else None, now)
                robot = self.robots.get(config.id)
                if robot is None:
                    robot = Robot(id=config.id, frame_id=None, child_frame_id=None,
                                  pose_source=config.odom_topic, pose=None, velocity=None,
                                  source_timestamp=None, received_at=None, age_seconds=None,
                                  status=status)
                else:
                    robot = robot.model_copy(update={'status': status, 'age_seconds': now-record[0]})
                robots.append(robot)
            receipts = [r.received_at for r in robots if r.received_at is not None]
            connection = ConnectionStatus(
                session_id=self.session_id, gateway='online', ros_initialized=self.ros_initialized,
                ros='online' if self.ros_initialized else 'offline', simulation=simulation,
                last_odometry_timestamp=max(receipts) if receipts else None,
                active_websocket_clients=clients, configured_publication_hz=self.settings.publication_hz,
                measured_publication_hz=measured_hz,
                topics=tuple(TopicStatus(
                    topic=topic, status=self.availability(
                        record[0] if record else None, now, topic == self.settings.tf_static_topic),
                    received_at=record[1] if record else None,
                    age_seconds=now-record[0] if record else None,
                    messages_received=record[2] if record else 0,
                ) for topic, record in self.topics.items()),
            )
            self.sequence += 1
            self.latest = TwinState(
                session_id=self.session_id, sequence=self.sequence, timestamp=timestamp,
                simulation_time=self.simulation_time,
                world=World(id=self.settings.world_id, frame_id=self.settings.world_frame,
                            scenario=self.settings.scenario, scenario_source=self.settings.scenario_source,
                            availability=simulation, status='running' if simulation == 'online' else
                            ('unknown' if simulation == 'unknown' else 'unavailable')),
                robots=tuple(robots), connection=connection,
            )
            dynamic = []
            for name, identity in self.settings.simulation_entities.items():
                receipt = self.entity_receipts.get(identity)
                status = self.availability(receipt, now)
                entity = self.entities.get(identity)
                if entity is None:
                    kind = 'robot' if identity in {r.id for r in self.settings.robots} else (
                        'human' if identity.startswith('human_') else 'obstacle')
                    entity = DynamicEntity(id=identity, type=kind, gazebo_name=name,
                        frame_id=self.settings.world_frame, pose=None, source_timestamp=None,
                        received_at=None, age_seconds=None, status=status,
                        pose_source=self.settings.simulation_pose_topic)
                else:
                    entity = entity.model_copy(update={'status': status, 'age_seconds': now-receipt})
                dynamic.append(entity)
            self.latest_v2 = TwinStateV2(
                **self.latest.model_dump(exclude={'protocol_version', 'humans', 'obstacles'}),
                simulation=SimulationState(epoch=self.simulation_epoch,
                    status='running' if simulation == 'online' else
                    'paused_or_stalled' if simulation == 'stale' else simulation,
                    pose_topic=self.settings.simulation_pose_topic),
                simulation_robots=tuple(e for e in dynamic if e.type == 'robot'),
                humans=tuple(e for e in dynamic if e.type == 'human'),
                obstacles=tuple(e for e in dynamic if e.type == 'obstacle'))
            return self.latest

    def snapshot(self, version=1):
        with self.lock:
            return self.latest_v2 if version == 2 else self.latest
