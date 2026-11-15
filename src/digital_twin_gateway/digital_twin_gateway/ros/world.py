"""Lite-engine subscriptions and validated commands for world-state v1.

Subscriptions normalize ROS messages once and hand plain values to the WorldStore.
Commands run as rclpy service/action calls whose futures complete on the executor thread
and are bridged to asyncio, so the API loop never blocks on ROS.
"""
import asyncio
import math
from datetime import datetime, timezone
from uuid import uuid4
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ..world.models import CommandResult, Mission, WorldObstacle
from ..world.store import field_name

LIVE = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
EVENTS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)


def normalize_world(msg):
    def obstacle(o):
        return dict(id=o.id, kind=o.kind, origin=o.origin, x=o.x, y=o.y, yaw=o.yaw,
                    length=o.length, width=o.width, height=o.height)
    entities = []
    for e in msg.entities:
        values = [e.x, e.y, e.z, e.yaw, e.vx, e.vy, e.wz, e.speed]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('non-finite entity state')
        entities.append(dict(id=e.id, type=e.type, x=e.x, y=e.y, z=e.z, yaw=e.yaw, vx=e.vx, vy=e.vy, wz=e.wz,
                             speed=e.speed, state=e.state, activity=e.activity or None,
                             activity_progress=min(1., max(0., e.activity_progress)), carrying=e.carrying,
                             look=e.look, field_state=field_name(e.field_state), field_length=max(0., e.field_length),
                             reason=e.reason or None))
    return dict(session_id=msg.session_id, sim_time=msg.sim_time, rtf=msg.real_time_factor,
                target_rtf=msg.target_real_time_factor, paused=msg.paused, overruns=msg.overruns,
                scene_sha256=msg.scene_sha256, entities=entities, obstacles=[obstacle(o) for o in msg.obstacles])


def subscribe_world(node, store, settings):
    """Subscriptions for world-state v1; empty when the lite interfaces are not installed."""
    try:
        from amr_lite_interfaces.msg import SimEvent, WorldState
    except ImportError:
        node.get_logger().warning('amr_lite_interfaces not installed: world-state v1 has no engine data')
        return []
    from nav_msgs.msg import Path
    from diagnostic_msgs.msg import DiagnosticArray

    def world(msg):
        try:
            store.update_engine(normalize_world(msg))
        except (ValueError, TypeError) as error:
            node.get_logger().warning(f'Invalid world state rejected: {error}', throttle_duration_sec=5)

    def event(msg):
        try:
            store.add_event(dict(seq=msg.seq, sim_time=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                 kind=msg.kind[:64], entity=msg.entity[:64], other=msg.other[:64],
                                 detail=msg.detail[:256], x=msg.x, y=msg.y))
        except (ValueError, TypeError):
            node.get_logger().warning('Invalid simulation event rejected', throttle_duration_sec=5)

    def plan(robot_id, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if all(math.isfinite(v) for point in points for v in point):
            store.update_plan(robot_id, points, msg.header.frame_id)

    def diagnostics(robot_id, msg):
        for status in msg.status:
            if status.name == 'navigation/interlock':
                store.update_interlock(robot_id, status.message)

    subscriptions = [node.create_subscription(WorldState, settings.lite_world_topic, world, LIVE),
                     node.create_subscription(SimEvent, settings.lite_events_topic, event, EVENTS)]
    # One plan and one interlock per robot, each from that robot's own namespace: a plan or an
    # interlock state can never be attributed to another robot.
    for robot in settings.fleet:
        plan_topic = settings.plan_topic if not robot.namespace else robot.topic('plan')
        subscriptions += [
            node.create_subscription(Path, plan_topic, lambda msg, r=robot.id: plan(r, msg),
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.VOLATILE)),
            node.create_subscription(DiagnosticArray, robot.topic('navigation/diagnostics'),
                                     lambda msg, r=robot.id: diagnostics(r, msg), LIVE)]
    return subscriptions


def bridge(ros_future, loop, timeout):
    """Await an rclpy future from asyncio; resolved on the executor thread."""
    future = loop.create_future()

    def done(f):
        def deliver():
            if future.done():
                return
            if f.exception() is not None:
                future.set_exception(f.exception())
            else:
                future.set_result(f.result())
        loop.call_soon_threadsafe(deliver)
    ros_future.add_done_callback(done)
    return asyncio.wait_for(future, timeout)


DRIVING = ('pending', 'active', 'recovering')
ENDED = ('succeeded', 'failed', 'canceled', 'rejected', 'blocked')
WAIT_FIELDS = dict(waiting_for=None, waiting_on=None, blocking_robot=None, queue_position=None)


def waiting_fields(info):
    return dict(waiting_for=info.kind, waiting_on=info.on, blocking_robot=info.blocker, queue_position=info.position)


class Commands:
    """Mission and obstacle commands. Missions go through the fleet coordinator (world/traffic.py):
    a station is reserved before the robot's Nav2 goal is sent, otherwise the mission waits in that
    station's queue; the coordinator's periodic step (coordinate()) dispatches queued missions, holds
    a goal (cancel) before a busy narrow corridor or while yielding to another AMR, sends it again
    (resume) when clear, and stops robots in an unresolvable conflict (blocked). Only Nav2 actions
    are used; each robot's command guard remains its only cmd_vel writer."""

    def __init__(self, node, store, settings, stations, corridors=(), vehicles=None):
        from amr_lite_interfaces.srv import AddObstacle, RemoveObstacle
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from ..world.traffic import Traffic
        self.node, self.store, self.settings, self.stations = node, store, settings, stations
        self.AddObstacle, self.RemoveObstacle, self.NavigateToPose = AddObstacle, RemoveObstacle, NavigateToPose
        self.add_client = node.create_client(AddObstacle, '/lite_sim/add_obstacle')
        self.remove_client = node.create_client(RemoveObstacle, '/lite_sim/remove_obstacle')
        # One NavigateToPose client per robot, on that robot's own Nav2 (its namespace).
        self.navigate = {r.id: ActionClient(node, NavigateToPose, r.topic('navigate_to_pose')) for r in settings.fleet}
        self.traffic = Traffic({k: v['pose'] for k, v in stations.items()}, corridors, vehicles=vehicles)
        self.goals = {}        # mission id → goal handle of its current Nav2 goal
        self.tokens = {}       # mission id → token of the goal whose callbacks may still update it
        self.intent = {}       # mission id → 'hold' | 'block': why the coordinator cancelled its goal
        self.recovery_mark = {}

    async def add_obstacle(self, request):
        if not self.add_client.service_is_ready():
            return CommandResult(accepted=False, reason='lite engine unavailable')
        r = self.AddObstacle.Request(kind=request.kind, x=float(request.x), y=float(request.y), yaw=float(request.yaw))
        response = await bridge(self.add_client.call_async(r), asyncio.get_running_loop(), self.settings.command_timeout)
        if not response.accepted:
            return CommandResult(accepted=False, reason=response.reason)
        o = response.obstacle
        return CommandResult(accepted=True, reason=None, obstacle=WorldObstacle(
            id=o.id, kind=o.kind, origin=o.origin, x=o.x, y=o.y, yaw=o.yaw, length=o.length, width=o.width, height=o.height))

    async def remove_obstacle(self, identifier=''):
        if not self.remove_client.service_is_ready():
            return CommandResult(accepted=False, reason='lite engine unavailable')
        response = await bridge(self.remove_client.call_async(self.RemoveObstacle.Request(id=identifier)),
                                asyncio.get_running_loop(), self.settings.command_timeout)
        return CommandResult(accepted=response.accepted, reason=response.reason or None, removed=response.removed)

    def _mission(self, previous, **changes):
        return previous.model_copy(update={**changes, 'updated_at': datetime.now(timezone.utc)})

    def _amrs(self):
        view = self.store.coordination_view()
        return {e.id: e for e in view[1] if e.type == 'amr'} if view else {}

    async def start_mission(self, request):
        registration = self.store.registration
        station = self.stations.get(request.destination)
        if station is None:
            return CommandResult(accepted=False, reason=f'unknown destination {request.destination!r}')
        if registration is None:
            return CommandResult(accepted=False, reason='world/map registration unavailable')
        navigate = self.navigate.get(request.robot_id)
        if navigate is None:
            return CommandResult(accepted=False, reason=f'unknown robot {request.robot_id!r}')
        active = self.store.mission(request.robot_id)
        if active and active.status in DRIVING:
            return CommandResult(accepted=False, reason=f'mission {active.id} is still {active.status}; cancel it first')
        if not navigate.server_is_ready():
            return CommandResult(accepted=False, reason=f'Nav2 navigate_to_pose of {request.robot_id} unavailable')
        if active and active.status == 'waiting':
            # A waiting mission has no Nav2 goal running: the new request replaces it.
            self._end(active, 'canceled', f'replaced by a mission to {request.destination}')
        wx, wy, wyaw = station['pose'][0], station['pose'][1], station['pose'][5]
        now = datetime.now(timezone.utc)
        mission = Mission(id=f'mission_{uuid4().hex[:8]}', robot_id=request.robot_id, destination=request.destination,
                          strategy='direct', status='pending', detail=None, goal=(wx, wy, wyaw),
                          distance_remaining_m=None, recoveries=0, created_at=now, updated_at=now)
        # Reservation decision before any await: simultaneous requests are decided one at a time.
        decision, info = self.traffic.request(request.robot_id, request.destination, self._amrs())
        if decision == 'reject':
            rejected = self._mission(mission, status='rejected', detail=info)
            self.store.update_mission(rejected)
            return CommandResult(accepted=False, reason=info, mission=rejected)
        if decision == 'queue':
            waiting = self._mission(mission, status='waiting', **waiting_fields(info))
            self.store.update_mission(waiting)
            return CommandResult(accepted=True, reason=None, mission=waiting)
        return await self._send(mission)

    def _end(self, mission, status, detail=None):
        """Mission over (no Nav2 goal left running for it): release everything it held."""
        # A blocked mission keeps naming the robot it conflicts with.
        fields = WAIT_FIELDS if status != 'blocked' else {**WAIT_FIELDS, 'blocking_robot': mission.blocking_robot}
        self.store.update_mission(self._mission(mission, status=status, detail=detail, **fields))
        self.traffic.mission_ended(mission.robot_id)
        for table in (self.goals, self.tokens, self.intent, self.recovery_mark):
            table.pop(mission.id, None)

    async def _send(self, mission):
        """Send (or send again, on resume) the mission's Nav2 goal; returns the command result."""
        robot = mission.robot_id
        navigate = self.navigate[robot]
        if not navigate.server_is_ready():
            self._end(mission, 'failed', f'Nav2 navigate_to_pose of {robot} unavailable')
            return CommandResult(accepted=False, reason=f'Nav2 navigate_to_pose of {robot} unavailable', mission=self.store.mission(robot))
        registration = self.store.registration
        wx, wy, wyaw = mission.goal
        mx, my = registration.world_to_map(wx, wy)
        myaw = registration.world_yaw_to_map(wyaw)
        pending = self._mission(mission, status='pending', **WAIT_FIELDS)
        self.store.update_mission(pending)
        self.intent.pop(mission.id, None)
        token = self.tokens[mission.id] = object()
        base = mission.recoveries
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = mx, my
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(myaw/2), math.cos(myaw/2)
        loop = asyncio.get_running_loop()

        def feedback(message):
            current = self.store.mission(robot)
            if self.tokens.get(mission.id) is not token or mission.id in self.intent or current is None or current.id != mission.id \
                    or current.status not in DRIVING:
                return
            f = message.feedback
            remaining, recoveries = max(0., f.distance_remaining), base+f.number_of_recoveries
            status = 'active' if current.status == 'pending' else current.status
            if recoveries > current.recoveries:
                status, self.recovery_mark[mission.id] = 'recovering', remaining
            elif status == 'recovering' and remaining < self.recovery_mark.get(mission.id, math.inf)-.5:
                status = 'active'
            self.store.update_mission(self._mission(current, status=status, distance_remaining_m=remaining, recoveries=recoveries))
        handle = await bridge(navigate.send_goal_async(goal, feedback_callback=feedback), loop,
                              self.settings.command_timeout)
        if self.tokens.get(mission.id) is not token:
            # Cancelled or replaced while the goal was being sent: that goal must not keep driving.
            if handle.accepted:
                await bridge(handle.cancel_goal_async(), loop, self.settings.command_timeout)
            return CommandResult(accepted=False, reason='mission cancelled or replaced', mission=self.store.mission(robot))
        if not handle.accepted:
            self._end(pending, 'rejected', 'Nav2 rejected the goal')
            return CommandResult(accepted=False, reason='Nav2 rejected the goal', mission=self.store.mission(robot))
        self.goals[mission.id] = handle
        current = self.store.mission(robot)
        if current is not None and current.id == mission.id and current.status == 'pending':
            self.store.update_mission(self._mission(current, status='active'))

        def finished(future):
            from action_msgs.msg import GoalStatus
            current = self.store.mission(robot)
            if self.tokens.get(mission.id) is not token or current is None or current.id != mission.id:
                return
            try:
                result = future.result()
                status = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled'}.get(
                    result.status, 'failed')
                detail = None if status == 'succeeded' else f'Nav2 status {result.status}, error {result.result.error_code}'
            except Exception as error:  # executor shutdown or transport failure
                status, detail = 'failed', str(error)[:256]
            intent = self.intent.pop(mission.id, None)
            self.goals.pop(mission.id, None)
            if status != 'succeeded' and intent == 'hold':
                return                         # held by the coordinator: stays waiting, resumed later
            if status != 'succeeded' and intent == 'block':
                status, detail = 'blocked', current.detail
            self._end(current, status, detail)
        handle.get_result_async().add_done_callback(finished)
        return CommandResult(accepted=True, reason=None, mission=self.store.mission(robot))

    async def cancel_mission(self, robot_id):
        current = self.store.mission(robot_id)
        if current is None or current.status not in DRIVING+('waiting',):
            return CommandResult(accepted=False, reason='no active mission')
        handle = self.goals.get(current.id)
        if current.status == 'waiting':
            # Queued or held: no goal is driving the robot (a hold's cancel may still be in flight).
            self._end(current, 'canceled', 'canceled while waiting')
            if handle is not None:
                await bridge(handle.cancel_goal_async(), asyncio.get_running_loop(), self.settings.command_timeout)
            return CommandResult(accepted=True, reason=None, mission=self.store.mission(robot_id))
        if handle is None:
            return CommandResult(accepted=False, reason='mission has no Nav2 goal handle')
        self.intent.pop(current.id, None)
        await bridge(handle.cancel_goal_async(), asyncio.get_running_loop(), self.settings.command_timeout)
        return CommandResult(accepted=True, reason=None, mission=self.store.mission(robot_id))

    # ---------------------------------------------------------------- fleet coordination
    async def coordinate(self):
        """One coordination step (called periodically by the API loop)."""
        view = self.store.coordination_view()
        if view is None:
            return
        sim, entities, missions, plans = view
        actions, waiting = self.traffic.step(sim, entities, missions, plans)
        for robot, info in waiting.items():
            m = missions.get(robot)
            fields = waiting_fields(info)
            if m is not None and m.status == 'waiting' and any(getattr(m, k) != v for k, v in fields.items()):
                self.store.update_mission(self._mission(m, **fields))
        for action in actions:
            kind, robot = action[0], action[1]
            m = self.store.mission(robot)
            if m is None:
                continue
            try:
                if kind in ('dispatch', 'resume') and m.status == 'waiting':
                    await self._send(m)
                elif kind == 'hold' and m.status in DRIVING:
                    await self._stop(m, 'hold', status='waiting', **waiting_fields(action[2]))
                elif kind == 'block' and m.status in DRIVING+('waiting',):
                    await self._stop(m, 'block', status='blocked', detail=f'persistent conflict with {action[2].on}',
                                     **waiting_fields(action[2]))
            except (asyncio.TimeoutError, RuntimeError) as error:
                self.node.get_logger().warning(f'coordination {kind} {robot} failed: {error}')
        stations, corridors = self.traffic.describe(entities)
        self.store.update_reservations(stations, corridors)

    async def _stop(self, mission, intent, **changes):
        """Cancel the mission's Nav2 goal on behalf of the coordinator (hold or block)."""
        handle = self.goals.get(mission.id)
        stopped = self._mission(mission, **changes)
        if intent == 'block':
            stopped = stopped.model_copy(update=WAIT_FIELDS | {k: v for k, v in changes.items() if k in WAIT_FIELDS})
            self.traffic.mission_ended(mission.robot_id)
        self.store.update_mission(stopped)
        if handle is None:
            if intent == 'block':
                self._end(stopped, 'blocked', stopped.detail)
            return
        self.intent[mission.id] = intent
        await bridge(handle.cancel_goal_async(), asyncio.get_running_loop(), self.settings.command_timeout)
