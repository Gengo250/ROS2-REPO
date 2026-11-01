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

    def plan(msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if all(math.isfinite(v) for point in points for v in point):
            store.update_plan(settings.robots[0].id, points, msg.header.frame_id)

    def diagnostics(msg):
        for status in msg.status:
            if status.name == 'navigation/interlock':
                store.update_interlock(status.message)

    return [node.create_subscription(WorldState, settings.lite_world_topic, world, LIVE),
            node.create_subscription(SimEvent, settings.lite_events_topic, event, EVENTS),
            node.create_subscription(Path, settings.plan_topic, plan,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.VOLATILE)),
            node.create_subscription(DiagnosticArray, '/navigation/diagnostics', diagnostics, LIVE)]


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


class Commands:
    def __init__(self, node, store, settings, stations):
        from amr_lite_interfaces.srv import AddObstacle, RemoveObstacle
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        self.node, self.store, self.settings, self.stations = node, store, settings, stations
        self.AddObstacle, self.RemoveObstacle, self.NavigateToPose = AddObstacle, RemoveObstacle, NavigateToPose
        self.add_client = node.create_client(AddObstacle, '/lite_sim/add_obstacle')
        self.remove_client = node.create_client(RemoveObstacle, '/lite_sim/remove_obstacle')
        self.navigate = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.goals = {}

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

    async def start_mission(self, request):
        registration = self.store.registration
        station = self.stations.get(request.destination)
        if station is None:
            return CommandResult(accepted=False, reason=f'unknown destination {request.destination!r}')
        if registration is None:
            return CommandResult(accepted=False, reason='world/map registration unavailable')
        if request.robot_id != self.settings.robots[0].id:
            return CommandResult(accepted=False, reason=f'unknown robot {request.robot_id!r}')
        active = self.store.mission(request.robot_id)
        if active and active.status in ('pending', 'active'):
            return CommandResult(accepted=False, reason=f'mission {active.id} is still {active.status}; cancel it first')
        if not self.navigate.server_is_ready():
            return CommandResult(accepted=False, reason='Nav2 navigate_to_pose unavailable')
        wx, wy, wyaw = station['pose'][0], station['pose'][1], station['pose'][5]
        mx, my = registration.world_to_map(wx, wy)
        myaw = registration.world_yaw_to_map(wyaw)
        now = datetime.now(timezone.utc)
        mission = Mission(id=f'mission_{uuid4().hex[:8]}', robot_id=request.robot_id, destination=request.destination,
                          strategy='direct', status='pending', detail=None, goal=(wx, wy, wyaw),
                          distance_remaining_m=None, recoveries=0, created_at=now, updated_at=now)
        self.store.update_mission(mission)
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = mx, my
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(myaw/2), math.cos(myaw/2)
        loop = asyncio.get_running_loop()

        def feedback(message):
            current = self.store.mission(request.robot_id)
            if current is not None and current.id == mission.id and current.status in ('pending', 'active'):
                f = message.feedback
                self.store.update_mission(self._mission(current, status='active',
                                                        distance_remaining_m=max(0., f.distance_remaining),
                                                        recoveries=f.number_of_recoveries))
        handle = await bridge(self.navigate.send_goal_async(goal, feedback_callback=feedback), loop,
                              self.settings.command_timeout)
        if not handle.accepted:
            rejected = self._mission(mission, status='rejected', detail='Nav2 rejected the goal')
            self.store.update_mission(rejected)
            return CommandResult(accepted=False, reason='Nav2 rejected the goal', mission=rejected)
        self.goals[mission.id] = handle
        started = self._mission(mission, status='active')
        self.store.update_mission(started)

        def finished(future):
            from action_msgs.msg import GoalStatus
            current = self.store.mission(request.robot_id)
            if current is None or current.id != mission.id:
                return
            try:
                result = future.result()
                status = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled'}.get(
                    result.status, 'failed')
                detail = None if status == 'succeeded' else f'Nav2 status {result.status}, error {result.result.error_code}'
            except Exception as error:  # executor shutdown or transport failure
                status, detail = 'failed', str(error)[:256]
            self.store.update_mission(self._mission(current, status=status, detail=detail))
            self.goals.pop(mission.id, None)
        handle.get_result_async().add_done_callback(finished)
        return CommandResult(accepted=True, reason=None, mission=started)

    async def cancel_mission(self, robot_id):
        current = self.store.mission(robot_id)
        if current is None or current.status not in ('pending', 'active'):
            return CommandResult(accepted=False, reason='no active mission')
        handle = self.goals.get(current.id)
        if handle is None:
            return CommandResult(accepted=False, reason='mission has no Nav2 goal handle')
        await bridge(handle.cancel_goal_async(), asyncio.get_running_loop(), self.settings.command_timeout)
        return CommandResult(accepted=True, reason=None, mission=self.store.mission(robot_id))
