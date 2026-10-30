"""Execute one validated warehouse route through Nav2; track, replan, report.

The graph decides the operational route; Nav2 NavigateThroughPoses plans the
collision-aware geometry and drives through the existing command guard. This
process never publishes velocity. Gazebo ground truth is recorded only for
offline evaluation and never influences a decision.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time
import uuid
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from . import coordinate_registration as registration
from . import map_validation as mv
from .cost_model import Restrictions, wrap
from .graph_builder import GraphStaleError, Paths
from .route_planner import RoutingError, inside_lanes, load_context, plan, resolve, route_lanes

STATES = ('IDLE', 'PLANNING', 'PLANNED', 'VALIDATING', 'EXECUTING', 'REPLANNING', 'COMPLETED', 'BLOCKED', 'FAILED', 'CANCELED')
EXIT = {'COMPLETED': 0, 'FAILED': 1, 'invalid_request': 2, 'BLOCKED': 3, 'incompatible': 4, 'unavailable': 5,
        'CANCELED': 6, 'timeout': 7}
STATUS_TOPIC = '/route_planner/status'
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def path_length(points):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))


def blocked_sections(costmap, start, end, width, skip=None, step=.1):
    """Cross-sections of a corridor (map frame) with no cell the robot centre may occupy.

    Uses Nav2's own inflated global costmap: values 0–98 are traversable for the
    centre; 99 inscribed, 100 lethal and -1 unknown are not.
    """
    info = costmap.info
    length = math.dist(start, end)
    if length < 1e-6:
        return []
    ux, uy = (end[0]-start[0])/length, (end[1]-start[1])/length
    blocked = []
    for k in range(int(length/step)+1):
        s = k*step
        cx, cy = start[0]+ux*s, start[1]+uy*s
        if skip is not None and math.dist((cx, cy), skip) < 1.0:
            continue
        passable = False
        offsets = int(width/2/info.resolution)
        for j in range(-offsets, offsets+1):
            px, py = cx-uy*j*info.resolution, cy+ux*j*info.resolution
            i = int(math.floor((px-info.origin.position.x)/info.resolution))
            r = int(math.floor((py-info.origin.position.y)/info.resolution))
            if 0 <= i < info.width and 0 <= r < info.height and 0 <= costmap.data[r*info.width+i] <= 98:
                passable = True
                break
        if not passable:
            blocked.append([round(cx, 3), round(cy, 3)])
    return blocked


class Executor:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('route_executor', parameter_overrides=[Parameter('use_sim_time', value=True)])
        n = self.node
        self.buffer = Buffer(node=n)
        self.listener = TransformListener(self.buffer, n)
        self.latest, self.plans, self.commands = {}, [], 0
        self.runtime_blocks, self.unblocks = {}, set()
        n.create_subscription(OccupancyGrid, '/map', lambda m: self.latest.update(map=m),
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: self.latest.update(costmap=m),
                              QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        # AMCL latches its last estimate and republishes only after motion: a late-starting
        # executor needs the transient-local copy; the volatile one also matches a volatile publisher.
        for durability in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: self.latest.update(amcl=m),
                                  QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=durability))
        n.create_subscription(Odometry, '/odom', lambda m: self.latest.update(odom=m), qos_profile_sensor_data)
        n.create_subscription(NavPath, '/plan', self.on_plan, 10)
        n.create_subscription(TwistStamped, '/cmd_vel', self.on_command, 10)
        n.create_subscription(TFMessage, '/digital_twin/simulation/poses', self.on_ground_truth, qos_profile_sensor_data)
        n.create_subscription(String, '/route_planner/block_connection', lambda m: self.on_block(m.data, True), 10)
        n.create_subscription(String, '/route_planner/unblock_connection', lambda m: self.on_block(m.data, False), 10)
        self.status_pub = n.create_publisher(String, STATUS_TOPIC, LATCHED)
        self.client = ActionClient(n, NavigateThroughPoses, args.action)
        self.cancel_requested = None
        self.report = {'request': {'from': args.origin, 'to': args.destination, 'algorithm': args.algorithm,
                                   'scenario': args.scenario, 'blocked': args.block},
                       'status_history': [], 'routes': [], 'replans': [], 'trajectory': [], 'feedback': [],
                       'plan_checks': [], 'restriction_events': [], 'collision_events': None}
        self.state = {'status': 'IDLE', 'route_id': None, 'route': None, 'completed': 0, 'remaining_m': None,
                      'action': None, 'failure': None, 'detail': None, 'blocked': []}
        self.last_sample = self.last_status = 0.

    # ---- inputs -------------------------------------------------------------------------
    def on_command(self, _):
        self.commands += 1

    def on_plan(self, msg):
        if msg.header.frame_id == 'map' and self.state['status'] == 'EXECUTING':
            self.plans.append([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def on_ground_truth(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'world' and t.child_frame_id == 'amr_01':
                self.latest['world'] = [t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)]

    def on_block(self, connection, blocked):
        connection = connection.strip()
        self.report['restriction_events'].append({'wall': time.monotonic(), 'connection': connection, 'blocked': blocked})
        if blocked:
            self.runtime_blocks[connection] = math.inf
        else:
            self.runtime_blocks.pop(connection, None)

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=.03)
        wall = time.monotonic()
        if wall-self.last_sample > .2:
            self.last_sample = wall
            pose = self.pose()
            odom = self.latest.get('odom')
            row = {'wall': wall, 'sim': self.node.get_clock().now().nanoseconds/1e9, 'map': pose,
                   'world': self.latest.get('world')}
            if odom:
                row['odom'] = [odom.pose.pose.position.x, odom.pose.pose.position.y, yaw_of(odom.pose.pose.orientation)]
                row['velocity'] = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]
            self.report['trajectory'].append(row)
        if wall-self.last_status > 1.:
            self.publish()

    def pose(self):
        try:
            t = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return [t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)]

    # ---- status -------------------------------------------------------------------------
    def set(self, status, **fields):
        assert status in STATES
        self.state.update(status=status, **fields)
        self.report['status_history'].append({'wall': time.monotonic(), 'status': status,
                                              **{k: v for k, v in fields.items() if k != 'route'}})
        self.node.get_logger().info(f"{status}: {fields.get('detail') or ''}")
        self.publish()

    def message(self):
        s, route = self.state, self.state['route'] or {}
        waypoints = route.get('waypoints', [])
        total = route.get('distance_m')
        remaining = s['remaining_m']
        progress = 1. if s['status'] == 'COMPLETED' else (
            max(0., min(1., 1-remaining/total)) if total and remaining is not None else 0.)
        return {'protocol_version': '1.0', 'message_type': 'route_status', 'route_id': s['route_id'],
                'status': s['status'], 'origin': self.args.origin, 'destination': self.args.destination,
                'algorithm': route.get('algorithm', self.args.algorithm), 'graph_version': route.get('graph_version'),
                'map_version': route.get('map_version'), 'registration_id': route.get('registration_id'),
                'selected_nodes': route.get('selected_nodes', []), 'selected_edges': route.get('selected_edges', []),
                'map_frame_path': route.get('map_frame_path', []),
                'map_frame_waypoints': [w['map'] for w in waypoints], 'waypoint_nodes': [w['node'] for w in waypoints],
                'current_waypoint': min(s['completed'], len(waypoints)-1) if waypoints and s['status'] == 'EXECUTING' else None,
                'completed_waypoints': s['completed'], 'total_waypoints': len(waypoints),
                'estimated_distance_m': total, 'distance_remaining_m': remaining, 'execution_progress': round(progress, 4),
                'replans': len(self.report['replans']), 'blocked_connections': sorted(s['blocked']),
                'navigation_action_status': s['action'], 'failure_reason': s['failure'],
                # Bounded so the Gateway's strict contract never rejects a failure report.
                'detail': None if s['detail'] is None else str(s['detail'])[:1000],
                'stamp': self.node.get_clock().now().nanoseconds/1e9,
                'updated_at': datetime.now(timezone.utc).isoformat()}

    def publish(self):
        self.last_status = time.monotonic()
        self.status_pub.publish(String(data=json.dumps(self.message())))

    # ---- preconditions ------------------------------------------------------------------
    def wait_ready(self, artifact):
        deadline = time.monotonic()+self.args.ready_timeout
        reason = 'waiting'
        while time.monotonic() < deadline and self.cancel_requested is None:
            self.spin()
            if 'map' not in self.latest:
                reason = 'map unavailable'
                continue
            if self.latest['map'] is not getattr(self, 'checked_map', None):
                self.checked_map = self.latest['map']
                self.live_map_version = mv.OccupancyMap.from_message(self.checked_map).version()
            if self.live_map_version != artifact['map']['version']:
                return 'incompatible', 'map_incompatible', 'Live /map differs from the map the graph was validated against'
            amcl, pose = self.latest.get('amcl'), self.pose()
            limits = self.settings['executor']
            if amcl is None or pose is None:
                reason = 'localization_unavailable'
                continue
            cov = amcl.pose.covariance
            if not all(math.isfinite(cov[i]) for i in (0, 7, 35)) or max(cov[0], cov[7]) > limits['max_position_variance_m2'] \
                    or cov[35] > limits['max_yaw_variance_rad2']:
                reason = 'localization_unavailable'
                continue
            if not self.client.server_is_ready():
                reason = 'nav2_unavailable'
                continue
            publishers = [(p.node_name, p.topic_type) for p in self.node.get_publishers_info_by_topic('/cmd_vel')]
            if publishers != [('navigation_command_guard', 'geometry_msgs/msg/TwistStamped')] or self.commands < 2:
                reason = 'nav2_unavailable'
                continue
            self.report['cmd_vel_publishers'] = publishers
            return None
        if self.cancel_requested:
            return 'CANCELED', 'canceled', 'Canceled before dispatch'
        return 'unavailable', reason, f'Preconditions not met within {self.args.ready_timeout} s: {reason}'

    def connector_check(self, route, pose):
        """Robot pose → its graph attachment point: no occupied cells; unknown only under the robot itself.

        The robot physically occupies its own footprint, so unobserved cells there (e.g.
        behind the spawn pose) are not a reason to refuse; everything else must be known free.
        """
        return self.connector_issue(pose, route['map_frame_path'][0])

    def connector_issue(self, pose, first):
        occupancy = mv.OccupancyMap.from_message(self.latest['map'])
        footprint = self.footprint
        here = mv.Box(pose[0]+math.cos(pose[2])*(footprint.front-footprint.back)/2,
                      pose[1]+math.sin(pose[2])*(footprint.front-footprint.back)/2,
                      pose[2], (footprint.front+footprint.back)/2, footprint.half_width)
        path = [here] if math.dist(pose[:2], first) < .3 else [mv.sweep(pose[:2], first, footprint)]
        counts = occupancy.region_counts(path)
        if counts['occupied']:
            return 'occupied map cells between the robot and the graph'
        if occupancy.region_counts(path, exclude=[here])['unknown'] or counts['outside']:
            return 'unknown map cells between the robot and the graph'
        return None

    # ---- planning -----------------------------------------------------------------------
    def restrictions(self):
        now = time.monotonic()
        blocks = {c for c, until in self.runtime_blocks.items() if until > now}
        return Restrictions(self.args.scenario, frozenset(set(self.args.block) | blocks))

    def retreat(self, pose):
        """The current route edge (connection, node behind the robot), allowing exit from a closed corridor."""
        route = self.state['route']
        if not route or not route.get('found') or pose is None:
            return None
        world = self.transform.inverse().apply(*pose[:2])
        best = None
        for (a, b), edge, node in zip(zip(route['path_world'], route['path_world'][1:]), route['connections'], route['selected_nodes']):
            length = math.dist(a, b)
            t = 0. if length < 1e-9 else max(0., min(1., ((world[0]-a[0])*(b[0]-a[0])+(world[1]-a[1])*(b[1]-a[1]))/length**2))
            d = math.dist(world, (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
            if node != 'robot_start' and (best is None or d < best[0]):
                best = (d, edge, node)
        return best[1:] if best else None

    def make_plan(self, artifact, pose, route_id):
        r = self.restrictions()
        report, graph = plan(artifact, self.transform, self.settings, 'current', self.args.destination,
                             self.args.algorithm, r, start_pose_map=pose, route_id=route_id, retreat=self.retreat(pose),
                             connector_ok=lambda world: self.connector_issue(pose, self.transform.apply(*world)) is None)
        report['robot_map_pose'] = pose
        self.report['routes'].append(report)
        self.state['blocked'] = sorted(r.blocked_connections)
        return report, graph

    def classify_no_route(self, artifact, pose):
        """BLOCKED if restrictions cut an otherwise validated connection; FAILED if none exists."""
        try:
            report, _ = plan(artifact, self.transform, self.settings, 'current', self.args.destination,
                             self.args.algorithm, Restrictions(None), start_pose_map=pose)
        except RoutingError:
            return 'FAILED'
        return 'BLOCKED' if report['found'] else 'FAILED'

    # ---- execution ----------------------------------------------------------------------
    def goal(self, route):
        g = NavigateThroughPoses.Goal()
        stamp = self.node.get_clock().now().to_msg()
        for w in route['waypoints']:
            p = PoseStamped()
            p.header.frame_id, p.header.stamp = 'map', stamp
            p.pose.position.x, p.pose.position.y = w['map'][0], w['map'][1]
            p.pose.orientation.z, p.pose.orientation.w = math.sin(w['map'][2]/2), math.cos(w['map'][2]/2)
            g.poses.append(p)
        return g

    def wait(self, future, seconds):
        deadline = time.monotonic()+seconds
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError('Action response deadline exceeded')
            self.spin()
        return future.result()

    def deviation(self, lanes, pose):
        """First Nav2 plan point leaving every route lane by more than the tolerance, ignoring the robot's surroundings."""
        tolerance = self.settings['executor']['plan_deviation_tolerance_m']
        inverse = self.transform.inverse()
        while self.plans:
            points = self.plans.pop(0)
            ahead = [p for p in points if pose is None or math.dist(p, pose[:2]) > 1.0]
            outside = [p for p in ahead if not inside_lanes(lanes, inverse.apply(*p), tolerance)]
            self.report['plan_checks'].append({'wall': time.monotonic(), 'points': len(points), 'outside': len(outside),
                                               'length_m': round(path_length(points), 3)})
            if len(self.report.setdefault('nav2_plans', [])) < 200:
                # Evidence of the geometric path Nav2 chose for the graph objectives (every 5th point).
                self.report['nav2_plans'].append({'wall': time.monotonic(), 'points': [[round(v, 3) for v in q] for q in points[::5]+points[-1:]]})
            if outside:
                return outside[0]
        return None

    def evidence(self, route, graph, pose):
        """Connections of the remaining route that Nav2's live global costmap shows impassable."""
        costmap = self.latest.get('costmap')
        if costmap is None:
            return {}
        found = {}
        passed = route['waypoints'][self.state['completed']-1]['route_index'] if self.state['completed'] else 0
        for edge_id in route['selected_edges'][passed:]:
            edge = graph.edges[edge_id]
            if edge.connection in found or edge.source == 'robot_start':
                continue
            a = self.transform.apply(*graph.nodes[edge.source].position)
            b = self.transform.apply(*graph.nodes[edge.target].position)
            sections = blocked_sections(costmap, a, b, edge.width, skip=pose[:2] if pose else None)
            if sections:
                found[edge.connection] = sections
        return found

    def expired(self, deadline):
        """Reason the route deadline (simulation time) or the wall-clock guard has passed, else None."""
        sim_deadline, wall_deadline = deadline
        if self.node.get_clock().now().nanoseconds/1e9 > sim_deadline:
            return f'Route timeout of {self.args.timeout} s (simulation time)'
        if time.monotonic() > wall_deadline:
            return f'Route wall-clock guard of {self.args.wall_guard} s (simulator stalled or too slow)'
        return None

    def execute(self, artifact, route, graph, deadline):
        """Dispatch one route. Returns ('done'|'replan'|'stop', status, failure, detail, evidence)."""
        handle = self.wait(self.client.send_goal_async(self.goal(route), feedback_callback=self.on_feedback), 15)
        if not handle.accepted:
            return 'stop', 'FAILED', 'nav2_rejected', 'Nav2 rejected the route objectives', None
        self.handle = handle
        self.plans.clear()
        total = len(route['waypoints'])
        self.state.update(completed=0, action='ACCEPTED')
        self.set('EXECUTING', detail=f'{total} Nav2 objectives dispatched', route=route, failure=None)
        lanes = route_lanes(graph, route['selected_edges'])
        result_future = handle.get_result_async()
        known_blocks = set(self.restrictions().blocked_connections)
        while not result_future.done():
            self.spin()
            if self.cancel_requested:
                self.cancel(handle)
                return 'stop', 'CANCELED', 'canceled', self.cancel_requested, None
            expired = self.expired(deadline)
            if expired:
                self.cancel(handle)
                return 'stop', 'FAILED', 'timeout', expired, None
            new = set(self.restrictions().blocked_connections)-known_blocks
            if new & set(route['connections']):
                self.cancel(handle)
                return 'replan', None, 'restriction', 'Operator closed '+','.join(sorted(new)), \
                    {c: 'operator restriction' for c in new}
            known_blocks |= new
            point = self.deviation(lanes, self.pose())
            if point is not None:
                found = self.evidence(route, graph, self.pose())
                self.cancel(handle)
                return 'replan', None, 'plan_deviation', f'Nav2 plan left the route corridors at {point}', found
        result = self.wait(result_future, 15)
        self.state['action'] = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                                GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(result.status, str(result.status))
        self.report.setdefault('action_results', []).append({'status': self.state['action'],
                                                             'error_code': result.result.error_code,
                                                             'error_msg': result.result.error_msg})
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.state['completed'] = total
            return 'done', None, None, None, None
        found = self.evidence(route, graph, self.pose())
        return 'replan', None, 'navigation_failed', f'Nav2 {self.state["action"]} error {result.result.error_code}: ' \
            f'{result.result.error_msg}', found

    def on_feedback(self, msg):
        f = msg.feedback
        total = len((self.state['route'] or {}).get('waypoints', []))
        # Nav2 removes passed intermediate poses; the destination completes only on success.
        self.state['completed'] = max(self.state['completed'], min(total-1, total-f.number_of_poses_remaining))
        self.state['remaining_m'] = round(float(f.distance_remaining), 3)
        self.state['action'] = 'EXECUTING'
        row = {'wall': time.monotonic(), 'poses_remaining': f.number_of_poses_remaining,
               'distance_remaining_m': f.distance_remaining, 'recoveries': f.number_of_recoveries,
               'navigation_time_s': f.navigation_time.sec+f.navigation_time.nanosec/1e9}
        if not self.report['feedback'] or row['wall']-self.report['feedback'][-1]['wall'] > .5 or \
                row['poses_remaining'] != self.report['feedback'][-1]['poses_remaining']:
            self.report['feedback'].append(row)

    def cancel(self, handle):
        try:
            response = self.wait(handle.cancel_goal_async(), 10)
            self.report.setdefault('cancel_responses', []).append(len(response.goals_canceling))
        except Exception as exc:
            self.report['cancel_error'] = repr(exc)
        self.state['action'] = 'CANCELED'

    def run(self):
        a = self.args
        self.set('PLANNING', detail='Loading graph, registration and map')
        try:
            paths = Paths.candidate(a.candidate) if a.candidate else Paths()
            artifact, self.transform, self.settings = load_context(paths)
            self.paths = paths
        except (GraphStaleError, registration.RegistrationError, OSError, ValueError) as exc:
            self.set('FAILED', failure='graph_or_registration_invalid', detail=str(exc))
            return EXIT['incompatible']
        fp = artifact['robot']['footprint']
        self.footprint = mv.Footprint(fp['front_m'], fp['back_m'], fp['half_width_m'])
        connections = {e['connection'] for e in artifact['edges']}
        if set(a.block)-connections:
            self.set('FAILED', failure='invalid_request', detail='Unknown connection: '+','.join(sorted(set(a.block)-connections)))
            return EXIT['invalid_request']
        try:
            resolve(artifact, a.destination)
            if a.origin != 'current':
                resolve(artifact, a.origin)
        except RoutingError as exc:
            self.set('FAILED', failure=exc.code, detail=str(exc))
            return EXIT['invalid_request']
        problem = self.wait_ready(artifact)
        if problem:
            kind, failure, detail = problem
            self.set('CANCELED' if kind == 'CANCELED' else 'FAILED', failure=failure, detail=detail)
            return EXIT[kind]
        pose = self.pose()
        if a.origin != 'current':
            dock, _ = resolve(artifact, a.origin)
            node = next(n for n in artifact['nodes'] if n['id'] == dock)
            if math.dist(pose[:2], node['map_position']) > self.settings['executor']['attach_radius_m']:
                self.set('FAILED', failure='invalid_request', detail=f'Robot is not at origin {a.origin}')
                return EXIT['invalid_request']
        # The route budget is simulation time: robot motion and waiting happen in sim time, so the
        # outcome must not depend on the host's real-time factor. A wall-clock guard still bounds a
        # stalled or extremely slow simulator.
        deadline = (self.node.get_clock().now().nanoseconds/1e9+a.timeout, time.monotonic()+a.wall_guard)
        route_id = uuid.uuid4().hex[:12]
        self.state['route_id'] = route_id
        tried = set()
        attempt = 0
        retry_same = False
        while True:
            pose = self.pose()
            try:
                route, graph = self.make_plan(artifact, pose, route_id)
            except RoutingError as exc:
                self.set('FAILED', failure=exc.code, detail=str(exc), route=None)
                return EXIT['invalid_request']
            if not route['found']:
                status = self.classify_no_route(artifact, pose)
                self.set(status, failure='no_route', detail='No validated route under current restrictions', route=route)
                return EXIT[status]
            signature = tuple(route['connections'])
            if signature in tried and not retry_same:
                self.set('BLOCKED', failure='replan_oscillation', detail='Replanning repeated a failed route', route=route)
                return EXIT['BLOCKED']
            tried.add(signature)
            retry_same = False
            self.state['remaining_m'] = route['distance_m']
            self.set('PLANNED', route=route, detail=f"{route['algorithm']} {route['distance_m']} m, "
                                                    f"{len(route['waypoints'])} objectives")
            self.set('VALIDATING', detail='Checking live map version and robot-to-graph connector')
            issue = self.connector_check(route, pose)
            if issue:
                self.set('FAILED', failure='route_validation_failed', detail=issue)
                return EXIT['FAILED']
            outcome, status, failure, detail, evidence = self.execute(artifact, route, graph, deadline)
            if outcome == 'done':
                final = self.settle()
                target = route['waypoints'][-1]['map']
                error = math.dist(final[:2], target[:2]) if final else math.inf
                yaw_error = abs(wrap(final[2]-target[2])) if final else math.inf
                self.report.update(final_position_error_m=error, final_yaw_error_rad=yaw_error)
                limits = self.settings['executor']
                if error <= limits['final_position_tolerance_m'] and yaw_error <= limits['final_yaw_tolerance_rad']:
                    self.set('COMPLETED', failure=None, detail=f'Arrived: {error:.3f} m, {yaw_error:.3f} rad')
                    return EXIT['COMPLETED']
                self.set('FAILED', failure='final_pose_error', detail=f'{error:.3f} m / {yaw_error:.3f} rad from destination')
                return EXIT['FAILED']
            if outcome == 'stop':
                self.settle()
                self.set(status, failure=failure, detail=detail)
                return EXIT['timeout'] if failure == 'timeout' else EXIT[status]
            attempt += 1
            self.report['replans'].append({'attempt': attempt, 'reason': failure, 'detail': detail, 'evidence': evidence,
                                           'robot_map_pose': self.pose()})
            if attempt > self.settings['executor']['max_replans']:
                self.settle()
                self.set('FAILED', failure='replan_limit', detail=f'More than {attempt-1} replans')
                return EXIT['FAILED']
            if not evidence:
                # A Nav2 failure or detour is not proof that a corridor is blocked (e.g. a worker
                # passing). Stop, let the obstruction clear, and retry the SAME graph route.
                wait = self.settings['executor']['retry_wait_s']
                self.set('REPLANNING', failure=failure, detail=f'{detail}; no corridor blockage evidence, '
                                                               f'retrying the same route after {wait} s')
                self.settle(wait)
                retry_same = True
                continue
            ttl = self.settings['executor']['blocked_connection_ttl_s']
            for connection in evidence:
                self.runtime_blocks.setdefault(connection, time.monotonic()+ttl)
            self.set('REPLANNING', failure=failure, detail=f"{detail}; closing {', '.join(sorted(evidence))}")
            self.settle(2.)

    def settle(self, seconds=4.):
        """Observe the original controller decelerating after Nav2 stops; never commands motion."""
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.spin()
        return self.pose()

    def finish(self, code):
        r = self.report
        r['final_status'] = self.message()
        r['exit_code'] = code
        r['command_samples'] = self.commands
        for frame in ('map', 'odom', 'world'):
            r[f'{frame}_travel_distance_m'] = path_length([t[frame] for t in r['trajectory'] if t.get(frame)])
        r['recoveries'] = max((f['recoveries'] for f in r['feedback']), default=0)
        last = r['trajectory'][-1] if r['trajectory'] else {}
        r['stopped'] = bool(last.get('velocity')) and max(abs(v) for v in last['velocity']) < .02
        self.evaluate(r)
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.args.output.write_text(json.dumps(r, indent=2)+'\n')
        summary = {k: v for k, v in r.items() if k not in ('trajectory', 'feedback', 'plan_checks', 'routes', 'status_history')}
        print(json.dumps(summary, indent=2, default=str))

    def evaluate(self, r):
        """Offline evaluation with independent ground truth: corridor conformance and registration error."""
        executed = [route for route in r['routes'] if route.get('found')]
        if not executed or not r['trajectory'] or not hasattr(self, 'transform'):
            return
        lanes = []
        for route in executed:
            for (a, b), width in zip(zip(route['path_world'], route['path_world'][1:]), route['edge_widths']):
                length = math.dist(a, b)
                lanes.append(mv.Box((a[0]+b[0])/2, (a[1]+b[1])/2, math.atan2(b[1]-a[1], b[0]-a[0]), length/2+width/2, width/2))
        inverse = self.transform.inverse()
        estimated = [t['map'] for t in r['trajectory'] if t.get('map')]
        truth = [t['world'] for t in r['trajectory'] if t.get('world')]
        r['corridor_conformance'] = {
            'estimated_inside_fraction': sum(inside_lanes(lanes, inverse.apply(*p[:2]), .3) for p in estimated)/max(1, len(estimated)),
            'ground_truth_inside_fraction': sum(inside_lanes(lanes, p[:2], .3) for p in truth)/max(1, len(truth)),
            'tolerance_m': .3, 'samples': [len(estimated), len(truth)]}
        errors = [math.dist(t['map'][:2], self.transform.apply(*t['world'][:2])) for t in r['trajectory'] if t.get('map') and t.get('world')]
        if errors:
            errors.sort()
            r['registration_evaluation'] = {'samples': len(errors), 'median_m': errors[len(errors)//2],
                                            'p90_m': errors[int(.9*(len(errors)-1))], 'max_m': errors[-1]}
        if truth and executed[-1].get('path_world') and r['final_status']['status'] == 'COMPLETED':
            goal = executed[-1]['path_world'][-1]
            r['final_ground_truth_error_m'] = math.dist(truth[-1][:2], goal)


def main():
    p = argparse.ArgumentParser(description='Execute a validated warehouse route through Nav2 (no velocity output)')
    p.add_argument('--from', dest='origin', default='current', help="Origin station/node, or 'current' (robot pose)")
    p.add_argument('--to', dest='destination', required=True)
    p.add_argument('--algorithm', choices=['astar', 'dijkstra'], default='astar')
    p.add_argument('--scenario', help='Active warehouse scenario (activates its temporary blocked zones)')
    p.add_argument('--block', action='append', default=[], metavar='CONNECTION')
    p.add_argument('--timeout', type=float, default=600., help='Route deadline, simulation seconds')
    p.add_argument('--wall-guard', type=float, default=3000., help='Wall-clock bound for a stalled simulator, seconds')
    p.add_argument('--ready-timeout', type=float, default=60.)
    p.add_argument('--cancel-after', type=float, help='Request cancellation after wall seconds (test hook)')
    p.add_argument('--action', default='/navigate_through_poses')
    p.add_argument('--candidate', type=Path, help='Candidate map package directory (evaluation before approval)')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if not 1 <= a.timeout <= 3600 or not 1 <= a.ready_timeout <= 600:
        p.error('timeouts out of range')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    executor = Executor(a)
    started = time.monotonic()

    def request_cancel(signum, _frame):
        executor.cancel_requested = f'Canceled by signal {signal.Signals(signum).name}'
    signal.signal(signal.SIGINT, request_cancel)
    signal.signal(signal.SIGTERM, request_cancel)
    if a.cancel_after is not None:
        original = executor.spin

        def spin():
            if executor.cancel_requested is None and time.monotonic()-started > a.cancel_after:
                executor.cancel_requested = f'Cancellation requested after {a.cancel_after} s'
            original()
        executor.spin = spin
    code = EXIT['FAILED']
    try:
        code = executor.run()
    except Exception as exc:
        executor.set('FAILED', failure='internal_error', detail=repr(exc))
        code = EXIT['FAILED']
    finally:
        handle = getattr(executor, 'handle', None)
        if handle is not None and executor.state['action'] not in ('SUCCEEDED', 'ABORTED', 'CANCELED'):
            executor.cancel(handle)
        executor.finish(code)
        # Keep the latched final status visible briefly for late subscribers.
        end = time.monotonic()+1.5
        while time.monotonic() < end:
            rclpy.spin_once(executor.node, timeout_sec=.05)
        executor.client.destroy()
        executor.node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
