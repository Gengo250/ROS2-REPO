"""ROS 2 node of the lite engine: fixed-step loop, /clock, odometry, TF, sensors, services.

The simulation loop runs in its own thread, paced by the wall clock (step / target RTF),
so neither the executor, the Gateway nor a browser can slow it down. The executor thread
only receives commands and serves requests; both sides share the world under one lock.
"""
import math
import threading
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from amr_lite_interfaces.msg import EntityState, Obstacle as ObstacleMsg, ObstacleArray, SimEvent, WorldState
from amr_lite_interfaces.srv import AddObstacle, RemoveObstacle
from atlas_scene.scene import Scene
from . import robot_model
from .agents import Traffic
from .world import World

POSE_COVARIANCE = [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]  # controllers.yaml
BEST_EFFORT_1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def entity(**fields):
    """EntityState with every float field coerced (scene JSON may hold integers)."""
    for key in ('x', 'y', 'z', 'yaw', 'vx', 'vy', 'wz', 'speed', 'activity_progress', 'look', 'field_length'):
        if key in fields:
            fields[key] = float(fields[key])
    return EntityState(**fields)


def stamp(seconds):
    """Time message from seconds, split in integer nanoseconds (never rounds a second away)."""
    sec, nanosec = divmod(int(round(seconds*1e9)), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def quaternion(yaw):
    return 0., 0., math.sin(yaw/2), math.cos(yaw/2)


class Schedule:
    """Fires at a fixed simulation-time rate."""

    def __init__(self, rate):
        self.period, self.next = 1./rate, 0.

    def due(self, now):
        if now+1e-9 >= self.next:
            self.next = max(self.next+self.period, now)
            return True
        return False


class LiteSim(Node):
    def __init__(self):
        super().__init__('lite_sim')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.step = p('step', 0.01)
        self.target_rtf = p('real_time_factor', 1.0)
        self.max_catch_up = p('max_catch_up_steps', 25)
        profile = p('profile', 'operations')
        scene_file = p('scene_file', '')
        noise = p('lidar_noise_stddev', 0.0)
        self.scene = Scene.load(scene_file or None)
        self.model = robot_model.load(noise_stddev=noise)
        self.world = World(self.scene, self.model, profile, seed=p('seed', 7))
        self.robot_id = next(iter(self.world.robots))
        self.session = str(uuid.uuid4())
        self.lock = threading.Lock()
        self.paused = p('start_paused', False)
        self.overruns = 0
        self.rtf = 0.
        self.step_times = []
        self.schedules = {'odom': Schedule(p('odom_rate', 50.)), 'front': Schedule(self.model.front.rate),
                          'top': Schedule(self.model.top.rate), 'imu': Schedule(p('imu_rate', 50.)),
                          'world': Schedule(p('world_state_rate', 30.)), 'truth': Schedule(p('ground_truth_rate', 20.))}
        self.clock_divider = max(1, round(1./(self.step*p('clock_rate', 100.))))
        self.pub_clock = self.create_publisher(Clock, '/clock', BEST_EFFORT_1)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_top = self.create_publisher(LaserScan, '/top_scan', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.pub_world = self.create_publisher(WorldState, '/lite_sim/world_state', BEST_EFFORT_1)
        self.pub_events = self.create_publisher(SimEvent, '/lite_sim/events', QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_obstacles = self.create_publisher(ObstacleArray, '/lite_sim/obstacles', LATCHED)
        self.pub_truth = self.create_publisher(TFMessage, '/digital_twin/simulation/poses', BEST_EFFORT_1)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_command, 10)
        self.create_service(AddObstacle, '/lite_sim/add_obstacle', self.on_add)
        self.create_service(RemoveObstacle, '/lite_sim/remove_obstacle', self.on_remove)
        self.create_service(SetBool, '/lite_sim/set_paused', self.on_pause)
        self.publish_obstacles()
        self.get_logger().info(
            f'lite_sim: scene {self.scene.geometry_sha256[:12]}, profile {profile}, {len(self.world.workers)} workers, '
            f'{len(self.world.forklifts)} forklifts, step {self.step*1000:.0f} ms, target RTF {self.target_rtf}')
        self.running = True
        self.thread = threading.Thread(target=self.loop, name='lite-sim-loop', daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ inputs
    def on_command(self, msg):
        with self.lock:
            self.world.command(self.robot_id, msg.twist.linear.x, msg.twist.angular.z)

    def on_add(self, request, response):
        with self.lock:
            obstacle, reason = self.world.add_obstacle(request.kind, request.x, request.y, request.yaw)
        response.accepted, response.reason = obstacle is not None, reason
        if obstacle:
            response.obstacle = self.obstacle_msg(obstacle)
            self.publish_obstacles()
        return response

    def on_remove(self, request, response):
        with self.lock:
            removed = self.world.remove_obstacle(request.id)
        response.removed = removed
        response.accepted = removed > 0 or not request.id
        response.reason = '' if response.accepted else f'no obstacle {request.id}'
        if removed:
            self.publish_obstacles()
        return response

    def on_pause(self, request, response):
        with self.lock:
            if self.paused != request.data:
                self.world.emit('paused' if request.data else 'resumed', 'simulation')
            self.paused = request.data
        response.success, response.message = True, 'paused' if request.data else 'running'
        return response

    # ------------------------------------------------------------------ loop
    def loop(self):
        period = self.step/self.target_rtf
        deadline = time.perf_counter()
        window = (time.perf_counter(), self.world.time)
        last_diag = 0.
        count = 0
        while self.running and rclpy.ok():
            wait = deadline-time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            started = time.perf_counter()
            if self.paused:
                with self.lock:
                    self.publish_events()
                if count % 10 == 0 and rclpy.ok():
                    self.pub_clock.publish(Clock(clock=stamp(self.world.time)))
                deadline += period
                count += 1
                window = (time.perf_counter(), self.world.time)
                continue
            with self.lock:
                self.world.step(self.step)
                try:
                    self.publish(count)
                except Exception:
                    if not rclpy.ok() or not self.running:
                        break  # context shut down by a signal: end the loop quietly
                    raise
            count += 1
            self.step_times.append(time.perf_counter()-started)
            deadline += period
            behind = time.perf_counter()-deadline
            if behind > self.max_catch_up*period:
                # Never spiral: drop the backlog and report it as overruns (RTF falls).
                self.overruns += int(behind/period)
                deadline = time.perf_counter()
            now = time.perf_counter()
            if now-window[0] >= 1.:
                self.rtf = (self.world.time-window[1])/(now-window[0])
                window = (now, self.world.time)
            if now-last_diag >= 1. and rclpy.ok():
                last_diag = now
                self.publish_diagnostics()

    # ------------------------------------------------------------------ outputs
    def publish(self, count):
        w, t = self.world, self.world.time
        when = stamp(t)
        if count % self.clock_divider == 0:
            self.pub_clock.publish(Clock(clock=when))
        robot = w.robots[self.robot_id]
        d = robot.drive
        if self.schedules['odom'].due(t):
            ox, oy, oyaw = w.odometry(self.robot_id)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = when, 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = ox, oy
            q = quaternion(oyaw)
            o = odom.pose.pose.orientation
            o.x, o.y, o.z, o.w = q
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = d.v, d.w
            for i, value in enumerate(POSE_COVARIANCE):
                odom.pose.covariance[7*i] = value
                odom.twist.covariance[7*i] = value
            self.pub_odom.publish(odom)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = when, 'odom', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = ox, oy
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = q
            self.pub_tf.publish(TFMessage(transforms=[tf]))
            left, right = w.drive.wheel_rates(d.v, d.w)
            joints = JointState()
            joints.header.stamp = when
            joints.name = ['left_wheel_joint', 'right_wheel_joint']
            joints.position = [d.left_angle, d.right_angle]
            joints.velocity = [left, right]
            self.pub_joints.publish(joints)
        for key, spec, pub in (('front', self.model.front, self.pub_scan), ('top', self.model.top, self.pub_top)):
            if self.schedules[key].due(t):
                ranges = w.scan(self.robot_id, spec)
                msg = LaserScan()
                msg.header.stamp, msg.header.frame_id = when, spec.frame_id
                msg.angle_min, msg.angle_max, msg.angle_increment = spec.angle_min, spec.angle_max, spec.increment
                msg.time_increment, msg.scan_time = 0., 1./spec.rate
                msg.range_min, msg.range_max = spec.range_min, spec.range_max
                msg.ranges = ranges.tolist()
                pub.publish(msg)
        if self.schedules['imu'].due(t):
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = when, 'imu_link'
            o = imu.orientation
            o.x, o.y, o.z, o.w = quaternion(d.yaw)
            imu.angular_velocity.z = d.w
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = d.accel, d.v*d.w, 9.81
            imu.orientation_covariance[0] = imu.orientation_covariance[4] = imu.orientation_covariance[8] = 1e-4
            imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = imu.angular_velocity_covariance[8] = 1e-4
            imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] = imu.linear_acceleration_covariance[8] = 1e-3
            self.pub_imu.publish(imu)
        if self.schedules['world'].due(t):
            self.pub_world.publish(self.world_state(when))
        if self.schedules['truth'].due(t):
            self.pub_truth.publish(self.ground_truth(when))
        self.publish_events()

    def entities(self):
        return [entity(**fields) for fields in self.entity_fields()]

    def entity_fields(self):
        w = self.world
        for r in w.robots.values():
            d = r.drive
            yield dict(id=r.id, type='amr', x=d.x, y=d.y, z=0., yaw=d.yaw, vx=d.v*math.cos(d.yaw),
                              vy=d.v*math.sin(d.yaw), wz=d.w, speed=d.v, state=r.state, field_state=r.field_state,
                              field_length=r.field_length, reason=r.reason)
        for k in w.workers:
            activity, progress = k.activity()
            yield dict(id=k.id, type='human', x=k.x, y=k.y, z=k.level_z, yaw=k.yaw, vx=k.v*math.cos(k.yaw),
                              vy=k.v*math.sin(k.yaw), wz=getattr(k, 'yaw_rate', 0.), speed=k.v, state=k.state(),
                              activity=activity, activity_progress=float(progress), carrying=k.carry,
                              look=float(k.look), reason=k.reason)
        for f in w.forklifts:
            state, reason = Traffic.state(f)
            yaw = f.pose[2]
            yield dict(id=f.id, type='forklift', x=f.pose[0], y=f.pose[1], z=0., yaw=yaw,
                              vx=f.v*math.cos(yaw), vy=f.v*math.sin(yaw), speed=f.v, state=state,
                              field_state=f.field, field_length=float(f.prot), reason=reason)

    def world_state(self, when):
        msg = WorldState()
        msg.header.stamp, msg.header.frame_id = when, 'world'
        msg.session_id, msg.epoch, msg.sim_time = self.session, 0, self.world.time
        msg.real_time_factor, msg.target_real_time_factor = float(self.rtf), float(self.target_rtf)
        msg.paused, msg.overruns, msg.scene_sha256 = self.paused, self.overruns, self.scene.scene_sha256
        msg.entities = list(self.entities())
        msg.obstacles = [self.obstacle_msg(o) for o in self.world.obstacles.values()]
        return msg

    def ground_truth(self, when):
        """Same format as the Gazebo adapter: world → entity, one stamp, never on /tf."""
        out = TFMessage()
        for e in self.entities():
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = when, 'world', e.id
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = e.x, e.y, e.z
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = quaternion(e.yaw)
            out.transforms.append(tf)
        return out

    @staticmethod
    def obstacle_msg(o):
        return ObstacleMsg(id=o.id, kind=o.kind, origin=o.origin, x=o.x, y=o.y, yaw=o.yaw,
                           length=o.length, width=o.width, height=o.height)

    def publish_obstacles(self):
        with self.lock:
            msg = ObstacleArray(obstacles=[self.obstacle_msg(o) for o in self.world.obstacles.values()])
            msg.header.stamp, msg.header.frame_id = stamp(self.world.time), 'world'
        self.pub_obstacles.publish(msg)

    def publish_events(self):
        for e in self.world.drain_events():
            msg = SimEvent(seq=e['seq'], kind=e['kind'], entity=e['entity'], other=e['other'],
                           detail=e['detail'], x=float(e['x']), y=float(e['y']))
            msg.header.stamp, msg.header.frame_id = stamp(e['time']), 'world'
            self.pub_events.publish(msg)

    def publish_diagnostics(self):
        times, self.step_times = self.step_times, []
        mean = 1000*sum(times)/len(times) if times else 0.
        worst = 1000*max(times) if times else 0.
        healthy = self.paused or self.rtf >= .95*self.target_rtf
        status = DiagnosticStatus(name='lite_sim/loop', hardware_id='lite_sim',
                                  level=DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN,
                                  message='paused' if self.paused else f'RTF {self.rtf:.3f}',
                                  values=[KeyValue(key='real_time_factor', value=f'{self.rtf:.4f}'),
                                          KeyValue(key='target_real_time_factor', value=f'{self.target_rtf:.3f}'),
                                          KeyValue(key='overruns', value=str(self.overruns)),
                                          KeyValue(key='step_cpu_ms_mean', value=f'{mean:.3f}'),
                                          KeyValue(key='step_cpu_ms_max', value=f'{worst:.3f}'),
                                          KeyValue(key='sim_time', value=f'{self.world.time:.3f}'),
                                          KeyValue(key='scene_sha256', value=self.scene.scene_sha256)])
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = stamp(self.world.time)
        self.pub_diag.publish(msg)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=2)
        return super().destroy_node()


def main():
    rclpy.init()
    node = LiteSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
