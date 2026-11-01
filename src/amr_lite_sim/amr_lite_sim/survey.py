"""SLAM coverage survey driver for mapping; NOT autonomous navigation.

Follows a fixed waypoint plan with rotate-then-drive legs on wheel odometry (no ground
truth, no map). Stops while the front scanner sees something ahead and refuses a rotation
while the 360° scanner shows anything inside the rotation clearance. Requires exclusive
/cmd_vel ownership (mapping mode: no Nav2, no guard). Writes a JSON report at the end.
"""
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from atlas_scene.scene import Scene
from .geometry import wrap


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Survey(Node):
    def __init__(self):
        super().__init__('lite_survey', parameter_overrides=[Parameter('use_sim_time', value=True)])
        p = lambda name, default: self.declare_parameter(name, default).value
        plan = json.loads(Path(p('plan', '')).read_text())
        scene = Scene.load(p('scene_file', '') or None)
        spawn = next(s for s in scene.section('spawns') if s['id'] == plan['start'])['pose']
        sx, sy, syaw = spawn[0], spawn[1], spawn[5]
        c, s = math.cos(-syaw), math.sin(-syaw)
        # World plan → odometry frame (odom starts at the spawn pose).
        self.goals = [(c*(x-sx)-s*(y-sy), s*(x-sx)+c*(y-sy)) for x, y in plan['waypoints']]
        self.speed = p('linear_speed', float(plan.get('linear_speed', .6)))
        self.turn_rate = p('angular_speed', .6)
        self.stop_distance = p('stop_distance', .6)
        self.rotation_clearance = p('rotation_clearance', .66)
        self.report_path = p('report', '')
        self.top_mount = (p('top_scanner_x', -.38), 0.)
        self.pose = None
        self.front_blocked = self.rotation_blocked = True
        self.index, self.mode = 1, 'rotate'
        self.distance = self.last_xy = None
        self.stops, self.refusals, self.started = 0, 0, None
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/top_scan', self.on_top, qos_profile_sensor_data)
        self.create_timer(.05, self.tick)
        self.done = False

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def on_scan(self, msg):
        blocked = False
        for i, r in enumerate(msg.ranges):
            a = msg.angle_min+i*msg.angle_increment
            if abs(a) < .45 and math.isfinite(r) and r < self.stop_distance+.05:
                blocked = True
                break
        self.front_blocked = blocked

    def on_top(self, msg):
        mx, my = self.top_mount
        blocked = False
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            a = msg.angle_min+i*msg.angle_increment
            if math.hypot(mx+r*math.cos(a), my+r*math.sin(a)) < self.rotation_clearance:
                blocked = True
                break
        self.rotation_blocked = blocked

    def command(self, v, w):
        msg = TwistStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = v, w
        self.pub.publish(msg)

    def tick(self):
        if self.done or self.pose is None:
            return
        if self.started is None:
            self.started, self.distance, self.last_xy = self.get_clock().now(), 0., self.pose[:2]
        x, y, yaw = self.pose
        self.distance += math.dist((x, y), self.last_xy)
        self.last_xy = (x, y)
        if self.index >= len(self.goals):
            return self.finish()
        gx, gy = self.goals[self.index]
        dx, dy = gx-x, gy-y
        dist = math.hypot(dx, dy)
        heading = wrap(math.atan2(dy, dx)-yaw)
        if self.mode == 'rotate':
            if abs(heading) < .02 or dist < .05:
                self.mode = 'drive'
            elif self.rotation_blocked:
                self.refusals += 1
                return self.command(0., 0.)
            else:
                return self.command(0., clamp(1.5*heading, self.turn_rate))
        if dist < .08:
            self.index, self.mode = self.index+1, 'rotate'
            self.get_logger().info(f'waypoint {self.index-1}/{len(self.goals)-1} reached, {self.distance:.1f} m')
            return self.command(0., 0.)
        if self.front_blocked:
            self.stops += 1
            return self.command(0., 0.)
        # Pure heading correction on a straight leg; lateral drift is tiny on exact odometry.
        v = min(self.speed, .8*dist+.05)*max(0., math.cos(heading))
        self.command(v, clamp(2.*heading, .4))

    def finish(self):
        self.command(0., 0.)
        self.done = True
        elapsed = (self.get_clock().now()-self.started).nanoseconds/1e9
        report = {'waypoints': len(self.goals), 'distance_m': round(self.distance, 2), 'sim_seconds': round(elapsed, 2),
                  'front_stop_ticks': self.stops, 'rotation_refusal_ticks': self.refusals}
        self.get_logger().info('survey complete: '+json.dumps(report))
        if self.report_path:
            Path(self.report_path).write_text(json.dumps(report, indent=1))


def main():
    rclpy.init()
    node = Survey()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
