"""Fleet tracking for the navigation stacks: where the *other* AMRs are, as costmap input.

The scanners cannot see another AMR reliably: the front scanner covers ±90 degrees and the elevated
360-degree scanner (0.48 m) passes over the other robots' bodies (sensed up to 0.455 m). An AMR that
turns in place next to another one therefore turns into it (reproduced: contacts at a shared goal).

This node publishes, for every navigated robot, the footprints of all *other* AMRs as seen from
that robot's base_link, computed from the backend-authoritative engine state
(/lite_sim/world_state). It is explicit shared tracking data, on its own topic
(``<ns>/fleet/robots``) and consumed by its own costmap layer (``fleet_layer``); no scanner topic
is touched. The message type is LaserScan only because Nav2's obstacle layer marks *and clears* a
range image: every beam without a robot clears its own layer up to the range limit, so a robot that
drives away never stays behind as a ghost obstacle, and the map is never modified.

Conventions: one range per 0.5 degree, 360 degrees, origin at the observer's base_link; each other
robot is its collision box (amr_dimensions.json, as the engine) inflated by ``margin``; the
observer itself is never included; data is only published while engine states arrive (stamped
with their simulation time), so stale fleet data simply stops and the costmap reports it.
"""
import array
import math
import numpy as np
from .geometry import oriented_box

BEAMS = 720


def beam_angles(beams=BEAMS):
    return -math.pi+np.arange(beams)*(2*math.pi/beams)


def fleet_ranges(observer, others, length, width, offset, margin=.10, range_max=30., beams=BEAMS):
    """Ranges from ``observer`` (x, y, yaw; base_link) to the boxes of ``others`` [(x, y, yaw)].

    Beam i points at angle -pi + i*2pi/beams in the observer's frame; inf where no robot is hit
    within range_max (the obstacle layer then clears that beam)."""
    ox, oy, oyaw = observer
    angles = beam_angles(beams)+oyaw
    dx, dy = np.cos(angles), np.sin(angles)
    ranges = np.full(beams, np.inf)
    reach = range_max+math.hypot(length, width)
    for x, y, yaw in others:
        if math.hypot(x-ox, y-oy) > reach:
            continue
        box = oriented_box(x, y, yaw, length+2*margin, width+2*margin, offset)
        for i in range(4):
            ax, ay = box[i]
            bx, by = box[(i+1) % 4]
            ex, ey = bx-ax, by-ay
            # observer + t·d = a + u·e  →  solve for t (range) and u (position on the edge).
            denom = dx*ey-dy*ex
            with np.errstate(divide='ignore', invalid='ignore'):
                t = ((ax-ox)*ey-(ay-oy)*ex)/denom
                u = ((ax-ox)*dy-(ay-oy)*dx)/denom
            hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
            ranges = np.where(hit & (t < ranges), t, ranges)
    ranges[ranges > range_max] = np.inf
    return ranges


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from amr_lite_interfaces.msg import WorldState
    from . import fleet, robot_model

    class FleetTracker(Node):
        def __init__(self):
            super().__init__('fleet_tracker')
            p = lambda name, default: self.declare_parameter(name, default).value
            self.root = p('root_robot', fleet.ROOT_ROBOT)
            self.robots = fleet.parse_robots(p('robots', 'all'))
            self.period = 1./p('rate', 10.)
            self.margin, self.range_max = p('margin', .10), p('range_max', 30.)
            model = robot_model.load()
            self.geometry = (model.length, model.width, model.footprint_offset)
            self.publishers_, self.last, self.count = {}, -math.inf, 0
            # The engine publishes its state at 30 Hz of simulation time: only every ``decimate``-th
            # message is deserialized (deserializing every full state in Python was ~10 % of a core).
            self.decimate = max(1, round(p('world_state_rate', 30.)*self.period))
            self.create_subscription(WorldState, '/lite_sim/world_state', self.on_raw,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)
            self.get_logger().info(f'fleet tracking for {self.robots or "every AMR"}: other AMRs\' footprints '
                                   f'(+{self.margin:.2f} m) to <ns>/fleet/robots at {1/self.period:.0f} Hz (sim time)')

        def publisher(self, robot):
            if robot not in self.publishers_:
                self.publishers_[robot] = self.create_publisher(LaserScan, fleet.topic(robot, 'fleet/robots', self.root), 10)
            return self.publishers_[robot]

        def on_raw(self, data):
            self.count += 1
            if self.count % self.decimate == 0:
                self.on_world(deserialize_message(data, WorldState))

        def on_world(self, msg):
            if msg.paused or msg.sim_time < self.last+self.period*.5:
                return
            self.last = msg.sim_time
            amrs = {e.id: (e.x, e.y, e.yaw) for e in msg.entities if e.type == 'amr'}
            for robot, pose in amrs.items():
                if self.robots is not None and robot not in self.robots:
                    continue
                others = [p for r, p in amrs.items() if r != robot]
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = msg.header.stamp, fleet.frame_prefix(robot, self.root)+'base_link'
                scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/BEAMS
                scan.angle_max = scan.angle_min+(BEAMS-1)*scan.angle_increment
                scan.scan_time, scan.range_min, scan.range_max = self.period, 0.05, self.range_max
                scan.ranges = array.array('f', fleet_ranges(pose, others, *self.geometry, self.margin, self.range_max).astype(np.float32).tobytes())
                self.publisher(robot).publish(scan)

    rclpy.init()
    node = FleetTracker()
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
