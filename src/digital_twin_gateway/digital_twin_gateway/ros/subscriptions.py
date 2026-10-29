"""QoS-compatible telemetry subscriptions; TF/joints provide receipt diagnostics."""
from functools import partial
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from pydantic import ValidationError
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from .state_collector import normalize_odometry, stamp_seconds, normalize_simulation


def subscribe(node, manager, settings):
    live = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def odometry(config, message):
        try:
            robot = normalize_odometry(config.id, config.odom_topic, message, manager.wall_clock())
        except ValidationError:
            node.get_logger().warning('Invalid odometry rejected', throttle_duration_sec=5)
            return
        manager.update_robot(robot)

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def route(message):
        try:
            manager.update_route(message.data)
        except (ValidationError, ValueError):
            node.get_logger().warning('Invalid route status rejected', throttle_duration_sec=5)

    def simulation(message):
        try:
            entities = normalize_simulation(settings, message, manager.wall_clock())
            manager.update_entities(entities)
        except (ValidationError, ValueError):
            node.get_logger().warning('Invalid simulation snapshot rejected', throttle_duration_sec=5)

    subscriptions = [
        node.create_subscription(DiagnosticArray, '/navigation/diagnostics', manager.update_navigation, live),
        # Latched by the route executor so a late Gateway still sees the last route.
        node.create_subscription(String, settings.route_status_topic, route, latched),
        node.create_subscription(TFMessage, settings.simulation_pose_topic, simulation, live),
        node.create_subscription(Clock, settings.clock_topic,
                                 lambda msg: manager.update_clock(stamp_seconds(msg.clock)), live),
        node.create_subscription(TFMessage, settings.tf_topic,
                                 lambda _: manager.touch(settings.tf_topic), live),
        node.create_subscription(TFMessage, settings.tf_static_topic,
                                 lambda _: manager.touch(settings.tf_static_topic), static),
    ]
    for config in settings.robots:
        subscriptions.append(node.create_subscription(Odometry, config.odom_topic,
                                                       partial(odometry, config), live))
    for topic in {r.joint_states_topic for r in settings.robots}:
        subscriptions.append(node.create_subscription(JointState, topic,
                             lambda _, t=topic: manager.touch(t), live))
    return subscriptions
