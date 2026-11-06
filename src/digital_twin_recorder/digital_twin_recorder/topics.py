"""Recording inventory: which ROS topics a profile records, how, and at what rate.

Names, types and publisher QoS were verified on the running lite stack with
``scripts/inspect_topics.py`` (see PHASE_2_STORAGE.md, "Recording inventory"). Subscriptions
use the most permissive QoS compatible with those publishers: best effort matches both
reliable and best-effort publishers; latched topics are read transient-local so the
recorder receives the current value even when it starts after the publisher.

``required`` topics must have at least one sample for the recording to complete; optional
ones may be absent in a simulation profile (e.g. no Nav2 running) and are then reported
with zero samples. Rates are upper bounds applied on the source (simulation) timestamp:
0 means every message.
"""
from dataclasses import dataclass

CORE, EXTENDED = 'core', 'extended'
PROFILES = (CORE, EXTENDED)


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    ros_type: str
    kind: str            # converter / ontology key (models.records.CONVERTERS)
    qos: str             # sensor | reliable | latched
    max_rate_hz: float   # 0: every message
    required: bool
    stamp: str = 'header'  # header | reception (message has no header: simulation-clock reception time)


def inventory(config):
    """Topic specs for ``config`` (profile, backend, topic names and rates)."""
    lite = config.simulation_backend == 'lite'
    specs = [
        TopicSpec(config.odom_topic, 'nav_msgs/msg/Odometry', 'odometry', 'sensor', config.odom_hz, True),
        TopicSpec(config.imu_topic, 'sensor_msgs/msg/Imu', 'imu', 'sensor', config.imu_hz, True),
    ]
    if lite:
        specs += [
            TopicSpec('/lite_sim/world_state', 'amr_lite_interfaces/msg/WorldState', 'world_state', 'sensor',
                      config.world_state_hz, True),
            TopicSpec('/lite_sim/events', 'amr_lite_interfaces/msg/SimEvent', 'sim_event', 'reliable', 0., False),
            TopicSpec('/lite_sim/obstacles', 'amr_lite_interfaces/msg/ObstacleArray', 'obstacles', 'latched', 0., True),
        ]
    specs += [
        TopicSpec('/navigate_to_pose/_action/status', 'action_msgs/msg/GoalStatusArray', 'goal_status', 'latched', 0.,
                  False, stamp='reception'),
        TopicSpec('/navigation/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', 'diagnostics', 'reliable', 0., False),
    ]
    if config.profile == EXTENDED:
        specs += [
            TopicSpec('/plan', 'nav_msgs/msg/Path', 'path', 'reliable', config.plan_hz, False),
            # Only the configured frame pairs (default map -> odom from AMCL); odom -> base_link
            # would duplicate the odometry topic.
            TopicSpec('/tf', 'tf2_msgs/msg/TFMessage', 'tf', 'reliable', 0., False),
            TopicSpec('/tf_static', 'tf2_msgs/msg/TFMessage', 'tf_static', 'latched', 0., False, stamp='reception'),
            TopicSpec('/scan', 'sensor_msgs/msg/LaserScan', 'scan', 'sensor', config.scan_hz, True),
            TopicSpec('/top_scan', 'sensor_msgs/msg/LaserScan', 'scan', 'sensor', config.scan_hz, True),
        ]
    return specs
