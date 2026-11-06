"""Lossless source representation using the documented Mosaico extension API.

No quaternion normalization or covariance-sentinel reinterpretation. Both timestamp
clocks are explicit; Message.timestamp_ns is reception wall time, never global pose.
Every record keeps the ROS frame it was published in: odometry stays in ``odom``,
lite-engine state in ``world`` and Nav2 plans in ``map``; nothing is re-expressed.

Schema evolution: a tag is never changed once data exists. A new field or meaning
gets a new class with a ``_v2`` tag; readers select the class by the stored tag.
"""
from mosaicolabs import Serializable, MosaicoType as T, MosaicoField as F
from pydantic import ConfigDict, BaseModel


class RosTelemetryV1(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    robot_id: T.string = F()
    source_topic: T.string = F()
    # ROS header stamp (simulation time); for header-less messages the recorder's
    # simulation-clock reception time (see topics.py, ``stamp`` column of the inventory).
    source_timestamp_ns: T.int64 = F()
    received_wall_ns: T.int64 = F()
    frame_id: T.string = F()


class RosOdometryV1(Serializable, RosTelemetryV1):
    __ontology_tag__ = 'digital_twin_ros_odometry_v1'
    child_frame_id: T.string = F()
    position: T.list_(T.float64, 3) = F(min_length=3, max_length=3)
    orientation: T.list_(T.float64, 4) = F(min_length=4, max_length=4)
    linear_velocity: T.list_(T.float64, 3) = F(min_length=3, max_length=3)
    angular_velocity: T.list_(T.float64, 3) = F(min_length=3, max_length=3)
    pose_covariance: T.list_(T.float64, 36) = F(min_length=36, max_length=36)
    twist_covariance: T.list_(T.float64, 36) = F(min_length=36, max_length=36)


class RosImuV1(Serializable, RosTelemetryV1):
    __ontology_tag__ = 'digital_twin_ros_imu_v1'
    orientation: T.list_(T.float64, 4) = F(min_length=4, max_length=4)
    angular_velocity: T.list_(T.float64, 3) = F(min_length=3, max_length=3)
    linear_acceleration: T.list_(T.float64, 3) = F(min_length=3, max_length=3)
    orientation_covariance: T.list_(T.float64, 9) = F(min_length=9, max_length=9)
    angular_velocity_covariance: T.list_(T.float64, 9) = F(min_length=9, max_length=9)
    linear_acceleration_covariance: T.list_(T.float64, 9) = F(min_length=9, max_length=9)


# ---------------------------------------------------------------- lite engine (amr_lite_interfaces)
class LiteEntityV1(Serializable):
    """amr_lite_interfaces/msg/EntityState, world frame (x east, y north, z up)."""
    __ontology_tag__ = 'digital_twin_lite_entity_v1'
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    id: T.string = F()
    type: T.string = F()
    x: T.float64 = F()
    y: T.float64 = F()
    z: T.float64 = F()
    yaw: T.float64 = F()
    vx: T.float64 = F()
    vy: T.float64 = F()
    wz: T.float64 = F()
    speed: T.float64 = F()
    state: T.string = F()
    activity: T.string = F()
    activity_progress: T.float64 = F()
    carrying: T.bool = F()
    look: T.float64 = F()
    field_state: T.uint8 = F()
    field_length: T.float64 = F()
    reason: T.string = F()


class LiteObstacleV1(Serializable):
    """amr_lite_interfaces/msg/Obstacle, world frame."""
    __ontology_tag__ = 'digital_twin_lite_obstacle_v1'
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    id: T.string = F()
    kind: T.string = F()
    origin: T.string = F()
    x: T.float64 = F()
    y: T.float64 = F()
    yaw: T.float64 = F()
    length: T.float64 = F()
    width: T.float64 = F()
    height: T.float64 = F()


class LiteWorldStateV1(Serializable, RosTelemetryV1):
    """amr_lite_interfaces/msg/WorldState: every AMR, worker and forklift of one engine step."""
    __ontology_tag__ = 'digital_twin_lite_world_state_v1'
    session_id: T.string = F()
    epoch: T.uint32 = F()
    sim_time: T.float64 = F()
    real_time_factor: T.float64 = F()
    target_real_time_factor: T.float64 = F()
    paused: T.bool = F()
    overruns: T.uint64 = F()
    scene_sha256: T.string = F()
    entities: T.list_(LiteEntityV1) = F()
    obstacles: T.list_(LiteObstacleV1) = F()


class LiteObstacleArrayV1(Serializable, RosTelemetryV1):
    """amr_lite_interfaces/msg/ObstacleArray (latched; one record per change)."""
    __ontology_tag__ = 'digital_twin_lite_obstacles_v1'
    obstacles: T.list_(LiteObstacleV1) = F()


class LiteSimEventV1(Serializable, RosTelemetryV1):
    """amr_lite_interfaces/msg/SimEvent: contacts, safety stops, obstacle and pause changes."""
    __ontology_tag__ = 'digital_twin_lite_event_v1'
    seq: T.uint64 = F()
    kind: T.string = F()
    entity: T.string = F()
    other: T.string = F()
    detail: T.string = F()
    x: T.float64 = F()
    y: T.float64 = F()


# ---------------------------------------------------------------- navigation
class GoalStatusV1(Serializable):
    __ontology_tag__ = 'digital_twin_goal_status_v1'
    model_config = ConfigDict(extra='forbid')
    goal_id: T.string = F()          # 16-byte UUID as hex
    status: T.int8 = F()             # action_msgs/msg/GoalStatus constants
    accepted_ns: T.int64 = F()


class GoalStatusArrayV1(Serializable, RosTelemetryV1):
    """action_msgs/msg/GoalStatusArray of a Nav2 action server (mission outcomes)."""
    __ontology_tag__ = 'digital_twin_goal_status_array_v1'
    statuses: T.list_(GoalStatusV1) = F()


class KeyValueV1(Serializable):
    __ontology_tag__ = 'digital_twin_key_value_v1'
    model_config = ConfigDict(extra='forbid')
    key: T.string = F()
    value: T.string = F()


class DiagnosticStatusV1(Serializable):
    __ontology_tag__ = 'digital_twin_diagnostic_status_v1'
    model_config = ConfigDict(extra='forbid')
    level: T.uint8 = F()
    name: T.string = F()
    message: T.string = F()
    hardware_id: T.string = F()
    values: T.list_(KeyValueV1) = F()


class DiagnosticArrayV1(Serializable, RosTelemetryV1):
    """diagnostic_msgs/msg/DiagnosticArray (navigation interlock state)."""
    __ontology_tag__ = 'digital_twin_diagnostic_array_v1'
    statuses: T.list_(DiagnosticStatusV1) = F()


class PathPoseV1(Serializable):
    __ontology_tag__ = 'digital_twin_path_pose_v1'
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    stamp_ns: T.int64 = F()
    frame_id: T.string = F()
    position: T.list_(T.float64, 3) = F(min_length=3, max_length=3)
    orientation: T.list_(T.float64, 4) = F(min_length=4, max_length=4)


class NavPathV1(Serializable, RosTelemetryV1):
    """nav_msgs/msg/Path; ``frame_id`` is the plan frame (map), never converted to world."""
    __ontology_tag__ = 'digital_twin_nav_path_v1'
    poses: T.list_(PathPoseV1) = F()


class TransformV1(Serializable):
    __ontology_tag__ = 'digital_twin_transform_v1'
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    stamp_ns: T.int64 = F()
    parent_frame_id: T.string = F()
    child_frame_id: T.string = F()
    translation: T.list_(T.float64, 3) = F(min_length=3, max_length=3)
    rotation: T.list_(T.float64, 4) = F(min_length=4, max_length=4)


class TfMessageV1(Serializable, RosTelemetryV1):
    """tf2_msgs/msg/TFMessage restricted to the configured frame pairs (frame_id empty)."""
    __ontology_tag__ = 'digital_twin_tf_message_v1'
    transforms: T.list_(TransformV1) = F()


# ---------------------------------------------------------------- range sensors
class LaserScanV1(Serializable, RosTelemetryV1):
    """sensor_msgs/msg/LaserScan. REP 117: +inf (no return), -inf and NaN are kept as sent."""
    __ontology_tag__ = 'digital_twin_laser_scan_v1'
    model_config = ConfigDict(extra='forbid', allow_inf_nan=True)
    angle_min: T.float64 = F()
    angle_max: T.float64 = F()
    angle_increment: T.float64 = F()
    time_increment: T.float64 = F()
    scan_time: T.float64 = F()
    range_min: T.float64 = F()
    range_max: T.float64 = F()
    ranges: T.list_(T.float32) = F()
    intensities: T.list_(T.float32) = F()


class RecordingSummaryV1(Serializable):
    # JSON is metadata only; telemetry uses typed Arrow fields above.
    summary_json: T.string = F()


ONTOLOGIES = {cls.__ontology_tag__: cls for cls in (
    RosOdometryV1, RosImuV1, LiteWorldStateV1, LiteObstacleArrayV1, LiteSimEventV1, GoalStatusArrayV1,
    DiagnosticArrayV1, NavPathV1, TfMessageV1, LaserScanV1)}
