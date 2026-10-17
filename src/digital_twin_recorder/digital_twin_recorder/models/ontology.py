"""Lossless source representation using the documented Mosaico extension API.

No quaternion normalization or covariance-sentinel reinterpretation. Both timestamp
clocks are explicit; Message.timestamp_ns is reception wall time, never global pose.
"""
from mosaicolabs import Serializable, MosaicoType as T, MosaicoField as F
from pydantic import ConfigDict, BaseModel

class RosTelemetryV1(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    robot_id: T.string = F()
    source_topic: T.string = F()
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

class RecordingSummaryV1(Serializable):
    # JSON is metadata only; telemetry uses typed Arrow fields above.
    summary_json: T.string = F()
