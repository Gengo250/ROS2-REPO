"""Wire models: JSON Schema and browser types are generated from these models."""
from typing import Annotated, Literal
from pydantic import AwareDatetime, BaseModel, ConfigDict, Field

Availability = Literal['unknown', 'offline', 'stale', 'online']
Number = Annotated[float, Field(allow_inf_nan=False)]
Scenario = Literal['normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked']


class Model(BaseModel):
    model_config = ConfigDict(extra='forbid', frozen=True)


class Vector3(Model):
    x: Number
    y: Number
    z: Number


class Quaternion(Vector3):
    w: Number


class Pose(Model):
    position: Vector3
    orientation: Quaternion


class Velocity(Model):
    linear: Vector3
    angular: Vector3


class Robot(Model):
    id: str
    frame_id: str | None
    child_frame_id: str | None
    pose_source: str
    pose: Pose | None
    velocity: Velocity | None
    status: Availability
    source_timestamp: Number | None
    received_at: AwareDatetime | None
    age_seconds: Number | None


class World(Model):
    id: str
    frame_id: str
    scenario: Scenario | None
    scenario_source: Literal['launch_configuration', 'environment', 'unspecified']
    status: Literal['unknown', 'running', 'unavailable']
    availability: Availability
    status_source: Literal['clock_progress'] = 'clock_progress'


class TopicStatus(Model):
    topic: str
    status: Availability
    received_at: AwareDatetime | None
    age_seconds: Number | None
    messages_received: int = Field(ge=0)


class ConnectionStatus(Model):
    protocol_version: Literal['1.0'] = '1.0'
    message_type: Literal['connection_status'] = 'connection_status'
    session_id: str
    gateway: Literal['online', 'offline']
    ros_initialized: bool
    ros: Availability
    simulation: Availability
    last_odometry_timestamp: AwareDatetime | None
    active_websocket_clients: int = Field(ge=0)
    configured_publication_hz: Number
    measured_publication_hz: Number
    topics: tuple[TopicStatus, ...]


class TwinState(Model):
    protocol_version: Literal['1.0'] = '1.0'
    message_type: Literal['twin_state'] = 'twin_state'
    session_id: str
    sequence: int = Field(ge=1)
    timestamp: AwareDatetime
    simulation_time: Number | None
    world: World
    robots: tuple[Robot, ...]
    # Reserved collections MUST remain empty in v1.0. A future protocol revision
    # will define their element types; empty does not mean the world is empty.
    humans: tuple[()] = ()
    obstacles: tuple[()] = ()
    stations: tuple[()] = ()
    connection: ConnectionStatus


class EntityTransform(Model):
    id: str
    frame_id: str
    pose: Pose


class DynamicEntity(Model):
    id: str
    type: Literal['robot', 'human', 'obstacle']
    gazebo_name: str
    frame_id: str
    pose: Pose | None
    # SceneBroadcaster supplies poses, not measured velocity.
    velocity: Velocity | None = None
    source_timestamp: Number | None
    received_at: AwareDatetime | None
    age_seconds: Number | None
    status: Availability
    pose_source: str
    components: tuple[EntityTransform, ...] = Field(default=(), max_length=64)


class SimulationState(Model):
    epoch: int = Field(ge=0)
    status: Literal['unknown', 'running', 'paused_or_stalled', 'offline']
    pose_topic: str


class TwinStateV2(Model):
    protocol_version: Literal['2.0'] = '2.0'
    message_type: Literal['twin_state'] = 'twin_state'
    session_id: str
    sequence: int = Field(ge=1)
    timestamp: AwareDatetime
    simulation_time: Number | None
    simulation: SimulationState
    world: World
    # Same odometry diagnostics as v1. World ground truth is explicitly separate.
    robots: tuple[Robot, ...]
    simulation_robots: tuple[DynamicEntity, ...]
    humans: tuple[DynamicEntity, ...]
    obstacles: tuple[DynamicEntity, ...]
    stations: tuple[()] = ()
    connection: ConnectionStatus


class NavigationDiagnostic(Model):
    name: Literal['navigation/interlock', 'navigation/action']
    level: int = Field(ge=0, le=3)
    message: str
    values: dict[str, str]
    received_at: AwareDatetime
    age_seconds: Number
    availability: Availability


class NavigationStatus(Model):
    # Independent, explicitly versioned extension; TwinState v1/v2 are unchanged.
    protocol_version: Literal['1.0'] = '1.0'
    message_type: Literal['navigation_status'] = 'navigation_status'
    diagnostics: tuple[NavigationDiagnostic, ...]
