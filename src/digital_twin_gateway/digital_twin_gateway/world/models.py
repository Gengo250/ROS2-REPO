"""world-state v1.0: the backend-authoritative world for the LIVE ROS frontend.

Independent, explicitly versioned contract (JSON Schema and browser types are generated
from these models). Twin v1/v2 and the route contracts are unchanged. Every pose is in the
world frame (x east, y north, z up, metres, radians); map-frame data is converted through
the verified world → map registration or omitted.
"""
from typing import Annotated, Literal
from pydantic import AwareDatetime, Field
from ..state.models import Availability, Model, Number

Short = Annotated[str, Field(max_length=64)]
Point2 = tuple[Number, Number]
FieldState = Literal['free', 'warning', 'protective']
ObstacleKind = Literal['cone', 'box', 'pallet']


class WorldEntity(Model):
    id: Short
    type: Literal['amr', 'human', 'forklift']
    x: Number
    y: Number
    z: Number
    yaw: Number
    vx: Number
    vy: Number
    wz: Number
    speed: Number
    state: Short
    activity: Short | None
    activity_progress: Number = Field(ge=0, le=1)
    carrying: bool
    look: Number
    field_state: FieldState
    field_length: Number = Field(ge=0)
    reason: Annotated[str, Field(max_length=128)] | None


class WorldObstacle(Model):
    id: Short
    kind: ObstacleKind
    origin: Literal['user']
    x: Number
    y: Number
    yaw: Number
    length: Number = Field(gt=0)
    width: Number = Field(gt=0)
    height: Number = Field(gt=0)


class WorldEvent(Model):
    seq: int = Field(ge=1)
    sim_time: Number
    kind: Short
    entity: Short
    other: Short
    detail: Annotated[str, Field(max_length=256)]
    x: Number
    y: Number


class Simulation(Model):
    backend: Literal['lite', 'gazebo', 'none']
    status: Literal['unknown', 'running', 'paused', 'stalled', 'offline']
    availability: Availability
    engine_session_id: Short | None
    sim_time: Number | None
    real_time_factor: Number | None
    target_real_time_factor: Number | None
    overruns: int = Field(ge=0)
    scene_sha256: Short | None
    age_seconds: Number | None


MissionStatus = Literal['pending', 'waiting', 'active', 'recovering', 'succeeded', 'failed', 'canceled', 'rejected', 'blocked']


class Mission(Model):
    """One robot's mission. ``waiting``: accepted but held by fleet coordination (no Nav2 goal running):
    queued for its station, before a narrow corridor another AMR is using, or yielding to another AMR
    (``waiting_for`` / ``waiting_on`` / ``blocking_robot`` / ``queue_position`` say which).
    ``recovering``: Nav2 recovery behaviours under way. ``blocked``: stopped by the coordinator after a
    persistent conflict it could not resolve safely (intervention needed; the robot did not arrive)."""
    id: Short
    robot_id: Short
    destination: Short
    strategy: Literal['direct']
    status: MissionStatus
    detail: Annotated[str, Field(max_length=256)] | None
    goal: tuple[Number, Number, Number]
    distance_remaining_m: Number | None
    recoveries: int = Field(ge=0)
    created_at: AwareDatetime
    updated_at: AwareDatetime
    waiting_for: Literal['station', 'corridor', 'robot'] | None = None
    waiting_on: Short | None = None
    blocking_robot: Short | None = None
    queue_position: int | None = Field(default=None, ge=1)
    yields: int = Field(default=0, ge=0)


class StationReservation(Model):
    """A station as a single-capacity resource: the robot whose mission to it is under way, robots
    physically inside its occupancy disc, and the FIFO queue of waiting missions."""
    id: Short
    owner: Short | None
    occupants: tuple[Short, ...] = Field(max_length=16)
    queue: tuple[Short, ...] = Field(max_length=16)


class CorridorReservation(Model):
    """A capacity-1 corridor: the robot cleared to drive through it, robots inside, robots held before it."""
    id: Short
    claimed_by: Short | None
    occupants: tuple[Short, ...] = Field(max_length=16)
    waiting: tuple[Short, ...] = Field(max_length=16)


class RobotNavigation(Model):
    id: Short
    interlock: Annotated[str, Field(max_length=256)] | None
    interlock_ready: bool
    interlock_availability: Availability
    path: tuple[Point2, ...] = Field(max_length=256)
    path_age_seconds: Number | None
    path_frame: Literal['world']
    path_omitted_reason: Annotated[str, Field(max_length=128)] | None
    mission: Mission | None


class Connection(Model):
    gateway: Literal['online']
    ros: Availability
    simulation: Availability
    navigation: Availability
    registration: Literal['verified', 'missing']


class WorldState(Model):
    protocol_version: Literal['1.0'] = '1.0'
    message_type: Literal['world_state'] = 'world_state'
    session_id: Short
    sequence: int = Field(ge=1)
    timestamp: AwareDatetime
    server_wall_time: Number
    simulation: Simulation
    connection: Connection
    entities: tuple[WorldEntity, ...] = Field(max_length=256)
    robots: tuple[RobotNavigation, ...] = Field(max_length=16)
    obstacles: tuple[WorldObstacle, ...] = Field(max_length=256)
    events: tuple[WorldEvent, ...] = Field(max_length=50)
    stations: tuple[StationReservation, ...] = Field(default=(), max_length=64)
    corridors: tuple[CorridorReservation, ...] = Field(default=(), max_length=128)


class ObstacleRequest(Model):
    kind: ObstacleKind
    x: Number
    y: Number
    yaw: Number = 0.


class CommandResult(Model):
    protocol_version: Literal['1.0'] = '1.0'
    message_type: Literal['command_result'] = 'command_result'
    accepted: bool
    reason: Annotated[str, Field(max_length=256)] | None
    obstacle: WorldObstacle | None = None
    removed: int | None = None
    mission: Mission | None = None


class MissionRequest(Model):
    robot_id: Short = 'amr_01'
    destination: Short
    strategy: Literal['direct'] = 'direct'
