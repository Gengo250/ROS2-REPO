"""WarehouseScene v1.0. Poses are parent-relative XYZ + fixed-axis RPY."""
from typing import Annotated, Literal
from pydantic import BaseModel, ConfigDict, Field, model_validator
from ..state.models import Scenario

Pose = tuple[float, float, float, float, float, float]
Positive = Annotated[float, Field(gt=0)]

class Model(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)

class Primitive(Model):
    id: str
    kind: Literal['box', 'cylinder', 'sphere', 'label']
    dimensions: list[Positive]
    transforms: list[Pose]
    colour: tuple[float, float, float, float]
    label: str | None = None
    category: Literal['structure', 'wall', 'pillar', 'beam', 'floor', 'marking', 'equipment', 'human']

    @model_validator(mode='after')
    def dimensions_match(self):
        if len(self.dimensions) != {'box': 3, 'cylinder': 2, 'sphere': 1, 'label': 2}[self.kind]:
            raise ValueError('Incorrect primitive dimension count')
        return self

class Entity(Model):
    id: str
    pose: Pose
    kind: str
    pose_source: Literal['configured_initial_pose'] = 'configured_initial_pose'

class Station(Model):
    id: str
    label: str
    pose: Pose
    goal_pose: Pose
    footprint: tuple[Positive, Positive]

class Spawn(Model):
    id: str
    robot_id: str
    active: bool
    pose: Pose

class Area(Model):
    id: str
    centre: tuple[float, float]
    size: tuple[Positive, Positive]
    enabled: bool = True

class Metadata(Model):
    world_id: str
    scenario: Scenario
    configured_scenario: Scenario | None
    source_version: int
    source_sha256: dict[str, str]
    geometry_sha256: str

class Coordinates(Model):
    frame: Literal['world'] = 'world'
    x: Literal['east'] = 'east'
    y: Literal['north'] = 'north'
    z: Literal['up'] = 'up'
    units: Literal['metres_radians'] = 'metres_radians'
    rotation: Literal['Rz(yaw) Ry(pitch) Rx(roll)'] = 'Rz(yaw) Ry(pitch) Rx(roll)'

class Building(Model):
    length: Positive
    width: Positive
    height: Positive
    floor_size: tuple[Positive, Positive, Positive]

class WarehouseScene(Model):
    schema_version: Literal['1.0'] = '1.0'
    metadata: Metadata
    coordinate_system: Coordinates = Coordinates()
    building: Building
    entities: list[Entity]
    primitives: list[Primitive]
    aisles: list[Area]
    zones: list[Area]
    stations: list[Station]
    robot_spawns: list[Spawn]
