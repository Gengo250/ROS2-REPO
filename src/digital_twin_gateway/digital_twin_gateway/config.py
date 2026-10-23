"""Validated environment configuration; no ROS graph or simulation side effects."""
import json
import os
from pathlib import Path
from typing import Annotated
from pydantic import BaseModel, ConfigDict, Field, model_validator
from .state.models import Scenario

Topic = Annotated[str, Field(pattern=r'^/[A-Za-z0-9_/]+$')]


class RobotConfig(BaseModel):
    model_config = ConfigDict(extra='forbid', frozen=True)
    id: str = Field(default='amr_01', pattern=r'^[A-Za-z0-9_-]+$')
    odom_topic: Topic = '/odom'
    joint_states_topic: Topic = '/joint_states'


class Settings(BaseModel):
    model_config = ConfigDict(extra='forbid', frozen=True, allow_inf_nan=False)
    host: str = '127.0.0.1'
    port: int = Field(default=8000, ge=1, le=65535)
    publication_hz: float = Field(default=10, ge=0.1, le=60)
    stale_seconds: float = Field(default=2, gt=0)
    offline_seconds: float = Field(default=10, gt=0)
    send_timeout: float = Field(default=2, gt=0, le=30)
    max_clients: int = Field(default=64, ge=1, le=1024)
    allowed_origins: tuple[str, ...] = ('http://localhost:5173', 'http://127.0.0.1:5173')
    world_id: str = 'warehouse'
    world_frame: str = 'world'
    scenario: Scenario | None = None
    scenario_source: str = 'unspecified'
    robots: tuple[RobotConfig, ...] = (RobotConfig(),)
    simulation_pose_topic: Topic = '/digital_twin/simulation/poses'
    simulation_entities: dict[str, str] = {'warehouse_amr': 'amr_01', 'human_01': 'human_01',
                                          'human_02': 'human_02', 'human_03': 'human_03'}
    clock_topic: Topic = '/clock'
    tf_topic: Topic = '/tf'
    tf_static_topic: Topic = '/tf_static'

    @model_validator(mode='after')
    def validate_config(self):
        if len(set(self.simulation_entities.values())) != len(self.simulation_entities):
            raise ValueError('Simulation identities must be unique')
        if any(not k or '/' in k or not v or '/' in v for k, v in self.simulation_entities.items()):
            raise ValueError('Simulation mapping requires top-level model names and stable IDs')
        if self.simulation_pose_topic in (self.tf_topic, self.tf_static_topic):
            raise ValueError('Ground truth must not enter the TF graph')
        if self.offline_seconds <= self.stale_seconds:
            raise ValueError('offline_seconds must exceed stale_seconds')
        if not self.robots or len({r.id for r in self.robots}) != len(self.robots):
            raise ValueError('robots must have unique, nonempty stable IDs')
        if len({r.odom_topic for r in self.robots}) != len(self.robots):
            raise ValueError('each robot must have a distinct odometry topic')
        if any('*' in o or not o.startswith(('http://', 'https://')) for o in self.allowed_origins):
            raise ValueError('allowed origins must be explicit HTTP(S) origins')
        if self.scenario_source not in ('launch_configuration', 'environment', 'unspecified'):
            raise ValueError('invalid scenario source')
        return self

    @classmethod
    def from_env(cls):
        values = {}
        for key in cls.model_fields:
            value = os.environ.get('TWIN_' + key.upper())
            if value is not None:
                values[key] = json.loads(value) if key in ('robots', 'allowed_origins', 'simulation_entities') else value
        # The installed existing layout remains authoritative for world metadata.
        from ament_index_python.packages import get_package_share_directory
        layout = Path(get_package_share_directory('amr_simulation')) / 'config/warehouse_layout.json'
        data = json.loads(layout.read_text())
        values.setdefault('world_id', data['world_name'])
        values.setdefault('world_frame', data['coordinates']['frame'])
        if values.get('scenario'):
            values.setdefault('scenario_source', 'environment')
        return cls(**values)
