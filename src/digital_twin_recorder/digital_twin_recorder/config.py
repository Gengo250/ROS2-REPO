"""Recorder configuration does not import the storage SDK or the Gateway."""
from dataclasses import dataclass
import os
from pathlib import Path
import re
from .topics import PROFILES

BACKENDS = ('lite', 'gazebo')
# Phase 1-5 Gazebo warehouse scenarios: descriptive metadata of a Gazebo recording only.
GAZEBO_SCENARIOS = ('normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked')
TOPIC = re.compile(r'/[A-Za-z0-9_/]+')
HOST = re.compile(r'[A-Za-z0-9.\-:\[\]]+')


def default_host():
    return os.environ.get('MOSAICO_HOST', '127.0.0.1')


def default_port():
    return int(os.environ.get('MOSAICO_PORT', '6726'))


@dataclass(frozen=True)
class Config:
    host: str = '127.0.0.1'
    port: int = 6726
    profile: str = 'core'
    simulation_backend: str = 'lite'
    scenario: str | None = None
    robot_id: str = 'amr_01'
    odom_topic: str = '/odom'
    imu_topic: str = '/imu/data'
    # Upper bounds on the recorded rate (simulation time); 0 records every message.
    odom_hz: float = 0.
    imu_hz: float = 0.
    world_state_hz: float = 10.
    scan_hz: float = 2.
    plan_hz: float = 0.
    tf_pairs: tuple = (('map', 'odom'),)
    queue_size: int = 2048
    io_timeout: float = 15.0
    finalize_timeout: float = 120.0
    discovery_timeout: float = 10.0
    duration: float = 0.0
    output: Path = Path('recordings')

    def __post_init__(self):
        if self.profile not in PROFILES: raise ValueError('Invalid profile')
        if self.simulation_backend not in BACKENDS: raise ValueError('Invalid simulation backend')
        if self.scenario is not None and (self.simulation_backend != 'gazebo' or self.scenario not in GAZEBO_SCENARIOS):
            raise ValueError('Invalid scenario: Gazebo warehouse scenarios only')
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', self.robot_id): raise ValueError('Invalid robot ID')
        if not HOST.fullmatch(self.host): raise ValueError('Invalid host')
        if not 1 <= self.port <= 65535: raise ValueError('Invalid port')
        if not 1 <= self.queue_size <= 65536: raise ValueError('Invalid queue bound')
        if not 1 <= self.io_timeout <= 120 or not 1 <= self.finalize_timeout <= 900: raise ValueError('Invalid deadline')
        if not 0 <= self.discovery_timeout <= 120: raise ValueError('Invalid discovery timeout')
        if self.duration < 0: raise ValueError('Invalid duration')
        if any(r < 0 for r in (self.odom_hz, self.imu_hz, self.world_state_hz, self.scan_hz, self.plan_hz)):
            raise ValueError('Invalid rate')
        if self.odom_topic == self.imu_topic: raise ValueError('Topics must be distinct')
        for topic in (self.odom_topic, self.imu_topic):
            if not TOPIC.fullmatch(topic): raise ValueError('Invalid topic')
        for pair in self.tf_pairs:
            if len(pair) != 2 or not all(re.fullmatch(r'[A-Za-z0-9_/]+', f) for f in pair): raise ValueError('Invalid TF pair')
