"""Recorder configuration does not import the storage SDK or the Gateway."""
from dataclasses import dataclass
from pathlib import Path
import re

@dataclass(frozen=True)
class Config:
    host: str = '127.0.0.1'
    port: int = 6726
    scenario: str = 'normal'
    robot_id: str = 'amr_01'
    odom_topic: str = '/odom'
    imu_topic: str = '/imu/data'
    queue_size: int = 2048
    io_timeout: float = 15.0
    finalize_timeout: float = 30.0
    duration: float = 0.0
    output: Path = Path('recordings')

    def __post_init__(self):
        if self.scenario not in ('normal','person_crossing','obstacle_in_aisle','corridor_blocked'):
            raise ValueError('Invalid scenario')
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', self.robot_id): raise ValueError('Invalid robot ID')
        if not 1 <= self.port <= 65535: raise ValueError('Invalid port')
        if not 1 <= self.queue_size <= 65536: raise ValueError('Invalid queue bound')
        if not 1 <= self.io_timeout <= 120 or not 1 <= self.finalize_timeout <= 120: raise ValueError('Invalid deadline')
        if self.duration < 0: raise ValueError('Invalid duration')
        if self.odom_topic == self.imu_topic: raise ValueError('Topics must be distinct')
        for topic in (self.odom_topic,self.imu_topic):
            if not re.fullmatch(r'/[A-Za-z0-9_/]+', topic): raise ValueError('Invalid topic')
