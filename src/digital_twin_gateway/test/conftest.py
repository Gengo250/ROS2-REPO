"""Deterministic clocks and lifecycle doubles; ROS messages remain real types."""
from datetime import datetime, timezone
import pytest
from nav_msgs.msg import Odometry
from digital_twin_gateway.config import Settings
from digital_twin_gateway.state.state_manager import StateManager


class Timer:
    value = 0.0
    def __call__(self):
        return self.value


class Runtime:
    def __init__(self, manager, settings):
        self.manager = manager
        self.stopped = False
    def start(self):
        self.manager.set_ros_initialized(True)
    def stop(self):
        self.stopped = True
        self.manager.set_ros_initialized(False)


@pytest.fixture
def state():
    timer = Timer()
    settings = Settings()
    manager = StateManager(settings, timer, lambda: datetime(2026, 10, 13, tzinfo=timezone.utc))
    manager.set_ros_initialized(True)
    return manager, timer


@pytest.fixture
def odom():
    msg = Odometry()
    msg.header.frame_id = 'robot_a/odom'
    msg.child_frame_id = 'robot_a/base_link'
    msg.header.stamp.sec, msg.header.stamp.nanosec = 123, 250_000_000
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1.2, -2.3, 0.4
    q = msg.pose.pose.orientation
    q.x, q.y, q.z, q.w = 0.1, 0.2, 0.3, 0.9
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = 0.4, 0.2, -0.1
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = 0.1, -0.2, 0.3
    return msg
