"""Differential-drive kinematics with the robot's velocity and acceleration limits."""
from dataclasses import dataclass
import math


def clamp(value, low, high):
    return low if value < low else high if value > high else value


@dataclass
class DriveState:
    x: float
    y: float
    yaw: float
    v: float = 0.
    w: float = 0.
    left_angle: float = 0.
    right_angle: float = 0.
    command: tuple = (0., 0.)
    command_time: float = -math.inf
    accel: float = 0.


class DiffDrive:
    """Unicycle model: commanded (v, w) → rate-limited (v, w) → exact arc integration."""

    def __init__(self, model):
        self.m = model

    def target(self, state, now):
        """Latest command, zeroed after the controller's command timeout."""
        if now-state.command_time > self.m.command_timeout:
            return 0., 0.
        v, w = state.command
        return clamp(v, -self.m.max_linear, self.m.max_linear), clamp(w, -self.m.max_angular, self.m.max_angular)

    def velocities(self, state, now, dt):
        tv, tw = self.target(state, now)
        dv = clamp(tv-state.v, -self.m.max_linear_acceleration*dt, self.m.max_linear_acceleration*dt)
        dw = clamp(tw-state.w, -self.m.max_angular_acceleration*dt, self.m.max_angular_acceleration*dt)
        return state.v+dv, state.w+dw

    @staticmethod
    def advance(x, y, yaw, v, w, dt):
        """Exact pose after dt at constant (v, w)."""
        if abs(w) < 1e-9:
            return x+v*dt*math.cos(yaw), y+v*dt*math.sin(yaw), yaw
        r = v/w
        new_yaw = yaw+w*dt
        return x+r*(math.sin(new_yaw)-math.sin(yaw)), y-r*(math.cos(new_yaw)-math.cos(yaw)), \
            math.atan2(math.sin(new_yaw), math.cos(new_yaw))

    def wheel_rates(self, v, w):
        half = self.m.wheel_separation/2
        return (v-w*half)/self.m.wheel_radius, (v+w*half)/self.m.wheel_radius
