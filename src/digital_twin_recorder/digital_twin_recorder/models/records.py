"""ROS message conversion and bounded integrity accumulator (no SDK import)."""
import hashlib
import json
import math
import time

def timestamp(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec

def vector(value, axes='xyz'):
    return [float(getattr(value, axis)) for axis in axes]

def convert(msg, topic, robot_id, received_wall_ns=None):
    record = dict(robot_id=robot_id, source_topic=topic, source_timestamp_ns=timestamp(msg.header.stamp),
                  received_wall_ns=received_wall_ns if received_wall_ns is not None else time.time_ns(), frame_id=msg.header.frame_id)
    if hasattr(msg, 'child_frame_id'):
        record.update(child_frame_id=msg.child_frame_id, position=vector(msg.pose.pose.position),
            orientation=vector(msg.pose.pose.orientation,'xyzw'), linear_velocity=vector(msg.twist.twist.linear),
            angular_velocity=vector(msg.twist.twist.angular), pose_covariance=list(msg.pose.covariance),
            twist_covariance=list(msg.twist.covariance))
    else:
        record.update(orientation=vector(msg.orientation,'xyzw'), angular_velocity=vector(msg.angular_velocity),
            linear_acceleration=vector(msg.linear_acceleration), orientation_covariance=list(msg.orientation_covariance),
            angular_velocity_covariance=list(msg.angular_velocity_covariance), linear_acceleration_covariance=list(msg.linear_acceleration_covariance))
    if any(not math.isfinite(v) for value in record.values() if isinstance(value,list) for v in value):
        raise ValueError('Nonfinite ROS telemetry')
    return record

def canonical(record):
    return (json.dumps(record,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()

class Integrity:
    def __init__(self):
        self.count = 0; self.digest = hashlib.sha256(); self.first = self.last = None
    def add(self, record):
        if self.last is not None and record['source_timestamp_ns'] < self.last['source_timestamp_ns']:
            raise ValueError('Source clock reset or out-of-order message')
        self.digest.update(canonical(record)); self.count += 1
        if self.first is None: self.first = record
        self.last = record
    def report(self):
        return dict(count=self.count, sha256=self.digest.hexdigest(), first=self.first, last=self.last)
