"""ROS message conversion and bounded integrity accumulator (no SDK import).

Every converter returns a plain dict that validates against the topic's ontology
(models/ontology.py). Values are copied as published: no unit, frame or quaternion
changes. Non-finite values are rejected except in LaserScan ranges/intensities, where
REP 117 gives +inf/-inf/NaN a meaning; the integrity hash encodes them explicitly.
"""
import hashlib
import json
import math
import time


def timestamp(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def vector(value, axes='xyz'):
    return [float(getattr(value, axis)) for axis in axes]


def _base(msg_frame, topic, robot_id, source_ns, received_wall_ns):
    return dict(robot_id=robot_id, source_topic=topic, source_timestamp_ns=source_ns,
                received_wall_ns=received_wall_ns if received_wall_ns is not None else time.time_ns(), frame_id=msg_frame)


def _finite(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite(v) for v in value.values())
    if isinstance(value, list):
        return all(_finite(v) for v in value)
    return True


def convert(msg, topic, robot_id, received_wall_ns=None):
    """nav_msgs/Odometry or sensor_msgs/Imu (Phase 2 contract, unchanged)."""
    record = _base(msg.header.frame_id, topic, robot_id, timestamp(msg.header.stamp), received_wall_ns)
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


def _entity(e):
    return dict(id=e.id, type=e.type, x=float(e.x), y=float(e.y), z=float(e.z), yaw=float(e.yaw), vx=float(e.vx),
                vy=float(e.vy), wz=float(e.wz), speed=float(e.speed), state=e.state, activity=e.activity,
                activity_progress=float(e.activity_progress), carrying=bool(e.carrying), look=float(e.look),
                field_state=int(e.field_state), field_length=float(e.field_length), reason=e.reason)


def _obstacle(o):
    return dict(id=o.id, kind=o.kind, origin=o.origin, x=float(o.x), y=float(o.y), yaw=float(o.yaw),
                length=float(o.length), width=float(o.width), height=float(o.height))


def _transform(t):
    return dict(stamp_ns=timestamp(t.header.stamp), parent_frame_id=t.header.frame_id, child_frame_id=t.child_frame_id,
                translation=vector(t.transform.translation), rotation=vector(t.transform.rotation, 'xyzw'))


def _level(value):
    # diagnostic_msgs/DiagnosticStatus.level is a ROS ``byte``: bytes in rclpy.
    return value[0] if isinstance(value, (bytes, bytearray)) else int(value)


def convert_message(kind, msg, topic, robot_id, received_wall_ns=None, reception_sim_ns=0, tf_pairs=()):
    """Record for ``kind`` (topics.TopicSpec.kind), or None when a filter leaves nothing to keep."""
    if kind in ('odometry', 'imu'):
        return convert(msg, topic, robot_id, received_wall_ns)
    if kind == 'world_state':
        record = _base(msg.header.frame_id, topic, robot_id, timestamp(msg.header.stamp), received_wall_ns)
        record.update(session_id=msg.session_id, epoch=int(msg.epoch), sim_time=float(msg.sim_time),
                      real_time_factor=float(msg.real_time_factor), target_real_time_factor=float(msg.target_real_time_factor),
                      paused=bool(msg.paused), overruns=int(msg.overruns), scene_sha256=msg.scene_sha256,
                      entities=[_entity(e) for e in msg.entities], obstacles=[_obstacle(o) for o in msg.obstacles])
    elif kind == 'obstacles':
        record = _base(msg.header.frame_id, topic, robot_id, timestamp(msg.header.stamp), received_wall_ns)
        record['obstacles'] = [_obstacle(o) for o in msg.obstacles]
    elif kind == 'sim_event':
        record = _base(msg.header.frame_id, topic, robot_id, timestamp(msg.header.stamp), received_wall_ns)
        record.update(seq=int(msg.seq), kind=msg.kind, entity=msg.entity, other=msg.other, detail=msg.detail,
                      x=float(msg.x), y=float(msg.y))
    elif kind == 'goal_status':
        record = _base('', topic, robot_id, reception_sim_ns, received_wall_ns)
        record['statuses'] = [dict(goal_id=bytes(s.goal_info.goal_id.uuid).hex(), status=int(s.status),
                                   accepted_ns=timestamp(s.goal_info.stamp)) for s in msg.status_list]
    elif kind == 'diagnostics':
        record = _base(msg.header.frame_id, topic, robot_id, timestamp(msg.header.stamp), received_wall_ns)
        record['statuses'] = [dict(level=_level(s.level), name=s.name, message=s.message, hardware_id=s.hardware_id,
                                   values=[dict(key=v.key, value=v.value) for v in s.values]) for s in msg.status]
    elif kind == 'path':
        record = _base(msg.header.frame_id, topic, robot_id, timestamp(msg.header.stamp), received_wall_ns)
        record['poses'] = [dict(stamp_ns=timestamp(p.header.stamp), frame_id=p.header.frame_id,
                                position=vector(p.pose.position), orientation=vector(p.pose.orientation, 'xyzw'))
                           for p in msg.poses]
    elif kind == 'tf':
        pairs = {(a.strip('/'), b.strip('/')) for a, b in tf_pairs}
        kept = [t for t in msg.transforms if (t.header.frame_id.strip('/'), t.child_frame_id.strip('/')) in pairs]
        if not kept:
            return None
        record = _base('', topic, robot_id, max(timestamp(t.header.stamp) for t in kept), received_wall_ns)
        record['transforms'] = [_transform(t) for t in kept]
    elif kind == 'tf_static':
        record = _base('', topic, robot_id, reception_sim_ns, received_wall_ns)
        record['transforms'] = [_transform(t) for t in msg.transforms]
    elif kind == 'scan':
        record = _base(msg.header.frame_id, topic, robot_id, timestamp(msg.header.stamp), received_wall_ns)
        record.update(angle_min=float(msg.angle_min), angle_max=float(msg.angle_max), angle_increment=float(msg.angle_increment),
                      time_increment=float(msg.time_increment), scan_time=float(msg.scan_time),
                      range_min=float(msg.range_min), range_max=float(msg.range_max),
                      ranges=[float(r) for r in msg.ranges], intensities=[float(i) for i in msg.intensities])
        if not _finite({k: v for k, v in record.items() if k not in ('ranges', 'intensities')}):
            raise ValueError('Nonfinite ROS telemetry')
        return record
    else:
        raise ValueError(f'Unknown record kind {kind!r}')
    if not _finite(record):
        raise ValueError('Nonfinite ROS telemetry')
    return record


def _sanitized(value):
    """JSON-safe copy: non-finite floats become explicit strings (hash and report only)."""
    if isinstance(value, float):
        return value if math.isfinite(value) else ('NaN' if math.isnan(value) else ('+Inf' if value > 0 else '-Inf'))
    if isinstance(value, dict):
        return {k: _sanitized(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_sanitized(v) for v in value]
    return value


def canonical(record):
    try:
        text = json.dumps(record, sort_keys=True, separators=(',', ':'), allow_nan=False)
    except ValueError:  # REP 117 ranges only (converters reject every other non-finite value)
        text = json.dumps(_sanitized(record), sort_keys=True, separators=(',', ':'), allow_nan=False)
    return (text+'\n').encode()


class Integrity:
    def __init__(self):
        self.count = 0; self.digest = hashlib.sha256(); self.first = self.last = None
    def add(self, record):
        if self.last is not None and record['source_timestamp_ns'] < self.last['source_timestamp_ns']:
            raise ValueError('Source clock reset or out-of-order message')
        self.digest.update(canonical(record)); self.count += 1
        if self.first is None: self.first = _sanitized(record)
        self.last = _sanitized(record)
    def report(self):
        return dict(count=self.count, sha256=self.digest.hexdigest(), first=self.first, last=self.last)
