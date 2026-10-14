"""Normalize ROS data once on receipt, preserving header and child frames."""
from ..state.models import Pose, Quaternion, Robot, Vector3, Velocity


def vector(value):
    return Vector3(x=value.x, y=value.y, z=value.z)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1_000_000_000


def normalize_odometry(robot_id, topic, message, received_at):
    p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
    return Robot(
        id=robot_id, frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
        pose_source=topic, pose=Pose(position=vector(p), orientation=Quaternion(
            x=q.x, y=q.y, z=q.z, w=q.w)),
        velocity=Velocity(linear=vector(v.linear), angular=vector(v.angular)),
        status='online', source_timestamp=stamp_seconds(message.header.stamp),
        received_at=received_at, age_seconds=0,
    )
