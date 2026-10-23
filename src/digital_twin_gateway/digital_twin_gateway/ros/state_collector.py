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


def normalize_simulation(settings, message, received_at):
    """Validate a coherent hierarchy, rejecting ambiguous frames and bad rotations."""
    import math
    from ..state.models import DynamicEntity, EntityTransform
    transforms = {}
    stamps = set()
    for transform in message.transforms:
        name = transform.child_frame_id
        if name in transforms:
            raise ValueError('Duplicate simulation frame')
        q = transform.transform.rotation
        norm = sum(getattr(q, axis)**2 for axis in 'xyzw')
        if not math.isfinite(norm) or abs(norm-1) > .001:
            raise ValueError('Invalid simulation quaternion')
        stamps.add(stamp_seconds(transform.header.stamp))
        transforms[name] = transform
    if len(stamps) != 1 or len(transforms) > 256:
        raise ValueError('Incoherent simulation snapshot')
    def pose(tf):
        q = tf.transform.rotation
        return Pose(position=vector(tf.transform.translation),
                    orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w))
    entities = []
    for gazebo_name, identity in settings.simulation_entities.items():
        root = transforms.get(identity)
        if root is None:
            continue
        if root.header.frame_id != settings.world_frame:
            raise ValueError('Simulation root is not in world frame')
        components = []
        for name, tf in transforms.items():
            if not name.startswith(identity+'/'):
                continue
            parent = tf.header.frame_id
            if parent != name.rsplit('/', 1)[0] or parent not in transforms:
                raise ValueError('Broken simulation hierarchy')
            components.append(EntityTransform(id=name, frame_id=parent, pose=pose(tf)))
        kind = 'robot' if identity in {r.id for r in settings.robots} else (
            'human' if identity.startswith('human_') else 'obstacle')
        entities.append(DynamicEntity(id=identity, type=kind, gazebo_name=gazebo_name,
            frame_id=settings.world_frame, pose=pose(root), source_timestamp=next(iter(stamps)),
            received_at=received_at, age_seconds=0, status='online',
            pose_source=settings.simulation_pose_topic, components=tuple(components)))
    return entities
