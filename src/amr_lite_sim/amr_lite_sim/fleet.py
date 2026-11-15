"""Naming of the robots' ROS interfaces (topics and TF frames) in the lite engine.

The reference robot (``amr_01``) keeps the root topics and unprefixed frames of the single-robot
twin (/odom, /scan, /cmd_vel, odom → base_link ...), so every Phase 6 tool keeps working unchanged.
Every other robot lives in its own namespace with prefixed frames: /amr_02/odom, /amr_02/cmd_vel,
amr_02/odom → amr_02/base_link. All robots share /tf, /tf_static, /clock and the ``map`` frame.
"""
ROOT_ROBOT = 'amr_01'


def namespace(robot_id, root=ROOT_ROBOT):
    """ROS namespace of a robot ('' for the root robot)."""
    return '' if robot_id == root else robot_id


def frame_prefix(robot_id, root=ROOT_ROBOT):
    """TF frame prefix of a robot ('' for the root robot, else '<id>/')."""
    return '' if robot_id == root else robot_id+'/'


def topic(robot_id, name, root=ROOT_ROBOT):
    """Absolute topic of ``name`` (relative, e.g. 'odom' or 'camera/image_raw') for a robot."""
    ns = namespace(robot_id, root)
    return f'/{ns}/{name}' if ns else f'/{name}'


def parse_robots(value):
    """Robot ids from a launch/parameter string ('amr_01,amr_02' or 'all' / '' for every spawn)."""
    value = (value or '').strip()
    if value in ('', 'all'):
        return None
    ids = [v.strip() for v in value.split(',') if v.strip()]
    if len(set(ids)) != len(ids):
        raise ValueError(f'duplicate robot ids in {value!r}')
    return ids
