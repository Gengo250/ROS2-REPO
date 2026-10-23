"""SceneBroadcaster → explicitly framed, filtered ROS ground truth (never /tf).

The scene service supplies parentage, Pose_V supplies all current local poses.
Only configured top-level models and human descendants are forwarded. IDs are
used to join the two Gazebo messages, never as application identities.
"""
from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from gz.transport13 import Node as GzNode
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.pose_v_pb2 import Pose_V
from ..config import Settings


def scene_frames(scene, mapping, world_frame):
    frames = {}
    names = [m.name for m in scene.model]
    if len(names) != len(set(names)):
        raise ValueError('Ambiguous top-level model names')
    for model in scene.model:
        if model.name not in mapping:
            continue
        identity = mapping[model.name]
        frames[model.id] = (model.name, world_frame, identity)
        if not identity.startswith('human_'):
            continue
        for link in model.link:
            child = identity+'/'+link.name
            frames[link.id] = (link.name, identity, child)
            for visual in link.visual:
                frames[visual.id] = (visual.name, child, child+'/'+visual.name)
    children = [v[2] for v in frames.values()]
    if len(children) != len(set(children)):
        raise ValueError('Ambiguous scoped entity names')
    return frames


def convert_poses(message, frames):
    result = TFMessage()
    for pose in message.pose:
        frame = frames.get(pose.id)
        if frame is None or pose.name != frame[0]:
            continue
        transform = TransformStamped()
        transform.header.stamp.sec = message.header.stamp.sec
        transform.header.stamp.nanosec = message.header.stamp.nsec
        transform.header.frame_id, transform.child_frame_id = frame[1:]
        for axis in 'xyz':
            setattr(transform.transform.translation, axis, getattr(pose.position, axis))
        for axis in 'xyzw':
            setattr(transform.transform.rotation, axis, getattr(pose.orientation, axis))
        result.transforms.append(transform)
    return result


class SimulationAdapter(Node):
    def __init__(self, settings):
        super().__init__('digital_twin_simulation_adapter')
        self.settings = settings
        self.lock = Lock()
        self.frames, self.latest = {}, None
        self.gz = GzNode()
        self.pub = self.create_publisher(TFMessage, settings.simulation_pose_topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.gz.subscribe(Pose_V, f'/world/{settings.world_id}/pose/info', self.receive)
        # Queries run only in this dedicated adapter, never in Gateway callbacks
        # or HTTP requests. The timeout and latest-value slot are bounded.
        self.create_timer(2., self.refresh_scene)
        self.create_timer(.05, self.publish_latest)
        self.refresh_scene()

    def refresh_scene(self):
        ok, scene = self.gz.request(f'/world/{self.settings.world_id}/scene/info',
                                   Empty(), Empty, Scene, 200)
        if ok:
            try:
                frames = scene_frames(scene, self.settings.simulation_entities, self.settings.world_frame)
            except ValueError as error:
                self.get_logger().error(str(error))
                frames = {}
            with self.lock:
                self.frames = frames

    def receive(self, message):
        with self.lock:
            self.latest = convert_poses(message, self.frames)

    def publish_latest(self):
        with self.lock:
            message, self.latest = self.latest, None
        if message is not None and message.transforms:
            self.pub.publish(message)

    def destroy_node(self):
        self.gz.unsubscribe(f'/world/{self.settings.world_id}/pose/info')
        return super().destroy_node()


def main():
    rclpy.init()
    node = SimulationAdapter(Settings.from_env())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
