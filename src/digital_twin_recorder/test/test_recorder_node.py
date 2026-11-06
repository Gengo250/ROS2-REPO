"""RecorderNode callbacks with a stand-in session (real rclpy, no Mosaico, no publishers)."""
import pytest
import rclpy
from amr_lite_interfaces.msg import Obstacle, ObstacleArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from digital_twin_recorder.config import Config
from digital_twin_recorder.ros.recorder_node import RecorderNode
from digital_twin_recorder.topics import inventory


class Session:
    def __init__(self):
        self.state, self.records, self.observed = 'connecting', [], {}
    def submit(self, record):
        self.records.append(record); return True
    def fail(self, reason):
        self.state = 'failed'


@pytest.fixture
def node():
    rclpy.init()
    config = Config(profile='extended')
    n = RecorderNode(config, inventory(config))
    yield n
    n.destroy_node(); rclpy.shutdown()


def test_latched_values_received_before_an_execution_start_it(node):
    specs = {s.topic: s for s in node.specs}
    obstacles = ObstacleArray(obstacles=[Obstacle(id='obstacle_0001', kind='cone', origin='user', x=1., y=2.)])
    static = TFMessage(transforms=[TransformStamped()])
    static.transforms[0].header.frame_id, static.transforms[0].child_frame_id = 'base_link', 'laser_frame'
    static.transforms[0].transform.rotation.w = 1.
    node.record(obstacles, specs['/lite_sim/obstacles'])        # discovery: no execution yet
    node.record(static, specs['/tf_static'])
    node.session = Session(); node.reset_counters()
    node.session.state = 'recording'
    node.start_execution()
    assert [r['source_topic'] for r in node.session.records] == ['/lite_sim/obstacles', '/tf_static']
    assert node.session.records[0]['obstacles'][0]['id'] == 'obstacle_0001'
    node.start_execution()                                         # never twice in one execution
    assert len(node.session.records) == 2
    # A later execution (e.g. after a recovery) starts again from the current latched values.
    node.session = Session(); node.reset_counters(); node.session.state = 'recording'
    node.start_execution()
    assert len(node.session.records) == 2
