"""Best-effort sensor subscriptions; storage is isolated in a bounded subprocess."""
import argparse
from pathlib import Path
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from ..config import Config
from ..models.records import convert, timestamp
from ..models.recording_metadata import create_metadata
from ..storage.session import RecordingSession

class RecorderNode(Node):
    def __init__(self, session):
        super().__init__('digital_twin_recorder')
        self.session=session; self.clock=None; self.previous_state=None
        config=session.config
        self.create_subscription(Odometry,config.odom_topic,lambda msg:self.record(msg,config.odom_topic),qos_profile_sensor_data)
        self.create_subscription(Imu,config.imu_topic,lambda msg:self.record(msg,config.imu_topic),qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',self.clock_received,qos_profile_sensor_data)
        self.create_timer(.2,self.check)
    def clock_received(self,msg):
        current=timestamp(msg.clock)
        if self.clock is not None and current < self.clock and self.session.state=='recording':
            self.session.fail('Simulation clock reset; refusing to merge executions')
        self.clock=current
    def record(self,msg,topic):
        if self.session.state!='recording': return
        try: self.session.submit(convert(msg,topic,self.session.config.robot_id))
        except ValueError as error: self.session.fail(str(error))
    def check(self):
        state=self.session.poll()
        if state != self.previous_state:
            self.get_logger().info(f'Recording {state}: {self.session.metadata["execution_id"]} {self.session.error or ""}')
            self.session.save(); self.previous_state=state

def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1'); parser.add_argument('--port',type=int,default=6726)
    parser.add_argument('--scenario',default='normal'); parser.add_argument('--robot-id',default='amr_01')
    parser.add_argument('--duration',type=float,default=0)
    parser.add_argument('--output',type=Path,default=Path('recordings'))
    parser.add_argument('--queue-size',type=int,default=2048)
    parser.add_argument('--io-timeout',type=float,default=15)
    parser.add_argument('--finalize-timeout',type=float,default=30)
    parser.add_argument('--odom-topic',default='/odom'); parser.add_argument('--imu-topic',default='/imu/data')
    config=Config(**vars(parser.parse_args(args)))
    session=RecordingSession(config,create_metadata(config))
    stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=RecorderNode(session); session.start(); recording_started=None
    try:
        while not stopping and session.state not in ('failed','completed'):
            rclpy.spin_once(node,timeout_sec=.1)
            if session.state=='recording' and recording_started is None: recording_started=time.monotonic()
            if recording_started and config.duration and time.monotonic()-recording_started>=config.duration: break
    finally:
        node.destroy_node(); rclpy.shutdown()
        report=session.finish('signal' if stopping else 'duration' if session.state=='recording' else 'failure')
        print(f'Recording {report["state"]}: {session.path / "report.json"}',flush=True)
    if report['state']!='completed': raise SystemExit(1)

if __name__ == '__main__': main()
