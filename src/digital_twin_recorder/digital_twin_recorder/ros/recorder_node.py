"""Best-effort subscriptions; storage is isolated in a bounded subprocess.

ROS callbacks only convert, decimate, hash and enqueue (``RecordingSession.submit`` never
blocks and never does network I/O). The storage worker owns the Mosaico SDK. A failed
execution is terminal; with ``--recover`` a NEW execution (new Mosaico sequence, linked by
``previous_execution_id``) starts after a back-off instead of resuming the failed one.
"""
import argparse
from pathlib import Path
import signal
import time
import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from ..config import Config, default_host, default_port
from ..models.records import convert_message, timestamp
from ..models.recording_metadata import create_metadata, map_facts
from ..storage.session import RecordingSession
from ..topics import PROFILES, inventory

QOS = {'sensor': qos_profile_sensor_data,
       'reliable': QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
       'latched': QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)}


class RecorderNode(Node):
    def __init__(self, config, specs):
        super().__init__('digital_twin_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.config, self.specs = config, specs
        self.session=None; self.clock=None; self.previous_state=None
        self.engine=None          # latest (session_id, epoch, scene_sha256, sim_time) of the lite engine
        # Latest message of each latched (transient-local) topic with its reception times. Such a
        # topic delivers its current value once, when the subscription matches (usually before an
        # execution starts), so every execution begins with it (start_execution).
        self.latched={}
        self.reset_counters()
        for spec in specs:
            self.create_subscription(get_message(spec.ros_type), spec.topic, lambda msg, spec=spec: self.record(msg, spec), QOS[spec.qos])
        self.create_subscription(Clock,'/clock',self.clock_received,qos_profile_sensor_data)
        # Wall-clock timer: storage health is polled even while simulation time stands still.
        self.create_timer(.2,self.check,clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def reset_counters(self):
        self.kept={}; self.counts={s.topic: dict(received=0, recorded=0, decimated=0, filtered=0) for s in self.specs}
        self.recording_engine=None; self.sim_span=[None, None]

    # ------------------------------------------------------------------ discovery
    def parameters(self, node_name, names, timeout):
        """Values of ``names`` on a running node through its GetParameters service, or None."""
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        client = self.create_client(GetParameters, f'{node_name}/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(GetParameters.Request(names=names))
            deadline = time.monotonic()+timeout
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
            if not future.done() or future.result() is None:
                return None
            return dict(zip(names, (parameter_value_to_python(v) for v in future.result().values)))
        finally:
            self.destroy_client(client)

    def discover(self):
        """Facts of the running system for the execution metadata (never guessed)."""
        facts, unavailable, timeout = {}, [], self.config.discovery_timeout
        if self.config.simulation_backend == 'lite':
            deadline = time.monotonic()+timeout
            while self.engine is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
            if self.engine:
                session_id, epoch, scene, sim_time = self.engine
                facts.update(simulation_session_id=session_id, simulation_epoch=epoch, engine_scene_sha256=scene,
                             sim_time_at_discovery_s=sim_time)
            else:
                unavailable.append('simulation_session_id')
            params = self.parameters('/lite_sim', ['profile', 'real_time_factor', 'seed', 'step'], min(timeout, 5.))
            if params:
                facts.update(simulation_profile=params['profile'], simulation_target_real_time_factor=params['real_time_factor'],
                             simulation_seed=params['seed'], simulation_step_s=params['step'])
            else:
                unavailable.append('simulation_profile')
        params = self.parameters('/map_server', ['yaml_filename'], min(timeout, 5.))
        if params and params['yaml_filename']:
            try:
                facts.update(map_facts(params['yaml_filename']))
            except Exception as error:
                unavailable.append('map_version')
                self.get_logger().warning(f'Map metadata unavailable: {error}')
        else:
            unavailable.append('map_version')   # no map server: e.g. engine without navigation
        facts['unavailable'] = unavailable
        return facts

    # ------------------------------------------------------------------ callbacks
    def clock_received(self,msg):
        current=timestamp(msg.clock)
        if self.clock is not None and current < self.clock and self.session and self.session.state=='recording':
            self.session.fail('Simulation clock reset; refusing to merge executions')
        self.clock=current

    def record(self, msg, spec, received_wall_ns=None, reception_sim_ns=None):
        if received_wall_ns is None:
            received_wall_ns, reception_sim_ns = time.time_ns(), self.get_clock().now().nanoseconds
        if spec.kind == 'world_state':
            self.engine = (msg.session_id, int(msg.epoch), msg.scene_sha256, float(msg.sim_time))
        if spec.qos == 'latched':
            self.latched[spec.topic] = (msg, spec, received_wall_ns, reception_sim_ns)
        session = self.session
        if session is None or session.state != 'recording':
            return
        counts = self.counts[spec.topic]
        counts['received'] += 1
        try:
            record = convert_message(spec.kind, msg, spec.topic, self.config.robot_id, received_wall_ns=received_wall_ns,
                                     reception_sim_ns=reception_sim_ns, tf_pairs=self.config.tf_pairs)
        except ValueError as error:
            session.fail(str(error)); return
        if record is None:
            counts['filtered'] += 1; return
        stamp = record['source_timestamp_ns']
        if spec.kind == 'world_state':
            identity = (msg.session_id, int(msg.epoch))
            if self.recording_engine is None:
                self.recording_engine = identity
            elif identity != self.recording_engine:
                session.fail('Simulation session or epoch changed; refusing to merge executions'); return
            self.sim_span = [self.sim_span[0] if self.sim_span[0] is not None else msg.sim_time, msg.sim_time]
        last = self.kept.get(spec.topic)
        # Decimation on the source (simulation) clock. A stamp that goes back is never decimated:
        # it reaches the integrity check, which fails the execution.
        if spec.max_rate_hz and last is not None and last <= stamp < last+1e9/spec.max_rate_hz:
            counts['decimated'] += 1; return
        if session.submit(record):
            self.kept[spec.topic] = stamp
            counts['recorded'] += 1

    def start_execution(self):
        """First records of a new execution: the current value of every latched topic."""
        for msg, spec, wall, sim in list(self.latched.values()):
            if not self.counts[spec.topic]['recorded']:
                self.record(msg, spec, wall, sim)

    def check(self):
        session = self.session
        if session is None:
            return
        state=session.poll()
        session.observed.update(topic_counts=self.counts, simulation_session=list(self.recording_engine or ()),
                                sim_time_span_s=self.sim_span)
        if state != self.previous_state:
            self.get_logger().info(f'Recording {state}: {session.metadata["execution_id"]} {session.error or ""}')
            session.save(); self.previous_state=state


def parse(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default=default_host(),help='Mosaico host (env MOSAICO_HOST)')
    parser.add_argument('--port',type=int,default=default_port(),help='Mosaico port (env MOSAICO_PORT)')
    parser.add_argument('--profile',choices=PROFILES,default='core',help='core: motion and events; extended: also plan, TF, scans')
    parser.add_argument('--simulation-backend',choices=['lite','gazebo'],default='lite')
    parser.add_argument('--scenario',default=None,help='Gazebo warehouse scenario (metadata, Gazebo backend only)')
    parser.add_argument('--robot-id',default='amr_01')
    parser.add_argument('--duration',type=float,default=0,help='wall seconds after storage is ready; 0 = until Ctrl+C')
    parser.add_argument('--output',type=Path,default=Path('recordings'))
    parser.add_argument('--queue-size',type=int,default=2048)
    parser.add_argument('--io-timeout',type=float,default=15)
    parser.add_argument('--finalize-timeout',type=float,default=120)
    parser.add_argument('--discovery-timeout',type=float,default=10)
    parser.add_argument('--odom-topic',default='/odom'); parser.add_argument('--imu-topic',default='/imu/data')
    parser.add_argument('--odom-hz',type=float,default=0.); parser.add_argument('--imu-hz',type=float,default=0.)
    parser.add_argument('--world-state-hz',type=float,default=10.,help='0 = every engine snapshot (30 Hz)')
    parser.add_argument('--scan-hz',type=float,default=2.,help='extended profile; 0 = full sensor rate')
    parser.add_argument('--plan-hz',type=float,default=0.)
    parser.add_argument('--tf-pair',action='append',default=None,metavar='PARENT:CHILD',help='extended profile; default map:odom')
    parser.add_argument('--recover',action='store_true',help='after a failure start a new execution instead of exiting')
    parser.add_argument('--recover-delay',type=float,default=5.)
    a=parser.parse_args(args)
    pairs=tuple(tuple(p.split(':',1)) for p in a.tf_pair) if a.tf_pair else (('map','odom'),)
    options={k:v for k,v in vars(a).items() if k not in ('tf_pair','recover','recover_delay')}
    return Config(**options,tf_pairs=pairs), a.recover, a.recover_delay


def main(args=None):
    config, recover, recover_delay = parse(args)
    specs=inventory(config)
    stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=RecorderNode(config,specs); reports=[]; previous=None; started_all=time.monotonic()
    try:
        while not stopping:
            metadata=create_metadata(config,specs,node.discover())
            if previous: metadata['previous_execution_id']=previous
            node.reset_counters()
            session=RecordingSession(config,metadata); node.session=session; node.previous_state=None
            session.start(); recording_started=None
            while not stopping and session.state not in ('failed','completed'):
                rclpy.spin_once(node,timeout_sec=.1)
                if session.state=='recording' and recording_started is None:
                    recording_started=time.monotonic(); node.start_execution()
                if recording_started and config.duration and time.monotonic()-recording_started>=config.duration: break
            node.check()
            report=session.finish('signal' if stopping else 'duration' if session.state=='recording' else 'failure')
            node.session=None; reports.append(report)
            print(f'Recording {report["state"]}: {session.path / "report.json"}',flush=True)
            previous=metadata['execution_id']
            if report['state']=='completed' or not recover or (config.duration and time.monotonic()-started_all>=config.duration):
                break
            node.get_logger().error(f'Execution {previous} failed ({report["error"]}); starting a new execution in {recover_delay:.0f} s')
            deadline=time.monotonic()+recover_delay
            while not stopping and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    finally:
        if node.session is not None:
            report=node.session.finish('signal'); reports.append(report)
            print(f'Recording {report["state"]}: {node.session.path / "report.json"}',flush=True)
        node.destroy_node(); rclpy.shutdown()
    if not reports or reports[-1]['state']!='completed': raise SystemExit(1)

if __name__ == '__main__': main()
