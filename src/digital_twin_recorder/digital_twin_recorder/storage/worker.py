"""Own the SDK lifecycle in a killable child process, outside ROS callbacks."""
import json
from queue import Empty
import signal
import time
from .retrieval import SUMMARY_TOPIC, inspect_sequence

def write(config, metadata, records, result, heartbeat):
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Parent owns graceful finalization.
    try:
        from mosaicolabs import MosaicoClient, Message, SessionLevelErrorPolicy, SessionStatus
        from ..models.ontology import RosOdometryV1, RosImuV1, RecordingSummaryV1
        with MosaicoClient.connect(config.host,config.port,timeout=3) as client:
            stored_metadata = {**metadata, 'topics': [{'name': t, 'ros_type': k} for t, k in metadata['topics'].items()]}
            with client.sequence_create(metadata['execution_id'],metadata=stored_metadata,on_error=SessionLevelErrorPolicy.Report) as sequence:
                writers = {topic:sequence.topic_create(topic_name=topic,
                    metadata=dict(robot_id=config.robot_id,source_topic=topic,ros_type=kind,schema_version='1.0'),
                    ontology_type=RosOdometryV1 if kind=='nav_msgs/msg/Odometry' else RosImuV1)
                    for topic,kind in metadata['topics'].items()}
                result.put({'state':'recording'})
                last_probe=time.monotonic()
                while True:
                    heartbeat.value=time.monotonic()
                    if time.monotonic()-last_probe > 3:
                        client.info(); last_probe=time.monotonic()
                    try: item=records.get(timeout=.2)
                    except Empty: continue
                    if 'finalize' in item:
                        summary=item['finalize']
                        if not all(v['count'] for v in summary['topics'].values()):
                            raise ValueError('Missing required odometry or IMU samples')
                        writer=sequence.topic_create(SUMMARY_TOPIC,metadata={'schema_version':'1.0'},ontology_type=RecordingSummaryV1)
                        writer.push(Message(timestamp_ns=time.time_ns(),data=RecordingSummaryV1(summary_json=json.dumps(summary))))
                        break
                    topic=item['source_topic']
                    ontology=RosOdometryV1 if topic==config.odom_topic else RosImuV1
                    writer=writers[topic]
                    writer.push(Message(timestamp_ns=item['received_wall_ns'],data=ontology.model_validate(item)))
                    if writer.last_error: raise RuntimeError(str(writer.last_error))
            if sequence.session_status != SessionStatus.Finalized: raise RuntimeError('Session not finalized')
            heartbeat.value=time.monotonic()
            verification=inspect_sequence(client,metadata['execution_id'],summary)
            result.put({'state':'completed','verification':verification})
    except Exception as error:
        result.put({'state':'failed','error':f'{type(error).__name__}: {error}'})
