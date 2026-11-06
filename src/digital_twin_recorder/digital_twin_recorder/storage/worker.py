"""Own the SDK lifecycle in a killable child process, outside ROS callbacks."""
import json
import multiprocessing as mp
from queue import Empty
import signal
import time
from .retrieval import SUMMARY_TOPIC, inspect_sequence


def connect(config):
    """Mosaico client; TLS (MOSAICO_TLS_CERT_FILE) and API key (MOSAICO_API_KEY) come from the environment."""
    from mosaicolabs import MosaicoClient
    return MosaicoClient.from_env(config.host, config.port, timeout=3)


def write(config, metadata, records, result, heartbeat):
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Parent owns graceful finalization.
    try:
        from mosaicolabs import Message, SessionLevelErrorPolicy, SessionStatus
        from ..models.ontology import ONTOLOGIES, RecordingSummaryV1
        specs = {s['topic']: s for s in metadata['recording_configuration']['topics']}
        with connect(config) as client:
            # Mosaico metadata keys cannot contain '/', so the topic map is stored as a list.
            stored_metadata = {**metadata, 'topics': [{'name': t, 'ros_type': k} for t, k in metadata['topics'].items()]}
            with client.sequence_create(metadata['execution_id'],metadata=stored_metadata,on_error=SessionLevelErrorPolicy.Report) as sequence:
                writers = {}
                result.put({'state':'recording'})
                last_probe=time.monotonic()
                parent=mp.parent_process()
                while True:
                    if parent is not None and not parent.is_alive():
                        # Recorder killed without finalizing: leave the sequence incomplete.
                        raise RuntimeError('Recorder process exited before finalization')
                    heartbeat.value=time.monotonic()
                    if time.monotonic()-last_probe > 3:
                        client.info(); last_probe=time.monotonic()
                    try: item=records.get(timeout=.2)
                    except Empty: continue
                    if 'finalize' in item:
                        summary=item['finalize']
                        missing=[t for t,s in specs.items() if s['required'] and not summary['topics'][t]['count']]
                        if missing:
                            raise ValueError('Missing required samples: '+', '.join(missing))
                        if any(summary['topics'][t]['count'] for t in specs if t not in writers):
                            raise RuntimeError('Recorded samples without a storage topic')
                        writer=sequence.topic_create(SUMMARY_TOPIC,metadata={'schema_version':'2.0'},ontology_type=RecordingSummaryV1)
                        if writer is None: raise RuntimeError('Summary topic creation failed')
                        writer.push(Message(timestamp_ns=time.time_ns(),data=RecordingSummaryV1(summary_json=json.dumps(summary))))
                        if writer.last_error: raise RuntimeError(str(writer.last_error))
                        break
                    topic=item['source_topic']
                    spec=specs[topic]
                    ontology=ONTOLOGIES[spec['ontology_tag']]
                    writer=writers.get(topic)
                    if writer is None:
                        # Topics are created on their first sample: optional topics that never
                        # published do not leave empty streams behind.
                        writer=sequence.topic_create(topic_name=topic,
                            metadata=dict(robot_id=config.robot_id,source_topic=topic,ros_type=spec['ros_type'],kind=spec['kind'],
                                          ontology_tag=spec['ontology_tag'],qos=spec['qos'],max_rate_hz=spec['max_rate_hz'],
                                          source_stamp=spec['source_stamp'],schema_version='1.0'),
                            ontology_type=ontology)
                        if writer is None: raise RuntimeError(f'Topic creation failed: {topic}')
                        writers[topic]=writer
                    writer.push(Message(timestamp_ns=item['received_wall_ns'],data=ontology.model_validate(item)))
                    if writer.last_error: raise RuntimeError(str(writer.last_error))
            if sequence.session_status != SessionStatus.Finalized: raise RuntimeError('Session not finalized')
            dropped={t:w.dropped_record_count for t,w in writers.items() if w.dropped_record_count}
            if dropped: raise RuntimeError(f'Storage dropped records: {dropped}')
            heartbeat.value=time.monotonic()
            verification=inspect_sequence(client,metadata['execution_id'],summary)
            result.put({'state':'completed','verification':verification})
    except Exception as error:
        result.put({'state':'failed','error':f'{type(error).__name__}: {error}'})
