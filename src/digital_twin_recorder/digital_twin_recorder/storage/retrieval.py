"""Retrieve and validate real sequences, without ROS or a running simulation.

``inspect_sequence`` is the integrity check used by the worker after finalization and by
the history CLI (storage/history.py). It reads every stored record back, rebuilds its typed
ontology and recomputes the per-topic SHA-256 that the recorder computed on the ROS side.
Schema 1.0 recordings (Phase 2: odometry and IMU only) remain readable.
"""
import argparse
import json
from pathlib import Path
from ..models.records import Integrity

SUMMARY_TOPIC = '/recording/summary'
V1_TAGS = {'nav_msgs/msg/Odometry': 'digital_twin_ros_odometry_v1', 'sensor_msgs/msg/Imu': 'digital_twin_ros_imu_v1'}


def topic_ontologies(metadata):
    """{topic: ontology tag} configured for a recording (schema 2.0) or implied by its ROS types (1.0)."""
    configuration = metadata.get('recording_configuration')
    if configuration:
        return {t['topic']: t['ontology_tag'] for t in configuration['topics']}
    return {item['name']: V1_TAGS[item['ros_type']] for item in metadata['topics']}


def required_topics(metadata):
    configuration = metadata.get('recording_configuration')
    if configuration:
        return {t['topic'] for t in configuration['topics'] if t['required']}
    return {item['name'] for item in metadata['topics']}


def read_summary(handler):
    """The finalization summary, or None when the recording never finalized."""
    from ..models.ontology import RecordingSummaryV1  # noqa: F401  registers the ontology
    if SUMMARY_TOPIC not in handler.topics:
        return None
    for _, message in handler.get_data_streamer(topics=[SUMMARY_TOPIC]):
        return json.loads(message.data.summary_json)
    return None


def inspect_sequence(client, name, expected=None):
    # Import registers our typed ontologies before reading persisted Arrow data.
    from ..models.ontology import ONTOLOGIES, RecordingSummaryV1
    handler = client.sequence_handler(name)
    if handler is None: raise ValueError('Sequence not found: '+name)
    inventory = handler.topics
    metadata = handler.user_metadata
    if metadata.get('project') != 'digital_twin': raise ValueError('Not a Digital Twin recording')
    tags = topic_ontologies(metadata)
    if not set(inventory) <= {*tags, SUMMARY_TOPIC}: raise ValueError('Unexpected topic in the recording')
    if SUMMARY_TOPIC not in inventory: raise ValueError('Recording did not complete finalization')
    values = {topic:Integrity() for topic in tags}
    summary = None
    try:
        for topic, message in handler.get_data_streamer():
            if topic == SUMMARY_TOPIC:
                if summary is not None or not isinstance(message.data, RecordingSummaryV1):
                    raise ValueError('Invalid recording summary')
                summary = json.loads(message.data.summary_json)
                continue
            record = message.data.model_dump()
            ONTOLOGIES[tags[topic]].model_validate(record)
            if record['source_topic'] != topic or record['robot_id'] != metadata['robot_id']:
                raise ValueError('Source identity mismatch')
            if message.timestamp_ns != record['received_wall_ns']: raise ValueError('Reception timestamp mismatch')
            values[topic].add(record)
        if summary is None: raise ValueError('Recording did not complete finalization')
        actual = {topic:value.report() for topic,value in values.items()}
        if any(not actual[t]['count'] for t in required_topics(metadata)): raise ValueError('Required topic has no samples')
        if set(inventory)-{SUMMARY_TOPIC} != {t for t, v in actual.items() if v['count']}:
            raise ValueError('Incomplete topic inventory')
        if actual != summary['topics']: raise ValueError('Stored recording integrity mismatch')
        if expected is not None and actual != expected['topics']: raise ValueError('Source-to-storage integrity mismatch')
        notifications = client.list_sequence_notifications(name)
        if notifications: raise ValueError('Sequence has ingestion notifications: '+str(notifications))
        return dict(sequence=name, metadata=metadata, topics=actual, summary=summary,
                    storage_bytes=handler.total_size_bytes, validated=True)
    finally: handler.close()

def main():
    from .worker import connect
    from ..config import default_host, default_port
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default=default_host()); parser.add_argument('--port',type=int,default=default_port())
    parser.add_argument('--sequence'); parser.add_argument('--expected',type=Path); parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    with connect(args) as client:
        if args.sequence:
            result=inspect_sequence(client,args.sequence,json.loads(args.expected.read_text()) if args.expected else None)
        else:
            result={'sequences': [name for name in client.list_sequences() if name.startswith('digital_twin_')]}
    text=json.dumps(result,indent=2)
    if args.output: args.output.write_text(text+'\n')
    print(text)

if __name__ == '__main__': main()
