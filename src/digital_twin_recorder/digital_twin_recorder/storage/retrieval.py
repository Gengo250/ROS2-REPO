"""Retrieve and validate real sequences, without ROS or a running simulation."""
import argparse
import json
from pathlib import Path
from ..models.records import Integrity

SUMMARY_TOPIC = '/recording/summary'

def inspect_sequence(client, name, expected=None):
    # Import registers our typed ontologies before reading persisted Arrow data.
    from ..models.ontology import RosOdometryV1, RosImuV1, RecordingSummaryV1
    handler = client.sequence_handler(name)
    if handler is None: raise ValueError('Sequence not found: '+name)
    inventory = handler.topics
    metadata = handler.user_metadata
    if metadata.get('project') != 'digital_twin': raise ValueError('Not a Digital Twin recording')
    topics = {item['name']: item['ros_type'] for item in metadata['topics']}
    if set(inventory) != {*topics, SUMMARY_TOPIC}: raise ValueError('Incomplete topic inventory')
    values = {topic:Integrity() for topic in topics}
    summary = None
    try:
        for topic, message in handler.get_data_streamer():
            if topic == SUMMARY_TOPIC:
                if summary is not None or not isinstance(message.data, RecordingSummaryV1):
                    raise ValueError('Invalid recording summary')
                summary = json.loads(message.data.summary_json)
                continue
            record = message.data.model_dump()
            ontology = RosOdometryV1 if topics[topic] == 'nav_msgs/msg/Odometry' else RosImuV1
            ontology.model_validate(record)
            if record['source_topic'] != topic or record['robot_id'] != metadata['robot_id']:
                raise ValueError('Source identity mismatch')
            if message.timestamp_ns != record['received_wall_ns']: raise ValueError('Reception timestamp mismatch')
            values[topic].add(record)
        if summary is None: raise ValueError('Recording did not complete finalization')
        actual = {topic:value.report() for topic,value in values.items()}
        if any(not value.count for value in values.values()): raise ValueError('Required topic has no samples')
        if actual != summary['topics']: raise ValueError('Stored recording integrity mismatch')
        if expected is not None and actual != expected['topics']: raise ValueError('Source-to-storage integrity mismatch')
        notifications = client.list_sequence_notifications(name)
        if notifications: raise ValueError('Sequence has ingestion notifications: '+str(notifications))
        return dict(sequence=name, metadata=metadata, topics=actual, summary=summary,
                    storage_bytes=handler.total_size_bytes, validated=True)
    finally: handler.close()

def main():
    from mosaicolabs import MosaicoClient
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1'); parser.add_argument('--port',type=int,default=6726)
    parser.add_argument('--sequence'); parser.add_argument('--expected',type=Path); parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    with MosaicoClient.connect(args.host,args.port,timeout=3) as client:
        if args.sequence:
            result=inspect_sequence(client,args.sequence,json.loads(args.expected.read_text()) if args.expected else None)
        else:
            result={'sequences': [name for name in client.list_sequences() if name.startswith('digital_twin_')]}
    text=json.dumps(result,indent=2)
    if args.output: args.output.write_text(text+'\n')
    print(text)

if __name__ == '__main__': main()
