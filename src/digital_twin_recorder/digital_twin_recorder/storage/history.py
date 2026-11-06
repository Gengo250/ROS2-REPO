"""Historical recordings in Mosaico: list, inspect, export, trajectories, integrity.

Needs only the Mosaico service (no ROS graph, no simulator). Python use:

    from digital_twin_recorder.storage import history
    with history.open_client() as client:
        for row in history.list_recordings(client): ...
        for topic, record in history.records(client, name, topics=['/odom'], sim_from=10, sim_to=20): ...

CLI: ``python -m digital_twin_recorder.storage.history {health,list,show,export,trajectory,verify} ...``.
Time filters: ``--sim-from/--sim-to`` (seconds of simulation time, on source_timestamp_ns) or
``--wall-from/--wall-to`` (ISO-8601 UTC or Unix ns, on the Mosaico message timestamp = reception
wall time, applied by the server). Both are half-open: from <= t < to.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from ..config import default_host, default_port
from ..models.records import _sanitized
from .retrieval import SUMMARY_TOPIC, inspect_sequence, read_summary, topic_ontologies


class _Target:
    def __init__(self, host, port):
        self.host, self.port = host, port


def open_client(host=None, port=None):
    """MosaicoClient from MOSAICO_HOST/MOSAICO_PORT (+ MOSAICO_TLS_CERT_FILE, MOSAICO_API_KEY)."""
    from .worker import connect
    return connect(_Target(host or default_host(), port or default_port()))


def _wall_ns(text):
    if text is None:
        return None
    if text.isdigit():
        return int(text)
    stamp = datetime.fromisoformat(text.replace('Z', '+00:00'))
    if stamp.tzinfo is None:
        stamp = stamp.replace(tzinfo=timezone.utc)
    return int(stamp.timestamp()*1e9)


def list_recordings(client, prefix='digital_twin_'):
    """One row per Digital Twin sequence; ``status`` is 'finalized' only with a finalization summary."""
    from ..models import ontology  # noqa: F401  registers the ontologies
    rows = []
    for name in sorted(client.list_sequences()):
        if not name.startswith(prefix):
            continue
        handler = client.sequence_handler(name)
        if handler is None:
            continue
        try:
            meta = handler.user_metadata
            summary = read_summary(handler)
            counts = {t: v['count'] for t, v in (summary or {}).get('topics', {}).items()}
            rows.append(dict(execution_id=name, schema_version=meta.get('schema_version'),
                             backend=meta.get('simulation_backend', meta.get('simulator')), robot_id=meta.get('robot_id'),
                             profile=meta.get('recording_profile', 'phase2'), scene=meta.get('scene_id', meta.get('world')),
                             simulation_session_id=meta.get('simulation_session_id'),
                             started_wall_utc=meta.get('started_wall_utc'),
                             ended_wall_utc=(summary or {}).get('ended_wall_utc'),
                             status='finalized' if summary else 'incomplete',
                             stop_reason=(summary or {}).get('stop_reason'), message_counts=counts,
                             storage_bytes=handler.total_size_bytes))
        finally:
            handler.close()
    return rows


def describe(client, name):
    from ..models import ontology  # noqa: F401
    handler = client.sequence_handler(name)
    if handler is None:
        raise ValueError('Sequence not found: '+name)
    try:
        topics = {}
        for topic in handler.topics:
            th = handler.get_topic_handler(topic)
            topics[topic] = dict(ontology_tag=th.ontology_tag, user_metadata=th.user_metadata, size_bytes=th.total_size_bytes,
                                 wall_ns_min=th.timestamp_ns_min, wall_ns_max=th.timestamp_ns_max)
        return dict(execution_id=name, metadata=handler.user_metadata, summary=read_summary(handler), topics=topics,
                    storage_bytes=handler.total_size_bytes, sessions=len(handler.sessions))
    finally:
        handler.close()


def records(client, name, topics=None, wall_from=None, wall_to=None, sim_from=None, sim_to=None):
    """Yield (topic, record dict) in reception order; the summary topic is never yielded."""
    from ..models import ontology  # noqa: F401
    handler = client.sequence_handler(name)
    if handler is None:
        raise ValueError('Sequence not found: '+name)
    try:
        wanted = [t for t in (topics or handler.topics) if t != SUMMARY_TOPIC]
        missing = set(wanted)-set(handler.topics)
        if missing:
            raise ValueError(f'Topics not in {name}: {sorted(missing)}')
        lo = None if sim_from is None else int(round(sim_from*1e9))
        hi = None if sim_to is None else int(round(sim_to*1e9))
        for topic, message in handler.get_data_streamer(topics=wanted, start_timestamp_ns=wall_from, end_timestamp_ns=wall_to):
            record = message.data.model_dump()
            t = record['source_timestamp_ns']
            if (lo is None or t >= lo) and (hi is None or t < hi):
                yield topic, record
    finally:
        handler.close()


def trajectory(client, name, entity=None, source='world_state', **window):
    """Pose samples of one entity: world-frame ground truth (world_state) or odom-frame odometry."""
    handler = client.sequence_handler(name)
    if handler is None:
        raise ValueError('Sequence not found: '+name)
    try:
        meta = handler.user_metadata
    finally:
        handler.close()
    entity = entity or meta['robot_id']
    tags = topic_ontologies(meta)
    if source == 'world_state':
        topic = next((t for t, tag in tags.items() if tag == 'digital_twin_lite_world_state_v1'), None)
    else:
        topic = next((t for t, tag in tags.items() if tag == 'digital_twin_ros_odometry_v1'), None)
    if topic is None:
        raise ValueError(f'{name} has no {source} topic')
    rows = []
    for _, r in records(client, name, topics=[topic], **window):
        if source == 'world_state':
            e = next((e for e in r['entities'] if e['id'] == entity), None)
            if e is not None:
                rows.append(dict(sim_time_s=r['source_timestamp_ns']/1e9, frame='world', x=e['x'], y=e['y'], z=e['z'],
                                 yaw=e['yaw'], speed=e['speed'], state=e['state'], type=e['type']))
        else:
            qx, qy, qz, qw = r['orientation']
            rows.append(dict(sim_time_s=r['source_timestamp_ns']/1e9, frame=r['frame_id'], x=r['position'][0],
                             y=r['position'][1], z=r['position'][2],
                             yaw=math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)), speed=r['linear_velocity'][0],
                             state='', type='odometry'))
    return rows


def path_length(rows):
    return sum(math.dist((a['x'], a['y']), (b['x'], b['y'])) for a, b in zip(rows, rows[1:]))


def _emit(data, output):
    text = json.dumps(data, indent=1, default=str)
    if output:
        Path(output).write_text(text+'\n')
    print(text if len(text) < 20000 else text[:20000]+'\n... (truncated; see --output)')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default=None); p.add_argument('--port', type=int, default=None)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('health', help='server round trip: version and number of stored sequences (exit 1 if unreachable)')
    sub.add_parser('list', help='all Digital Twin recordings').add_argument('--output')
    for command in ('show', 'export', 'trajectory', 'verify'):
        s = sub.add_parser(command)
        s.add_argument('sequence')
        s.add_argument('--output')
        if command in ('export', 'trajectory'):
            s.add_argument('--sim-from', type=float); s.add_argument('--sim-to', type=float)
            s.add_argument('--wall-from'); s.add_argument('--wall-to')
        if command == 'export':
            s.add_argument('--topic', action='append', help='repeatable; default: every telemetry topic')
            s.add_argument('--limit', type=int, default=0, help='stop after N records (0 = all)')
        if command == 'trajectory':
            s.add_argument('--entity', help='entity id (default: the recording robot)')
            s.add_argument('--source', choices=['world_state', 'odometry'], default='world_state')
        if command == 'verify':
            s.add_argument('--expected', help='report.json written by the recorder for this execution')
    a = p.parse_args(argv)
    if a.command == 'health':
        try:
            with open_client(a.host, a.port) as client:
                names = client.list_sequences()
                print(json.dumps({'reachable': True, 'server_version': client.info().version, 'sequences': len(names),
                                  'digital_twin_sequences': sum(n.startswith('digital_twin_') for n in names)}))
        except Exception as error:
            print(json.dumps({'reachable': False, 'error': f'{type(error).__name__}: {error}'}))
            raise SystemExit(1)
        return
    with open_client(a.host, a.port) as client:
        if a.command == 'list':
            _emit(list_recordings(client), a.output)
        elif a.command == 'show':
            _emit(describe(client, a.sequence), a.output)
        elif a.command == 'verify':
            expected = json.loads(Path(a.expected).read_text()) if a.expected else None
            result = inspect_sequence(client, a.sequence, expected)
            _emit({k: v for k, v in result.items() if k != 'metadata'}, a.output)
        else:
            window = dict(wall_from=_wall_ns(a.wall_from), wall_to=_wall_ns(a.wall_to), sim_from=a.sim_from, sim_to=a.sim_to)
            if a.command == 'trajectory':
                rows = trajectory(client, a.sequence, a.entity, a.source, **window)
                out = open(a.output, 'w', newline='') if a.output else sys.stdout
                writer = csv.DictWriter(out, fieldnames=['sim_time_s', 'frame', 'x', 'y', 'z', 'yaw', 'speed', 'state', 'type'])
                writer.writeheader(); writer.writerows(rows)
                print(f'{len(rows)} samples, path length {path_length(rows):.3f} m', file=sys.stderr)
            else:
                out = open(a.output, 'w') if a.output else sys.stdout
                count = 0
                for topic, record in records(client, a.sequence, a.topic, **window):
                    out.write(json.dumps({'topic': topic, 'record': _sanitized(record)}, separators=(',', ':'))+'\n')
                    count += 1
                    if a.limit and count >= a.limit:
                        break
                print(f'{count} records', file=sys.stderr)


if __name__ == '__main__':
    main()
