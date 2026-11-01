#!/usr/bin/env python3
"""WebSocket feed probe: arrival jitter, payload size and per-entity source-time gaps.

Usage: ws_probe.py <ws-url> <seconds> <out.json>
Works with TwinState v2 (simulation_robots/humans) and v3 (entities) payloads.
"""
import asyncio
import json
import sys
import time
import websockets


def stats(values):
    if not values:
        return None
    v = sorted(values)
    pick = lambda q: v[min(len(v)-1, int(q*(len(v)-1)))]
    return {'n': len(v), 'mean': sum(v)/len(v), 'p50': pick(.5), 'p95': pick(.95), 'p99': pick(.99), 'max': v[-1]}


async def main(url, seconds, out):
    arrivals, sizes, latency, source = [], [], [], {}
    async with websockets.connect(url, max_size=2**22) as ws:
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            try:
                text = await asyncio.wait_for(ws.recv(), timeout=max(.1, end-time.monotonic()))
            except asyncio.TimeoutError:
                break
            now = time.monotonic()
            arrivals.append(now)
            sizes.append(len(text))
            data = json.loads(text)
            if 'server_wall_time' in data:
                latency.append(time.time()-data['server_wall_time'])
            entities = data.get('entities') or [*data.get('simulation_robots', []), *data.get('humans', [])]
            for e in entities:
                stamp = e.get('source_timestamp', e.get('stamp'))
                if stamp is not None:
                    source.setdefault(e['id'], []).append((now, stamp))
    gaps = [b-a for a, b in zip(arrivals, arrivals[1:])]
    entity = {}
    for ident, samples in source.items():
        changes = [samples[0]]
        for s in samples[1:]:
            if s[1] != changes[-1][1]:
                changes.append(s)
        entity[ident] = {'fresh_pose_wall_gap_s': stats([b[0]-a[0] for a, b in zip(changes, changes[1:])]),
                         'fresh_pose_hz': (len(changes)-1)/(changes[-1][0]-changes[0][0]) if len(changes) > 1 else 0}
    report = {'url': url, 'messages': len(arrivals), 'hz': (len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals) > 1 else 0,
              'arrival_gap_s': stats(gaps), 'payload_bytes': stats(sizes), 'latency_s': stats(latency), 'entities': entity}
    json.dump(report, open(out, 'w'), indent=1)
    print(json.dumps({'hz': report['hz'], 'gap_p99': (report['arrival_gap_s'] or {}).get('p99'),
                      'gap_max': (report['arrival_gap_s'] or {}).get('max'),
                      'bytes_p50': (report['payload_bytes'] or {}).get('p50'),
                      'entity_gap_max': {k: (v['fresh_pose_wall_gap_s'] or {}).get('max') for k, v in entity.items()}}))


if __name__ == '__main__':
    asyncio.run(main(sys.argv[1], float(sys.argv[2]), sys.argv[3]))
