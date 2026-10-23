#!/usr/bin/env python3
"""Independent Gazebo observer for real synchronization evidence, not production."""
import argparse
from collections import deque
import json
import signal
from threading import Lock
import time
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from digital_twin_gateway.config import Settings
from digital_twin_gateway.ros.simulation_adapter import scene_frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    settings = Settings.from_env()
    node, lock = Node(), Lock()
    samples, stats = deque(maxlen=18000), deque(maxlen=2000)
    frames, stop, count, total_bytes = {}, False, 0, 0
    def on_pose(message):
        nonlocal count, total_bytes
        wall = time.time()
        with lock:
            count += 1
            total_bytes += message.ByteSize()
            poses = {frames[p.id][2]: {'parent': frames[p.id][1],
                'position': [getattr(p.position,k) for k in 'xyz'],
                'quaternion': [getattr(p.orientation,k) for k in 'xyzw']}
                for p in message.pose if p.id in frames and p.name == frames[p.id][0]}
            if poses:
                samples.append({'wall':wall,'time':message.header.stamp.sec+message.header.stamp.nsec/1e9,'poses':poses})
    def on_stats(message):
        with lock:
            stats.append({'wall':time.time(), 'rtf':message.real_time_factor,'paused':message.paused})
    def finish(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    node.subscribe(Pose_V, f'/world/{settings.world_id}/pose/info', on_pose)
    node.subscribe(WorldStatistics, f'/world/{settings.world_id}/stats', on_stats)
    start = time.time()
    while not stop:
        ok, scene = node.request(f'/world/{settings.world_id}/scene/info', Empty(), Empty, Scene, 200)
        if ok:
            mapping = scene_frames(scene,settings.simulation_entities,settings.world_frame)
            with lock: frames = mapping
        for _ in range(20):
            if stop: break
            time.sleep(.1)
    node.unsubscribe(f'/world/{settings.world_id}/pose/info')
    node.unsubscribe(f'/world/{settings.world_id}/stats')
    with lock:
        report = {'start_wall':start,'end_wall':time.time(),'messages':count,
                  'total_bytes':total_bytes,'samples':list(samples),'stats':list(stats)}
    with open(args.output,'w') as stream: json.dump(report,stream)

if __name__ == '__main__': main()
