#!/usr/bin/env python3
"""Wait for the real world before starting ros_gz_sim's non-idempotent create.

The installed create client retries every five seconds, including requests sent
before service discovery. A slow cold start can therefore queue multiple create
requests. Only read-only scene requests may be retried during this barrier.
"""
import argparse
import time
from gz.transport13 import Node
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene


def wait_for_world(world, timeout=90):
    node = Node()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ok, scene = node.request(f'/world/{world}/scene/info', Empty(), Empty, Scene, 500)
        if ok and scene.model:
            print(f'Gazebo world {world} ready for entity creation', flush=True)
            return
        time.sleep(.1)
    raise TimeoutError(f'Gazebo world {world} did not become ready within {timeout} s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world')
    args = parser.parse_args()
    wait_for_world(args.world)
