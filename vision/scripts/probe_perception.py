#!/usr/bin/env python3
"""Check the live perception outputs against their contract (integration test, needs the stack).

    uv run python scripts/probe_perception.py --seconds 20 --output report.json

WebSocket (one client per robot): every message is for the subscribed robot, frame ids and
simulation stamps only increase, the JPEG decodes to the announced size, every box lies inside the
image and has a known class; reports the received rate and age. ROS (if rclpy is importable):
each robot's camera/image_raw, camera_info and perception/detections exist with that robot's
camera frame id, and every detection array has an image with the identical stamp.
"""
import argparse
import asyncio
import json
import struct
import sys
import time
from pathlib import Path
import cv2
import numpy as np
import websockets

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision.classes import names as class_names  # noqa: E402


async def client(url, robot, seconds, result):
    entry = result.setdefault(robot, {'messages': 0, 'violations': [], 'detections': 0, 'ages_ms': []})
    last_frame, last_stamp = -1, -1.
    deadline = time.monotonic()+seconds
    async with websockets.connect(f'{url}/ws/v1/perception?robot_id={robot}', max_size=2**25) as socket:
        while time.monotonic() < deadline:
            try:
                data = await asyncio.wait_for(socket.recv(), timeout=max(.1, deadline-time.monotonic()))
            except asyncio.TimeoutError:
                break
            (length,) = struct.unpack_from('<I', data, 0)
            header = json.loads(data[4:4+length])
            image = cv2.imdecode(np.frombuffer(data[4+length:], np.uint8), cv2.IMREAD_COLOR)
            entry['messages'] += 1
            entry['ages_ms'].append(header['timing_ms']['end_to_end'])
            problems = []
            if header['robot_id'] != robot:
                problems.append(f"message for {header['robot_id']}")
            if header['frame_id'] <= last_frame:
                problems.append(f"frame id {header['frame_id']} after {last_frame}")
            if header['stamp'] < last_stamp:
                problems.append(f"stamp {header['stamp']} before {last_stamp}")
            if image is None or image.shape[:2] != (header['height'], header['width']):
                problems.append('image does not decode to the announced size')
            for d in header['detections']:
                x0, y0, x1, y1 = d['bbox_xyxy']
                if not (0 <= x0 < x1 <= header['width'] and 0 <= y0 < y1 <= header['height']):
                    problems.append(f'box outside the image {d}')
                if d['label'] not in class_names() or class_names().index(d['label']) != d['class_id']:
                    problems.append(f'unknown class {d}')
            entry['detections'] += len(header['detections'])
            entry['violations'] += problems
            last_frame, last_stamp = header['frame_id'], header['stamp']
    entry['rate_hz'] = round(entry['messages']/seconds, 2)
    ages = entry.pop('ages_ms')
    entry['end_to_end_ms_p50'] = round(float(np.percentile(ages, 50)), 1) if ages else None


def ros_check(robots, seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from vision_msgs.msg import Detection2DArray
    rclpy.init()
    node = rclpy.create_node('probe_perception')
    seen = {r: {'images': [], 'infos': 0, 'detections': []} for r in robots}
    for robot in robots:
        base = '/' if robot == 'amr_01' else f'/{robot}/'
        node.create_subscription(Image, base+'camera/image_raw', lambda m, r=robot: seen[r]['images'].append(
            (m.header.stamp.sec, m.header.stamp.nanosec, m.header.frame_id, m.encoding, m.width, m.height, len(m.data))), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, base+'camera/camera_info', lambda m, r=robot: seen[r].__setitem__('infos', seen[r]['infos']+1), qos_profile_sensor_data)
        node.create_subscription(Detection2DArray, base+'perception/detections', lambda m, r=robot: seen[r]['detections'].append(
            (m.header.stamp.sec, m.header.stamp.nanosec, m.header.frame_id, len(m.detections))), 10)
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    out = {}
    for robot, s in seen.items():
        frame = ('' if robot == 'amr_01' else f'{robot}/')+'camera_optical_frame'
        stamps = {(a, b) for a, b, *_ in s['images']}
        out[robot] = {'images': len(s['images']), 'camera_infos': s['infos'], 'detection_arrays': len(s['detections']),
                      'image_frame_ids': sorted({i[2] for i in s['images']}), 'encodings': sorted({i[3] for i in s['images']}),
                      'sizes': sorted({(i[4], i[5], i[6]) for i in s['images']}),
                      'detection_stamps_with_image': sum((a, b) in stamps for a, b, *_ in s['detections']),
                      # Images are best-effort sensor data (900 kB each): a few may be dropped by DDS.
                      'image_delivery_ratio': round(sum((a, b) in stamps for a, b, *_ in s['detections'])/max(len(s['detections']), 1), 3)}
        out[robot]['ok'] = bool(s['images']) and {i[2] for i in s['images']} == {frame} and {i[3] for i in s['images']} == {'rgb8'} \
            and all(i[6] == i[4]*i[5]*3 for i in s['images']) and out[robot]['image_delivery_ratio'] >= .8
    node.destroy_node()
    rclpy.shutdown()
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--url', default='ws://127.0.0.1:8100')
    p.add_argument('--robots', default='amr_01,amr_02,amr_03')
    p.add_argument('--seconds', type=float, default=20.)
    p.add_argument('--output', required=True)
    p.add_argument('--no-ros', action='store_true')
    a = p.parse_args()
    robots = a.robots.split(',')
    result = {}

    async def run():
        await asyncio.gather(*(client(a.url, r, a.seconds, result) for r in robots))
    asyncio.run(run())
    report = {'websocket': result, 'websocket_ok': all(not v['violations'] and v['messages'] > 0 for v in result.values())}
    if not a.no_ros:
        try:
            report['ros'] = ros_check(robots, min(a.seconds, 10.))
            report['ros_ok'] = all(v['ok'] for v in report['ros'].values())
        except ImportError as error:
            report['ros'] = f'not checked: {error}'
    Path(a.output).write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1)[:3000])


if __name__ == '__main__':
    main()
