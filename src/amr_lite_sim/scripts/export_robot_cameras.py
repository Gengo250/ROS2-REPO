#!/usr/bin/env python3
"""Export the AMR's onboard camera mount for the browser twin, derived from the robot model.

The lens pose is base_link → camera_optical_frame composed from the URDF's fixed joints (xacro,
gazebo:=false; full roll/pitch/yaw), so the browser never restates it. Optics come from
amr_dimensions.json; the image topics from the Gazebo bridge (they exist only with the Gazebo
backend and sensors:=full; the lite engine publishes no camera images).
Usage: export_robot_cameras.py [--output web/digital-twin/public/robots/amr.cameras.json] [--check]
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml

OUTPUT = Path(__file__).resolve().parents[3]/'web/digital-twin/public/robots/amr.cameras.json'


def rpy_matrix(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def frame_pose(urdf_text, frame, base='base_link'):
    """Position and rotation matrix of ``frame`` in ``base``, composing the fixed joints."""
    root = ET.fromstring(urdf_text)
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain, link = [], frame
    while link != base:
        joint = parents.get(link)
        if joint is None or joint.get('type') != 'fixed':
            raise ValueError(f'{frame} is not rigidly attached to {base}')
        chain.append(joint)
        link = joint.find('parent').get('link')
    position, rotation = np.zeros(3), np.eye(3)
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = np.array([float(v) for v in origin.get('xyz', '0 0 0').split()]) if origin is not None else np.zeros(3)
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0., 0., 0.]
        position, rotation = position+rotation@xyz, rotation@rpy_matrix(*rpy)
    return position, rotation


def quaternion(m):
    """Rotation matrix → (x, y, z, w), w ≥ 0."""
    w = math.sqrt(max(0., 1+m[0, 0]+m[1, 1]+m[2, 2]))/2
    x = math.copysign(math.sqrt(max(0., 1+m[0, 0]-m[1, 1]-m[2, 2]))/2, m[2, 1]-m[1, 2])
    y = math.copysign(math.sqrt(max(0., 1-m[0, 0]+m[1, 1]-m[2, 2]))/2, m[0, 2]-m[2, 0])
    z = math.copysign(math.sqrt(max(0., 1-m[0, 0]-m[1, 1]+m[2, 2]))/2, m[1, 0]-m[0, 1])
    return [x, y, z, w]


def describe(urdf_text, dimensions, bridge):
    position, rotation = frame_pose(urdf_text, 'camera_optical_frame')
    topics = {entry['ros_topic_name']: entry for entry in bridge}
    clean = lambda values: [round(float(v), 6)+0. for v in values]
    return {
        'schema_version': '1.0', 'robot': 'amr',
        'source': 'amr_description/urdf/amr.urdf.xacro (gazebo:=false): base_link → camera_optical_frame composed from '
                  'the fixed joints; optics from config/amr_dimensions.json; topics from config/bridge.yaml',
        'cameras': [{
            'id': 'rgbd_camera', 'parent_frame': 'base_link', 'frame_id': 'camera_optical_frame',
            'convention': 'REP 103 optical frame: x right, y down, z forward',
            'position': clean(position), 'quaternion_xyzw': clean(quaternion(rotation)),
            'horizontal_fov': round(math.radians(dimensions['camera_fov_degrees']), 6),
            'resolution': [dimensions['camera_image_width'], dimensions['camera_image_height']],
            'near': dimensions['camera_near'], 'far': dimensions['camera_far'], 'rate_hz': dimensions['camera_rate'],
            'image_topics': {'availability': 'Gazebo backend with sensors:=full only; the lite engine publishes no images',
                             **{key: name for key, name in (('rgb', '/camera/image_raw'), ('depth', '/camera/depth/image_raw'),
                                                            ('camera_info', '/camera/camera_info')) if name in topics}},
        }],
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, default=OUTPUT)
    p.add_argument('--check', action='store_true', help='fail if the file differs from the robot model')
    a = p.parse_args()
    from ament_index_python.packages import get_package_share_directory
    from amr_lite_sim.robot_model import robot_urdf
    share = Path(get_package_share_directory('amr_description'))
    text = json.dumps(describe(robot_urdf(), json.loads((share/'config/amr_dimensions.json').read_text()),
                               yaml.safe_load((share/'config/bridge.yaml').read_text())), indent=1, ensure_ascii=False)+'\n'
    if a.check:
        if not a.output.exists() or a.output.read_text() != text:
            raise SystemExit(f'{a.output} differs from the robot model; run export_robot_cameras.py')
        print('camera mount up to date')
        return
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(text)
    print(f'wrote {a.output}')


if __name__ == '__main__':
    main()
