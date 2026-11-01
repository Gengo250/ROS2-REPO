#!/usr/bin/env python3
"""Published-scan integrity: project live /scan and /top_scan through TF and ground truth into
the world frame and measure each point's distance to the scene geometry it should lie on.

Catches frame, angle-convention and timing mistakes between the engine, robot_state_publisher
and the messages consumers see. Ground truth is evaluation data only.
Usage: check_scan_geometry.py --output report.json [--samples 20]
"""
import argparse
import json
import math
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from atlas_scene.scene import Scene, rasterize


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--samples', type=int, default=20)
    a = p.parse_args()
    scene = Scene.load()
    res, origin = .02, (-80., -60.)
    shape = (int(120/res), int(160/res))
    from scipy.ndimage import distance_transform_edt
    fields = {}
    for frame, height in (('laser_frame', .213), ('top_laser_frame', .48)):
        grid = rasterize(scene.slice_polygons(height), origin, res, shape)
        fields[frame] = distance_transform_edt(~grid)*res
    rclpy.init()
    node = rclpy.create_node('scan_geometry_check')
    buffer = Buffer(node=node)
    TransformListener(buffer, node)
    truth = []
    node.create_subscription(TFMessage, '/digital_twin/simulation/poses', lambda m: truth.extend(
        (t.header.stamp.sec+t.header.stamp.nanosec*1e-9, t.transform.translation.x, t.transform.translation.y,
         yaw_of(t.transform.rotation)) for t in m.transforms if t.child_frame_id == 'amr_01'), qos_profile_sensor_data)
    scans = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/top_scan', scans.append, qos_profile_sensor_data)
    results = {'laser_frame': [], 'top_laser_frame': []}
    end = time.monotonic()+60
    while time.monotonic() < end and min(len(v) for v in results.values()) < a.samples:
        rclpy.spin_once(node, timeout_sec=.05)
        keep = []
        for scan in scans:
            stamp = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            match = [t for t in truth if abs(t[0]-stamp) < 1e-6]
            if not match:
                if not truth or truth[-1][0] < stamp+.2:
                    keep.append(scan)
                continue
            try:
                mount = buffer.lookup_transform('base_link', scan.header.frame_id, Time())
            except Exception:
                keep.append(scan)
                continue
            _, x, y, yaw = match[0]
            mx, my, myaw = mount.transform.translation.x, mount.transform.translation.y, yaw_of(mount.transform.rotation)
            sx, sy, syaw = x+math.cos(yaw)*mx-math.sin(yaw)*my, y+math.sin(yaw)*mx+math.cos(yaw)*my, yaw+myaw
            r = np.asarray(scan.ranges)
            ang = scan.angle_min+np.arange(len(r))*scan.angle_increment+syaw
            ok = np.isfinite(r)
            px, py = sx+r[ok]*np.cos(ang[ok]), sy+r[ok]*np.sin(ang[ok])
            col = ((px-origin[0])/res).astype(int)
            row = ((py-origin[1])/res).astype(int)
            inside = (col >= 0) & (col < shape[1]) & (row >= 0) & (row < shape[0])
            # Moving entities (workers, forklifts) are not in the static field; ignore far outliers.
            d = fields[scan.header.frame_id][row[inside], col[inside]]
            results[scan.header.frame_id].append([float(np.median(d)), float(np.percentile(d, 90)), int(ok.sum())])
        scans[:] = keep
    report = {frame: {'scans': len(v), 'median_point_distance_m': float(np.median([x[0] for x in v])) if v else None,
                      'p90_point_distance_m': float(np.median([x[1] for x in v])) if v else None,
                      'median_returns': float(np.median([x[2] for x in v])) if v else None} for frame, v in results.items()}
    report['note'] = 'Distance from each projected return to the nearest scene geometry at the scanner height (2 cm grid).'
    json.dump(report, open(a.output, 'w'), indent=1)
    print(json.dumps(report))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
