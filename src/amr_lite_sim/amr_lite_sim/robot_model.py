"""Robot facts for the lite engine, read from the same sources as the Gazebo model.

Sensor mounts come from the URDF joint chain (xacro with gazebo:=false); sizes, limits and
sensor settings from amr_dimensions.json. Nothing is restated by hand.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .lidar import ScannerSpec


@dataclass(frozen=True)
class RobotModel:
    length: float
    width: float
    height: float
    ground_clearance: float
    footprint_offset: float      # chassis centre ahead of base_link
    wheel_radius: float
    wheel_separation: float      # odometry separation used for wheel joint rates
    max_linear: float
    max_angular: float
    max_linear_acceleration: float
    max_angular_acceleration: float
    command_timeout: float
    front: ScannerSpec
    top: ScannerSpec
    imu_rate: float
    urdf: str


def _origin(joint):
    o = joint.find('origin')
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    if abs(rpy[0]) > 1e-9 or abs(rpy[1]) > 1e-9:
        raise ValueError('lite engine supports only yaw rotations in the sensor chain')
    return xyz, rpy[2]


def frame_in_base(urdf_text, frame):
    """Planar pose (x, y, z, yaw) of ``frame`` in base_link, composing fixed joints."""
    root = ET.fromstring(urdf_text)
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain = []
    link = frame
    while link != 'base_link':
        joint = parents.get(link)
        if joint is None:
            raise ValueError(f'{frame} is not connected to base_link')
        chain.append(joint)
        link = joint.find('parent').get('link')
    x = y = z = yaw = 0.
    for joint in reversed(chain):
        (jx, jy, jz), jyaw = _origin(joint)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, z, yaw = x+c*jx-s*jy, y+s*jx+c*jy, z+jz, yaw+jyaw
    return x, y, z, yaw


def robot_urdf():
    import xacro
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('amr_description'))
    return xacro.process_file(str(share/'urdf/amr.urdf.xacro'), mappings={'gazebo': 'false'}).toxml()


def load(urdf_text=None, dimensions_path=None, noise_stddev=0.):
    if urdf_text is None:
        urdf_text = robot_urdf()
    if dimensions_path is None:
        from ament_index_python.packages import get_package_share_directory
        dimensions_path = Path(get_package_share_directory('amr_description'))/'config/amr_dimensions.json'
    d = json.loads(Path(dimensions_path).read_text())
    fx, fy, fz, fyaw = frame_in_base(urdf_text, 'laser_frame')
    tx, ty, tz, tyaw = frame_in_base(urdf_text, 'top_laser_frame')
    chassis_x = frame_in_base(urdf_text, 'chassis_link')[0]
    half = math.radians(d['lidar_fov_degrees'])/2
    top_step = 2*math.pi/d['top_lidar_samples']
    return RobotModel(
        # Collision envelope: chassis plus the front bumper (as the Nav2 footprint).
        length=d['chassis_length']+d['bumper_length'], width=d['chassis_width'],
        height=tz+d['top_lidar_height']/2, ground_clearance=d['ground_clearance'],
        footprint_offset=chassis_x+d['bumper_length']/2,
        wheel_radius=d['wheel_radius'], wheel_separation=d.get('odometry_wheel_separation', d['track_width']),
        max_linear=d['max_linear_velocity'], max_angular=d['max_angular_velocity'],
        max_linear_acceleration=d['max_linear_acceleration'], max_angular_acceleration=d['max_angular_acceleration'],
        command_timeout=d['command_timeout'],
        # Same beam layout as the Gazebo gpu_lidar sensors (sensors_gazebo.xacro).
        front=ScannerSpec('laser_frame', d['lidar_samples'], -half, half, d['lidar_min_range'], d['lidar_max_range'],
                          d['lidar_rate'], fx, fy, fz, fyaw, noise_stddev),
        top=ScannerSpec('top_laser_frame', d['top_lidar_samples'], -math.pi, math.pi-top_step, d['top_lidar_min_range'],
                        d['top_lidar_max_range'], d['top_lidar_rate'], tx, ty, tz, tyaw, noise_stddev),
        imu_rate=50., urdf=urdf_text)
