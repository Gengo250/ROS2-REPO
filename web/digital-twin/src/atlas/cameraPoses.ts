// Poses of the twin's views for the one Three.js camera, in WebGL axes (coords.ts).
// A Three.js camera looks along its local −Z with +Y up and +X right. The ROS optical frame
// (REP 103) has z forward, x right, y down, so camera-local = optical · diag(1, −1, −1).
import { Matrix4, Quaternion, Vector3 } from 'three';
import { rosToGl } from './coords';
import type { IndustrialCamera } from './scene';

export interface Pose { position: number[]; quaternion: number[] }
/** Vertical field of view in degrees, width/height, clip planes (metres). */
export interface Optics { fov: number; aspect: number; near: number; far: number }
/** Onboard camera of a robot model, as exported from the URDF (public/robots/*.cameras.json). */
export interface MountedCamera {
  id: string; parent_frame: string; frame_id: string; position: number[]; quaternion_xyzw: number[];
  horizontal_fov: number; resolution: [number, number]; near: number; far: number; rate_hz: number;
  image_topics: Record<string, string>;
}
export interface Rect { x: number; y: number; width: number; height: number }

// ROS world → WebGL as a rotation: X = x, Y = z, Z = −y.
const ROS_TO_GL = new Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);
const OPTICAL_TO_CAMERA = new Matrix4().makeRotationX(Math.PI);   // diag(1, −1, −1)
const Z = new Vector3(0, 0, 1);
/** Far plane of the rendered first-person views: the whole campus (not the sensor's depth range). */
export const VIEW_FAR = 260;
const deg = (rad: number) => rad * 180 / Math.PI;

/** Camera axes in the world frame; ``pitch`` is the depression angle (positive looks down). */
export function cameraAxes(cam: Pick<IndustrialCamera, 'yaw' | 'pitch'>) {
  const p = cam.pitch, c = Math.cos(cam.yaw), s = Math.sin(cam.yaw);
  return { forward: [Math.cos(p) * c, Math.cos(p) * s, -Math.sin(p)], right: [s, -c, 0], up: [Math.sin(p) * c, Math.sin(p) * s, Math.cos(p)] };
}

function toPose(position: number[], rosRotation: Matrix4): Pose {
  const q = new Quaternion().setFromRotationMatrix(new Matrix4().multiplyMatrices(ROS_TO_GL, rosRotation));
  return { position: rosToGl(position[0]!, position[1]!, position[2]!), quaternion: [q.x, q.y, q.z, q.w] };
}

/** Industrial camera: the lens at the scene's mount position, aimed by its yaw and pitch. */
export function industrialPose(cam: IndustrialCamera): Pose {
  const { forward: f, right: r, up: u } = cameraAxes(cam);
  // Camera-local X → right, Y → up, Z → backwards, expressed in the ROS world.
  const basis = new Matrix4().makeBasis(new Vector3(...r), new Vector3(...u), new Vector3(-f[0]!, -f[1]!, -f[2]!));
  return toPose(cam.position, basis);
}
export function industrialOptics(cam: IndustrialCamera): Optics {
  return { fov: deg(cam.vertical_fov), aspect: Math.tan(cam.horizontal_fov / 2) / Math.tan(cam.vertical_fov / 2), near: cam.near, far: VIEW_FAR };
}

/** Onboard camera lens from the robot's base pose (ROS world x, y, z, yaw) and its URDF mount
 * (base_link → optical frame): position = base + R(yaw)·p, rotation = R(yaw)·R_mount. */
export function onboardPose(base: { x: number; y: number; z: number; yaw: number }, mount: Pick<MountedCamera, 'position' | 'quaternion_xyzw'>): Pose {
  const yaw = new Quaternion().setFromAxisAngle(Z, base.yaw);
  const [qx, qy, qz, qw] = mount.quaternion_xyzw as [number, number, number, number];
  const optical = yaw.clone().multiply(new Quaternion(qx, qy, qz, qw));
  const p = new Vector3(mount.position[0], mount.position[1], mount.position[2]).applyQuaternion(yaw).add(new Vector3(base.x, base.y, base.z));
  return toPose([p.x, p.y, p.z], new Matrix4().makeRotationFromQuaternion(optical).multiply(OPTICAL_TO_CAMERA));
}
/** Square pixels, as Gazebo renders: vertical FOV from the horizontal one and the image size. */
export function onboardOptics(mount: MountedCamera): Optics {
  const [w, h] = mount.resolution;
  return { fov: deg(2 * Math.atan(Math.tan(mount.horizontal_fov / 2) * h / w)), aspect: w / h, near: mount.near, far: VIEW_FAR };
}

/** Orbit view: eye looking at a target with +Y up (WebGL axes). */
export function lookPose(eye: number[], target: number[]): Pose {
  const m = new Matrix4().lookAt(new Vector3(...eye), new Vector3(...target), new Vector3(0, 1, 0));
  const q = new Quaternion().setFromRotationMatrix(m);
  return { position: [...eye], quaternion: [q.x, q.y, q.z, q.w] };
}

/** Largest rectangle of the given aspect centred in ``area`` (sensor framing). */
export function fitViewport(area: Rect, aspect: number): Rect {
  const width = Math.min(area.width, area.height * aspect), height = width / aspect;
  return { x: area.x + (area.width - width) / 2, y: area.y + (area.height - height) / 2, width, height };
}
