import { Euler, Matrix4, Quaternion, Vector3 } from 'three';

// Renderer world = ROS world: X east, Y north, Z up, metres. No odom mapping.
export function poseMatrix(pose: number[]): Matrix4 {
  const [x, y, z, roll, pitch, yaw] = pose;
  return new Matrix4().compose(new Vector3(x, y, z),
    new Quaternion().setFromEuler(new Euler(roll, pitch, yaw, 'ZYX')), new Vector3(1, 1, 1));
}
export function worldMatrix(transforms: number[][]): Matrix4 {
  return transforms.reduce((matrix, pose) => matrix.multiply(poseMatrix(pose)), new Matrix4());
}
export function rosQuaternion(q: { x: number; y: number; z: number; w: number }): Quaternion {
  return new Quaternion(q.x, q.y, q.z, q.w);
}
// glTF specification is Y-up. Blender export rotates Z-up to Y-up; invert ONCE.
export const gltfToWorld = () => new Matrix4().makeRotationX(Math.PI / 2);
// Verified world → map SE(2) registration: map = R(yaw)·world + (x, y). Routes arrive in
// the Nav2 map frame; the scene is Gazebo world, so invert explicitly before rendering.
export interface Registration { x: number; y: number; yaw: number }
export function mapToWorld(r: Registration, x: number, y: number): [number, number] {
  const dx = x - r.x, dy = y - r.y, c = Math.cos(r.yaw), s = Math.sin(r.yaw);
  return [c * dx + s * dy, -s * dx + c * dy];
}
export function worldToMap(r: Registration, x: number, y: number): [number, number] {
  const c = Math.cos(r.yaw), s = Math.sin(r.yaw);
  return [c * x - s * y + r.x, s * x + c * y + r.y];
}
