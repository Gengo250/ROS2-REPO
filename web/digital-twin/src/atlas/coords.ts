// The one coordinate convention of the twin and its explicit WebGL mapping.
// ROS world:  x east, y north, z up (metres, radians; REP 103).
// WebGL/Three.js (ATLAS): X east, Y up, Z south.  X = x, Y = z, Z = -y.
// Yaw is the same number in both: ATLAS rotates local +X to (cos yaw, 0, -sin yaw), which is
// (cos yaw, sin yaw) in the ROS x/y plane, i.e. a rotation about +Y in Three.js.
export const rosToGl = (x: number, y: number, z = 0): [number, number, number] => [x, z, -y];
export const glToRos = (X: number, Y: number, Z: number): [number, number, number] => [X, -Z, Y];
export const glPlanar = (x: number, y: number) => ({ x, z: -y });

/** ROS-frame scene instances (x, y, z, length, width, height, r, g, b, e, yaw, tilt, roll) → WebGL instances
 * (x, y, z, scale x, scale y, scale z, r, g, b, e, yaw, tilt, roll) as the ATLAS shader expects. */
export function instancesToGl(values: number[]): Float32Array {
  const out = new Float32Array(values.length);
  for (let i = 0; i < values.length; i += 13) {
    out[i] = values[i]!; out[i + 1] = values[i + 2]!; out[i + 2] = -values[i + 1]!;
    out[i + 3] = values[i + 3]!; out[i + 4] = values[i + 5]!; out[i + 5] = values[i + 4]!;
    for (let k = 6; k < 13; k++) out[i + k] = values[i + k]!;
  }
  return out;
}

/** World → map SE(2): map = R(yaw)·world + (x, y); inverse for drawing map-frame data. */
export interface Registration { x: number; y: number; yaw: number }
export function mapToWorld(r: Registration, x: number, y: number): [number, number] {
  const dx = x - r.x, dy = y - r.y, c = Math.cos(r.yaw), s = Math.sin(r.yaw);
  return [c * dx + s * dy, -s * dx + c * dy];
}
