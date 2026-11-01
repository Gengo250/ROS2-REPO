import { readFileSync } from 'node:fs';
import { Quaternion, Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { cameraAxes, fitViewport, industrialOptics, industrialPose, lookPose, onboardOptics, onboardPose, type MountedCamera, type Pose } from '../src/atlas/cameraPoses';
import { rosToGl } from '../src/atlas/coords';
import { ViewController } from '../src/atlas/ViewController';
import type { IndustrialCamera } from '../src/atlas/scene';

// The onboard camera exactly as the browser loads it (exported from the URDF).
const mount = (JSON.parse(readFileSync(new URL('../public/robots/amr.cameras.json', import.meta.url), 'utf8')) as { cameras: MountedCamera[] }).cameras[0]!;
const scene = JSON.parse(readFileSync(new URL('../public/scene/atlas_campus.scene.json', import.meta.url), 'utf8')) as { cameras: IndustrialCamera[] };

/** Camera axes in WebGL from a pose: forward (−Z), up (+Y), right (+X). */
function axes(p: Pose) {
  const q = new Quaternion(...(p.quaternion as [number, number, number, number]));
  return { forward: new Vector3(0, 0, -1).applyQuaternion(q), up: new Vector3(0, 1, 0).applyQuaternion(q), right: new Vector3(1, 0, 0).applyQuaternion(q) };
}
const near = (v: Vector3, x: number[]) => [v.x, v.y, v.z].forEach((c, i) => expect(c).toBeCloseTo(x[i]!, 9));

describe('onboard camera (URDF camera_optical_frame)', () => {
  it('sits at the lens and looks along the robot heading, upright', () => {
    for (const yaw of [0, Math.PI / 2, -2.3]) {
      const pose = onboardPose({ x: 2, y: -1, z: 0, yaw }, mount), a = axes(pose);
      const lens = rosToGl(2 + Math.cos(yaw) * mount.position[0]!, -1 + Math.sin(yaw) * mount.position[0]!, mount.position[2]!);
      pose.position.forEach((v, i) => expect(v).toBeCloseTo(lens[i]!, 9));
      near(a.forward, rosToGl(Math.cos(yaw), Math.sin(yaw), 0));              // optical +z = heading
      near(a.up, [0, 1, 0]);                                                     // optical −y = up
      near(a.right, rosToGl(Math.sin(yaw), -Math.cos(yaw), 0));                // optical +x = right of the heading
    }
  });
  it('uses the sensor optics: 75° horizontal, 640×480 square pixels', () => {
    const o = onboardOptics(mount);
    expect(o.aspect).toBeCloseTo(4 / 3, 9);
    expect(Math.tan(o.fov * Math.PI / 360) * o.aspect).toBeCloseTo(Math.tan(mount.horizontal_fov / 2), 9);
    expect(o.near).toBe(.08);
  });
});

describe('industrial cameras', () => {
  it('look along the configured yaw and depression with the configured aperture', () => {
    for (const cam of scene.cameras) {
      const pose = industrialPose(cam), a = axes(pose), f = cameraAxes(cam).forward;
      pose.position.forEach((v, i) => expect(v).toBeCloseTo(rosToGl(...cam.position)[i]!, 9));
      near(a.forward, rosToGl(f[0]!, f[1]!, f[2]!));
      expect(a.up.y).toBeGreaterThan(0); expect(a.right.y).toBeCloseTo(0, 9);  // no roll
      const o = industrialOptics(cam);
      expect(o.fov).toBeCloseTo(cam.vertical_fov * 180 / Math.PI, 9);
      expect(Math.tan(o.fov * Math.PI / 360) * o.aspect).toBeCloseTo(Math.tan(cam.horizontal_fov / 2), 9);
    }
  });
});

describe('framing and transitions', () => {
  it('frames the sensor aspect inside the stage', () => {
    const r = fitViewport({ x: 300, y: 100, width: 1000, height: 600 }, 4 / 3);
    expect(r).toEqual({ x: 400, y: 100, width: 800, height: 600 });
  });
  it('blends into a first-person view, then holds the lens exactly; look-around is explicit and reversible', () => {
    const view = new ViewController(), canvas = { x: 0, y: 0, width: 1600, height: 900 };
    const overview = { pose: lookPose([0, 50, 80], [0, 0, 0]), optics: { fov: 47, aspect: 16 / 9, near: .2, far: 700 }, viewport: null };
    view.frame(overview, 0, canvas);
    view.enter('onboard', 'amr_01', 1);
    const target = { pose: onboardPose({ x: 0, y: 0, z: 0, yaw: .4 }, mount), optics: onboardOptics(mount), viewport: fitViewport(canvas, 4 / 3) };
    const halfway = view.frame(target, 1.3, canvas);
    expect(halfway.position[1]).toBeGreaterThan(target.pose.position[1]! + 1);   // still on its way down
    const exact = view.frame(target, 1.7, canvas);
    expect(exact.position).toEqual(target.pose.position); expect(exact.quaternion).toEqual(target.pose.quaternion);
    expect(exact.viewport).toEqual(target.viewport);
    view.drag(200, 0);                                                            // not enabled: ignored
    expect(view.frame(target, 2, canvas).quaternion).toEqual(target.pose.quaternion);
    view.setLook(true); view.drag(200, -50);
    expect(view.frame(target, 2.1, canvas).quaternion).not.toEqual(target.pose.quaternion);
    expect(view.frame(target, 2.1, canvas).position).toEqual(target.pose.position);   // only the gaze turns
    view.setLook(false);
    expect(view.frame(target, 2.2, canvas).quaternion).toEqual(target.pose.quaternion);
  });
});
