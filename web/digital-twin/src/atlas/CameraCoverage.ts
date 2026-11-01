import { BufferGeometry, DoubleSide, Float32BufferAttribute, Group, LineBasicMaterial, LineSegments, Mesh, MeshBasicMaterial } from 'three';
import { rosToGl } from './coords';
import type { IndustrialCamera } from './scene';

/** Camera axes in the world frame; ``pitch`` is the depression angle (positive looks down). */
export function cameraAxes(cam: IndustrialCamera) {
  const p = cam.pitch, c = Math.cos(cam.yaw), s = Math.sin(cam.yaw);
  return { forward: [Math.cos(p) * c, Math.cos(p) * s, -Math.sin(p)], right: [s, -c, 0], up: [Math.sin(p) * c, Math.sin(p) * s, Math.cos(p)] };
}

/** Unit ray (world frame) through normalised image coordinates u (right), v (up) ∈ [-1, 1]. */
function ray(cam: IndustrialCamera, u: number, v: number): [number, number, number] {
  const { forward: f, right: r, up } = cameraAxes(cam);
  const a = Math.tan(cam.horizontal_fov / 2) * u, b = Math.tan(cam.vertical_fov / 2) * v;
  const d = [0, 1, 2].map(i => f[i]! + r[i]! * a + up[i]! * b);
  const n = Math.hypot(d[0]!, d[1]!, d[2]!);
  return [d[0]! / n, d[1]! / n, d[2]! / n];
}

/** Floor footprint (z = 0) of the camera's view cone clipped to its range: the geometric coverage. */
export function coverage(cam: IndustrialCamera, steps = 12): [number, number][] {
  const [px, py, pz] = cam.position;
  const border: [number, number][] = [];
  for (let i = 0; i < steps; i++) border.push([-1 + 2 * i / steps, -1]);
  for (let i = 0; i < steps; i++) border.push([1, -1 + 2 * i / steps]);
  for (let i = 0; i < steps; i++) border.push([1 - 2 * i / steps, 1]);
  for (let i = 0; i < steps; i++) border.push([-1, 1 - 2 * i / steps]);
  const out: [number, number][] = [];
  for (const [u, v] of border) {
    const [dx, dy, dz] = ray(cam, u, v);
    let t = dz < -1e-6 ? pz / -dz : cam.range;
    if (Math.hypot(dx * t, dy * t) > cam.range) t = cam.range / Math.max(Math.hypot(dx, dy), 1e-6);
    out.push([px + dx * t, py + dy * t]);
  }
  return out;
}

export class CameraCoverage {
  readonly group = new Group();
  private items = new Map<string, { fill: Mesh; lines: LineSegments; frustum: LineSegments }>();
  private readonly fill = new MeshBasicMaterial({ color: '#c79bff', transparent: true, opacity: .13, depthWrite: false, side: DoubleSide });
  private readonly fillSelected = new MeshBasicMaterial({ color: '#ffd07e', transparent: true, opacity: .2, depthWrite: false, side: DoubleSide });
  private readonly edge = new LineBasicMaterial({ color: '#c79bff', transparent: true, opacity: .5 });
  private readonly edgeSelected = new LineBasicMaterial({ color: '#ffd07e' });
  selected: string | null = null;

  build(cameras: IndustrialCamera[]) {
    this.dispose();
    for (const cam of cameras) {
      const poly = coverage(cam);
      const [cx, cy] = poly.reduce((a, p) => [a[0] + p[0] / poly.length, a[1] + p[1] / poly.length], [0, 0]);
      const verts: number[] = [];
      poly.forEach((p, i) => { const q = poly[(i + 1) % poly.length]!; verts.push(...rosToGl(cx, cy, .035), ...rosToGl(p[0], p[1], .035), ...rosToGl(q[0], q[1], .035)); });
      const fillGeometry = new BufferGeometry(); fillGeometry.setAttribute('position', new Float32BufferAttribute(verts, 3));
      const fill = new Mesh(fillGeometry, this.fill);
      // Floor outline always; the four frustum edges only while the camera is selected.
      const outline: number[] = [], rays: number[] = [];
      const origin = rosToGl(...cam.position);
      for (const k of [0, 12, 24, 36]) rays.push(...origin, ...rosToGl(poly[k]![0], poly[k]![1], .04));
      poly.forEach((p, i) => { const q = poly[(i + 1) % poly.length]!; outline.push(...rosToGl(p[0], p[1], .04), ...rosToGl(q[0], q[1], .04)); });
      const make = (values: number[]) => { const g = new BufferGeometry(); g.setAttribute('position', new Float32BufferAttribute(values, 3)); return new LineSegments(g, this.edge); };
      const segments = make(outline), frustum = make(rays);
      frustum.visible = false;
      fill.renderOrder = 2; segments.renderOrder = 3;
      this.group.add(fill, segments, frustum);
      this.items.set(cam.id, { fill, lines: segments, frustum });
    }
  }
  select(id: string | null) {
    this.selected = id;
    for (const [key, item] of this.items) {
      item.fill.material = key === id ? this.fillSelected : this.fill;
      item.lines.material = key === id ? this.edgeSelected : this.edge;
      item.frustum.material = this.edgeSelected; item.frustum.visible = key === id;
    }
  }
  dispose() {
    for (const item of this.items.values()) { item.fill.geometry.dispose(); item.lines.geometry.dispose(); item.frustum.geometry.dispose(); }
    this.group.clear(); this.items.clear();
  }
}
