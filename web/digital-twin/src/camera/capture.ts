// Synthetic onboard-camera capture: RGB images of the twin as seen from an AMR's camera, plus an
// instance-ID image of the same frame for automatic ground truth.
//
// It reuses the twin's own renderer, scene and drawing code (InstancedRenderer + ATLAS shaders,
// buildStatic, LiveScene and the ATLAS rigs) and the URDF camera mount (cameraPoses.onboardPose,
// REP 103 optical frame): no second scene implementation. Only physical things are drawn: no
// safety fields, routes, selection highlight, placement preview, labels or HUD; the capturing robot
// itself is not drawn (its lens sits inside its own body).
import {
  GLSL3, InstancedBufferAttribute, Mesh, NearestFilter, RawShaderMaterial, RGBAFormat, UnsignedByteType,
  WebGLRenderTarget,
} from 'three';
import { DynamicBatch, InstancedRenderer, SHAPES, VERTEX, type Shape } from '../atlas/render/InstancedRenderer';
import { buildStatic, type CampusScene } from '../atlas/scene';
import { onboardOptics, onboardPose, VIEW_FAR, type MountedCamera } from '../atlas/cameraPoses';
import { LiveScene } from '../atlas/live/LiveScene';
import type { RenderedEntity } from '../atlas/live/LiveWorld';
import { boxDyn, drawLooseObject } from '../atlas/rig/atlasRig.js';
import { rosToGl } from '../atlas/coords';
import type { WorldEntity, WorldObstacle } from '../core/types/world-state';
import { staticInstances, type LabelledInstance } from './annotation';

/** Kinds reported for labelled pixels; the dataset maps them to class ids (vision/configs/dataset.yaml). */
export type ObjectKind = 'person' | 'forklift' | 'amr' | 'pallet' | 'box' | 'cone';
export interface CaptureState { entities: WorldEntity[]; obstacles: WorldObstacle[]; sim_time: number }
export interface CaptureOptions { width: number; height: number; night: boolean; ids: boolean; dt?: number }
export interface InstanceInfo { code: number; id: string; kind: ObjectKind }
export interface Capture {
  width: number; height: number;
  /** RGBA, rows bottom-up (WebGL order). */
  rgb: Uint8Array;
  /** RGBA instance codes (r | g << 8 | b << 16; 0 = background), rows bottom-up; null without ids. */
  ids: Uint8Array | null;
  instances: InstanceInfo[];
  camera: { position: number[]; quaternion: number[]; fov_vertical_deg: number; horizontal_fov_rad: number; near: number; far: number };
  base: { x: number; y: number; z: number; yaw: number };
  render_ms: number;
}

const ID_FRAGMENT = `precision highp float;
flat in vec3 vId; out vec4 fragColor;
void main(){fragColor=vec4(vId,1.);}`;
// The twin's instance vertex shader plus one flat per-instance code (exact 8-bit channels).
const ID_VERTEX = VERTEX.replace('out vec3 vNormal;', 'in vec3 iIdColor; flat out vec3 vId; out vec3 vNormal;')
  .replace('void main(){', 'void main(){vId=iIdColor;');
const SKY_DAY = [.075, .129, .185], SKY_NIGHT = [.034, .065, .108];   // as the twin (main.ts)
const code = (n: number) => [(n & 255) / 255, ((n >> 8) & 255) / 255, ((n >> 16) & 255) / 255];

export class CaptureRenderer {
  readonly instanced: InstancedRenderer;
  private readonly batch = new DynamicBatch();
  private readonly live: LiveScene;
  private readonly idMaterial: RawShaderMaterial;
  private readonly idRecessed: RawShaderMaterial;
  private readonly targets = new Map<string, WebGLRenderTarget>();
  private readonly dynamicIds: Record<Shape, Float32Array> = { cube: new Float32Array(0), cylinder: new Float32Array(0), sphere: new Float32Array(0) };
  private readonly staticLabels: LabelledInstance[];
  readonly staticInstanceCount: number;

  constructor(canvas: HTMLCanvasElement, readonly scene: CampusScene, readonly mount: MountedCamera) {
    this.instanced = new InstancedRenderer(canvas);
    this.instanced.resize(canvas.width, canvas.height, 1);
    this.staticInstanceCount = buildStatic(this.instanced, scene);
    // Every wall and the roof are physical: drawn closed, as a camera inside the hall sees them.
    for (const layer of this.instanced.layers.keys()) this.instanced.setLayerVisible(layer, true);
    this.live = new LiveScene(scene);
    const uniforms = { uProjectionView: this.instanced.material.uniforms.uProjectionView! };
    this.idMaterial = new RawShaderMaterial({ vertexShader: ID_VERTEX, fragmentShader: ID_FRAGMENT, glslVersion: GLSL3, side: this.instanced.material.side, uniforms });
    this.idRecessed = this.idMaterial.clone();
    this.idRecessed.uniforms = this.idMaterial.uniforms;
    Object.assign(this.idRecessed, { polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 2 });
    this.staticLabels = staticInstances(scene);
    this.attachStaticIds();
  }

  /** Per-instance codes of the static layers: labelled scene objects (floor pallets, boxes on the
   * inspection tables) get their object's code; everything else is background (0). */
  private attachStaticIds() {
    const byLayer = new Map<string, Map<Shape, Map<number, number>>>();
    this.staticLabels.forEach((l, i) => {
      const shapes = byLayer.get(l.layer) ?? new Map(); byLayer.set(l.layer, shapes);
      const map = shapes.get(l.shape) ?? new Map(); shapes.set(l.shape, map);
      for (const index of l.instances) map.set(index, i + 1);
    });
    for (const [name, list] of this.instanced.layers) {
      for (const layer of list) {
        const shape = layer.shape;
        const n = layer.geometry.instanceCount, data = new Float32Array(n * 3);
        const labelled = byLayer.get(name)?.get(shape);
        if (labelled) for (const [index, objectIndex] of labelled) data.set(code(objectIndex), index * 3);
        layer.geometry.setAttribute('iIdColor', new InstancedBufferAttribute(data, 3));
      }
    }
  }

  private target(key: string, width: number, height: number, samples: number) {
    let t = this.targets.get(key);
    if (!t || t.width !== width || t.height !== height) {
      t?.dispose();
      t = new WebGLRenderTarget(width, height, { samples, format: RGBAFormat, type: UnsignedByteType, minFilter: NearestFilter,
        magFilter: NearestFilter, depthBuffer: true, generateMipmaps: false });
      this.targets.set(key, t);
    }
    return t;
  }

  /** Render the camera of ``robotId`` for one backend world snapshot. */
  capture(state: CaptureState, robotId: string, opts: CaptureOptions): Capture {
    const started = performance.now();
    const robot = state.entities.find(e => e.id === robotId && e.type === 'amr');
    if (!robot) throw new Error(`robot ${robotId} is not in the world state`);
    const base = { x: robot.x, y: robot.y, z: robot.z, yaw: robot.yaw };
    const mount = { ...this.mount, resolution: [opts.width, opts.height] as [number, number] };
    const pose = onboardPose(base, mount), optics = onboardOptics(mount);
    const eye = pose.position;
    // ------------------------------------------------------------- dynamic bodies, one code each
    const b = this.batch; b.reset();
    const instances: InstanceInfo[] = this.staticLabels.map((l, i) => ({ code: i + 1, id: l.id, kind: l.kind }));
    const ranges: { code: number; from: Record<Shape, number>; to: Record<Shape, number> }[] = [];
    const counts = (): Record<Shape, number> => ({ cube: b.cube.count, cylinder: b.cylinder.count, sphere: b.sphere.count });
    const routes = new Map();
    const draw = (id: string, kind: ObjectKind, paint: () => void) => {
      const from = counts(); paint(); const to = counts();
      const c = instances.length + 1; instances.push({ code: c, id, kind }); ranges.push({ code: c, from, to });
    };
    for (const e of state.entities) {
      if (e.id === robotId) continue;              // the capturing robot's own body
      const kind: ObjectKind = e.type === 'human' ? 'person' : e.type;
      const rendered: RenderedEntity = { ...e, stale: false };
      draw(e.id, kind, () => this.live.draw(b, [rendered], [], routes, { people: true, vehicles: true, fields: false, routes: false,
        selected: null, eye, time: state.sim_time, onboard: robotId, dt: opts.dt ?? 0 }));
    }
    for (const o of state.obstacles)
      draw(o.id, o.kind, () => drawLooseObject(b, { type: o.kind, x: o.x, z: -o.y, yaw: o.yaw, w: o.length, d: o.width, tip: 0 }));
    // Installed industrial cameras (physical housings, background), as in the twin.
    for (const c of this.scene.cameras) {
      const [x, y, z] = rosToGl(...c.position), dx = Math.cos(c.yaw), dz = -Math.sin(c.yaw);
      boxDyn(b, x - dx * .25, y, z - dz * .25, .34, .2, .22, '#c79bff', c.yaw, .5);
      boxDyn(b, x - dx * .04, y - .02, z - dz * .04, .08, .14, .14, '#1b2129', c.yaw, 0);
    }
    this.instanced.uploadDynamic(b);
    for (const shape of SHAPES) {
      const layer = this.instanced.dynamic[shape], n = b[shape].count;
      if (this.dynamicIds[shape].length < b[shape].data.length / 13 * 3) this.dynamicIds[shape] = new Float32Array(b[shape].data.length / 13 * 3);
      const ids = this.dynamicIds[shape]; ids.fill(0, 0, n * 3);
      for (const r of ranges) { const c = code(r.code); for (let i = r.from[shape]; i < r.to[shape]; i++) ids.set(c, i * 3); }
      const attr = layer.geometry.getAttribute('iIdColor') as InstancedBufferAttribute | undefined;
      if (!attr || attr.array !== ids) layer.geometry.setAttribute('iIdColor', new InstancedBufferAttribute(ids, 3));
      else { attr.clearUpdateRanges(); attr.addUpdateRange(0, n * 3); attr.needsUpdate = true; }
    }
    // ------------------------------------------------------------- RGB pass (antialiased, like the twin)
    const three = this.instanced.renderer, view = { position: pose.position, quaternion: pose.quaternion, fov: optics.fov,
      aspect: opts.width / opts.height, near: optics.near, far: VIEW_FAR, viewport: null };
    // InstancedRenderer.render sets the viewport from its drawing-buffer size: keep it the image size.
    if (this.instanced.renderer.domElement.width !== opts.width || this.instanced.renderer.domElement.height !== opts.height)
      this.instanced.resize(opts.width, opts.height, 1);
    const rgbTarget = this.target('rgb', opts.width, opts.height, 4);
    three.setRenderTarget(rgbTarget);
    this.instanced.render(view, opts.night ? SKY_NIGHT : SKY_DAY, opts.night ? 1 : 0, 0);
    const rgb = new Uint8Array(opts.width * opts.height * 4);
    three.readRenderTargetPixels(rgbTarget, 0, 0, opts.width, opts.height, rgb);
    // ------------------------------------------------------------- instance-ID pass (no AA, same camera and geometry)
    let ids: Uint8Array | null = null;
    if (opts.ids) {
      const idTarget = this.target('ids', opts.width, opts.height, 0);
      const swapped: [Mesh, RawShaderMaterial][] = [];
      this.instanced.scene.traverse(o => {
        const m = o as Mesh;
        if (!m.isMesh) return;
        const material = m.material as RawShaderMaterial;
        swapped.push([m, material]);
        m.material = material === this.instanced.recessedMaterial ? this.idRecessed : this.idMaterial;
      });
      three.setRenderTarget(idTarget); three.setClearColor(0x000000, 1); three.clear();
      three.render(this.instanced.scene, this.instanced.camera);
      for (const [m, material] of swapped) m.material = material;
      ids = new Uint8Array(opts.width * opts.height * 4);
      three.readRenderTargetPixels(idTarget, 0, 0, opts.width, opts.height, ids);
    }
    three.setRenderTarget(null);
    return { width: opts.width, height: opts.height, rgb, ids, instances, base, render_ms: performance.now() - started,
      camera: { position: pose.position, quaternion: pose.quaternion, fov_vertical_deg: optics.fov, horizontal_fov_rad: this.mount.horizontal_fov,
        near: optics.near, far: VIEW_FAR } };
  }
}
