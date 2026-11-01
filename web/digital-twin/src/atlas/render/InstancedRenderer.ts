import {
  BufferGeometry, DoubleSide, DynamicDrawUsage, Float32BufferAttribute, GLSL3, Group, InstancedBufferGeometry,
  InstancedInterleavedBuffer, InterleavedBufferAttribute, Matrix4, Mesh, PerspectiveCamera, RawShaderMaterial,
  Scene, StaticDrawUsage, Uint16BufferAttribute, Vector3, WebGLRenderer,
} from 'three';

// ATLAS shaders, unchanged lighting: per-instance position, scale, colour, emission and
// yaw/tilt/roll; one directional light, a warm fill, emission boosted at night, distance fog.
const VERTEX = `precision highp float;
in vec3 position; in vec3 normal;
in vec3 iPosition; in vec3 iScale; in vec3 iColor; in float iEmission; in float iYaw; in vec2 iRot;
uniform mat4 uProjectionView;
out vec3 vNormal; out vec3 vWorld; out vec3 vColor; out float vEmission;
vec3 orient(vec3 p){float c=cos(iRot.y),s=sin(iRot.y);p=vec3(p.x,c*p.y-s*p.z,s*p.y+c*p.z);c=cos(iRot.x);s=sin(iRot.x);p=vec3(c*p.x-s*p.y,s*p.x+c*p.y,p.z);c=cos(iYaw);s=sin(iYaw);return vec3(c*p.x+s*p.z,p.y,-s*p.x+c*p.z);}
void main(){vec3 p=orient(position*iScale)+iPosition;vNormal=normalize(orient(normal));vWorld=p;vColor=iColor;vEmission=iEmission;gl_Position=uProjectionView*vec4(p,1.0);}`;
const FRAGMENT = `precision highp float;
in vec3 vNormal; in vec3 vWorld; in vec3 vColor; in float vEmission;
uniform vec3 uEye; uniform vec3 uFogColor; uniform float uNight; uniform float uDim;
out vec4 fragColor;
void main(){vec3 n=normalize(vNormal);vec3 light=normalize(vec3(-.43,.85,.53));float dif=max(dot(n,light),0.);float shade=(.38+.58*dif)*(1.-.32*uNight);float warm=max(dot(n,normalize(vec3(.3,.7,-.8))),0.)*.16;vec3 color=vColor*(shade+warm);color+=vColor*vEmission*(.65+.5*uNight);float distance=length(uEye-vWorld);float fog=clamp((distance-100.)/230.,0.,.43);color=mix(color,uFogColor,fog);color=mix(color,vec3(dot(color,vec3(.3,.59,.11))),uDim);fragColor=vec4(pow(max(color,vec3(0.)),vec3(.88)),1.);}`;

export type Shape = 'cube' | 'cylinder' | 'sphere';
export const SHAPES: Shape[] = ['cube', 'cylinder', 'sphere'];
export const STRIDE = 13;

function geometryOf(shape: Shape): BufferGeometry {
  const P: number[] = [], N: number[] = [], I: number[] = [];
  if (shape === 'cube') {
    const faces: [number[], number[][]][] = [[[0,0,1],[[-.5,-.5,.5],[.5,-.5,.5],[.5,.5,.5],[-.5,.5,.5]]],[[0,0,-1],[[.5,-.5,-.5],[-.5,-.5,-.5],[-.5,.5,-.5],[.5,.5,-.5]]],[[1,0,0],[[.5,-.5,.5],[.5,-.5,-.5],[.5,.5,-.5],[.5,.5,.5]]],[[-1,0,0],[[-.5,-.5,-.5],[-.5,-.5,.5],[-.5,.5,.5],[-.5,.5,-.5]]],[[0,1,0],[[-.5,.5,.5],[.5,.5,.5],[.5,.5,-.5],[-.5,.5,-.5]]],[[0,-1,0],[[-.5,-.5,-.5],[.5,-.5,-.5],[.5,-.5,.5],[-.5,-.5,.5]]]];
    for (const [n, p] of faces) { const st = P.length / 3; for (const v of p) { P.push(...v); N.push(...n); } I.push(st, st + 1, st + 2, st, st + 2, st + 3); }
  } else if (shape === 'cylinder') {
    const segments = 12;
    for (let i = 0; i < segments; i++) {
      const a = i * 2 * Math.PI / segments, b = (i + 1) * 2 * Math.PI / segments, start = P.length / 3;
      const p = [[Math.cos(a), -.5, Math.sin(a)], [Math.cos(b), -.5, Math.sin(b)], [Math.cos(b), .5, Math.sin(b)], [Math.cos(a), .5, Math.sin(a)]];
      p.forEach((v, j) => { P.push(...v); N.push(j === 0 || j === 3 ? Math.cos(a) : Math.cos(b), 0, j === 0 || j === 3 ? Math.sin(a) : Math.sin(b)); });
      I.push(start, start + 1, start + 2, start, start + 2, start + 3);
      for (const top of [-.5, .5]) {
        const st = P.length / 3;
        P.push(0, top, 0, Math.cos(a), top, Math.sin(a), Math.cos(b), top, Math.sin(b));
        for (let j = 0; j < 3; j++) N.push(0, top > 0 ? 1 : -1, 0);
        if (top > 0) I.push(st, st + 1, st + 2); else I.push(st, st + 2, st + 1);
      }
    }
  } else {
    const lat = 10, lon = 14;
    for (let i = 0; i <= lat; i++) { const v = i / lat * Math.PI; for (let j = 0; j <= lon; j++) { const a = j / lon * 2 * Math.PI; const x = Math.sin(v) * Math.cos(a), y = Math.cos(v), z = Math.sin(v) * Math.sin(a); P.push(x, y, z); N.push(x, y, z); } }
    for (let i = 0; i < lat; i++) for (let j = 0; j < lon; j++) { const a = i * (lon + 1) + j, b = a + lon + 1; I.push(a, a + 1, b, a + 1, b + 1, b); }
  }
  const g = new BufferGeometry();
  g.setAttribute('position', new Float32BufferAttribute(P, 3));
  g.setAttribute('normal', new Float32BufferAttribute(N, 3));
  g.setIndex(new Uint16BufferAttribute(I, 1));
  return g;
}

/** Growable typed-array batch with the prototype's 13-argument push. No per-frame allocation. */
export class PrimitiveBatch {
  data = new Float32Array(STRIDE * 1024);
  count = 0;
  push(x: number, y: number, z: number, sx: number, sy: number, sz: number, r: number, g: number, b: number, e: number, yaw: number, tilt: number, roll: number) {
    if ((this.count + 1) * STRIDE > this.data.length) { const next = new Float32Array(this.data.length * 2); next.set(this.data); this.data = next; }
    const d = this.data, o = this.count++ * STRIDE;
    d[o] = x; d[o + 1] = y; d[o + 2] = z; d[o + 3] = sx; d[o + 4] = sy; d[o + 5] = sz; d[o + 6] = r; d[o + 7] = g; d[o + 8] = b; d[o + 9] = e; d[o + 10] = yaw; d[o + 11] = tilt; d[o + 12] = roll;
  }
}
export class DynamicBatch {
  cube = new PrimitiveBatch(); cylinder = new PrimitiveBatch(); sphere = new PrimitiveBatch();
  reset() { this.cube.count = this.cylinder.count = this.sphere.count = 0; }
  get total() { return this.cube.count + this.cylinder.count + this.sphere.count; }
}

/** One instanced draw call: a primitive shape with 13-float interleaved instances. */
export class InstancedLayer {
  readonly geometry = new InstancedBufferGeometry();
  readonly mesh: Mesh;
  private buffer: InstancedInterleavedBuffer;
  constructor(shape: Shape, material: RawShaderMaterial, data: Float32Array, dynamic: boolean) {
    const base = geometryOf(shape);
    this.geometry.index = base.index;
    this.geometry.setAttribute('position', base.getAttribute('position'));
    this.geometry.setAttribute('normal', base.getAttribute('normal'));
    this.buffer = this.attach(data, dynamic);
    this.geometry.instanceCount = data.length / STRIDE;
    this.mesh = new Mesh(this.geometry, material);
    this.mesh.frustumCulled = false;
  }
  private attach(data: Float32Array, dynamic: boolean) {
    const buffer = new InstancedInterleavedBuffer(data, STRIDE, 1);
    buffer.setUsage(dynamic ? DynamicDrawUsage : StaticDrawUsage);
    for (const [name, size, offset] of [['iPosition', 3, 0], ['iScale', 3, 3], ['iColor', 3, 6], ['iEmission', 1, 9], ['iYaw', 1, 10], ['iRot', 2, 11]] as const)
      this.geometry.setAttribute(name, new InterleavedBufferAttribute(buffer, size, offset));
    return buffer;
  }
  /** Upload the first ``count`` instances of a dynamic batch (only that range is sent). */
  upload(batch: PrimitiveBatch) {
    if (batch.data.length !== this.buffer.array.length) this.buffer = this.attach(batch.data, true);
    else this.buffer.array.set(batch.data.subarray(0, batch.count * STRIDE));
    this.buffer.clearUpdateRanges();
    this.buffer.addUpdateRange(0, batch.count * STRIDE);
    this.buffer.needsUpdate = true;
    this.geometry.instanceCount = batch.count;
  }
  dispose() { this.geometry.dispose(); }
}

/** What the one camera shows this frame: pose in WebGL axes, optics (vertical FOV in degrees) and the
 * viewport in CSS pixels from the top-left (null: the whole canvas, canvas aspect). */
export interface RenderView {
  position: number[]; quaternion: number[]; fov: number; aspect: number | null; near: number; far: number;
  viewport: { x: number; y: number; width: number; height: number } | null;
}

export class InstancedRenderer {
  readonly renderer: WebGLRenderer;
  readonly scene = new Scene();
  readonly camera = new PerspectiveCamera(47, 1, .2, 700);
  readonly material: RawShaderMaterial;
  readonly staticGroup = new Group();
  readonly overlay = new Group();
  readonly layers = new Map<string, InstancedLayer[]>();
  readonly dynamic: Record<Shape, InstancedLayer>;
  private readonly pv = new Matrix4();
  private size = [1, 1];
  constructor(readonly canvas: HTMLCanvasElement) {
    this.renderer = new WebGLRenderer({ canvas, antialias: true, alpha: false, preserveDrawingBuffer: true, powerPreference: 'high-performance' });
    this.material = new RawShaderMaterial({
      vertexShader: VERTEX, fragmentShader: FRAGMENT, glslVersion: GLSL3, side: DoubleSide,
      uniforms: { uProjectionView: { value: this.pv }, uEye: { value: new Vector3() }, uFogColor: { value: new Vector3() },
        uNight: { value: 0 }, uDim: { value: 0 } },
    });
    this.camera.up.set(0, 1, 0);
    this.scene.add(this.staticGroup, this.overlay);
    this.dynamic = Object.fromEntries(SHAPES.map(shape => {
      const layer = new InstancedLayer(shape, this.material, new Float32Array(STRIDE * 1024), true);
      layer.geometry.instanceCount = 0; this.scene.add(layer.mesh); return [shape, layer];
    })) as Record<Shape, InstancedLayer>;
  }
  /** Static layer from 13-float WebGL instances, grouped by name so it can be toggled. */
  setStaticLayer(name: string, shapes: Partial<Record<Shape, Float32Array>>) {
    for (const old of this.layers.get(name) ?? []) { this.staticGroup.remove(old.mesh); old.dispose(); }
    const list = Object.entries(shapes).filter(([, data]) => data && data.length)
      .map(([shape, data]) => new InstancedLayer(shape as Shape, this.material, data!, false));
    list.forEach(layer => { layer.mesh.name = name; this.staticGroup.add(layer.mesh); });
    this.layers.set(name, list);
  }
  setLayerVisible(name: string, visible: boolean) { for (const layer of this.layers.get(name) ?? []) layer.mesh.visible = visible; }
  clearStatic() { for (const name of [...this.layers.keys()]) this.setStaticLayer(name, {}); this.layers.clear(); }
  uploadDynamic(batch: DynamicBatch) { for (const shape of SHAPES) this.dynamic[shape].upload(batch[shape]); }
  render(view: RenderView, sky: number[], night: number, dim: number) {
    const cam = this.camera, [w, h] = this.size as [number, number], v = view.viewport;
    const aspect = view.aspect ?? (v ? v.width / Math.max(v.height, 1) : w / Math.max(h, 1));
    if (cam.fov !== view.fov || cam.aspect !== aspect || cam.near !== view.near || cam.far !== view.far) {
      cam.fov = view.fov; cam.aspect = aspect; cam.near = view.near; cam.far = view.far; cam.updateProjectionMatrix();
    }
    cam.position.set(view.position[0]!, view.position[1]!, view.position[2]!);
    cam.quaternion.set(view.quaternion[0]!, view.quaternion[1]!, view.quaternion[2]!, view.quaternion[3]!);
    cam.updateMatrixWorld();
    this.pv.multiplyMatrices(cam.projectionMatrix, cam.matrixWorldInverse);
    const u = this.material.uniforms;
    (u.uEye!.value as Vector3).set(view.position[0]!, view.position[1]!, view.position[2]!);
    (u.uFogColor!.value as Vector3).set(sky[0]!, sky[1]!, sky[2]!);
    u.uNight!.value = night; u.uDim!.value = dim;
    const r = this.renderer;
    if (v) {
      // Sensor framing: dark bars around a viewport with the camera's own aspect ratio.
      r.setScissorTest(false); r.setViewport(0, 0, w, h); r.setClearColor(0x04080e, 1); r.clear();
      r.setViewport(v.x, h - v.y - v.height, v.width, v.height); r.setScissor(v.x, h - v.y - v.height, v.width, v.height); r.setScissorTest(true);
    } else { r.setScissorTest(false); r.setViewport(0, 0, w, h); }
    r.setClearColor((sky[0]! * 255) << 16 | (sky[1]! * 255) << 8 | (sky[2]! * 255), 1);
    r.render(this.scene, cam);
  }
  resize(width: number, height: number, ratio: number) {
    this.size = [width, height];
    this.renderer.setPixelRatio(ratio);
    this.renderer.setSize(width, height, false);
  }
  dispose() {
    for (const list of this.layers.values()) list.forEach(l => l.dispose());
    for (const shape of SHAPES) this.dynamic[shape].dispose();
    this.material.dispose(); this.renderer.dispose();
  }
}
