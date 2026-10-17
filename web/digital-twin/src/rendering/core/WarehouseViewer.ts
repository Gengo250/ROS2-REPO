import { AmbientLight, Color, DirectionalLight, Group, PerspectiveCamera, Scene, WebGLRenderer, PCFSoftShadowMap } from 'three';
import Ajv from 'ajv/dist/2020';
import schema from '../../../../../shared/schemas/warehouse-scene.schema.json';
import type { WarehouseScene } from '../../core/types/warehouse-scene';
import { buildWarehouse, setStructureVisible } from '../warehouse/WarehouseBuilder';
import { loadRobot } from '../robots/RobotLoader';
import { CameraController } from '../cameras/CameraController';
import { disposeTree } from './resources';

const validate = new Ajv({ strict: false }).compile<WarehouseScene>(schema);
export class WarehouseViewer {
  readonly renderer: WebGLRenderer;
  readonly scene = new Scene();
  readonly camera = new PerspectiveCamera(45, 1, .05, 200);
  controls?: CameraController;
  warehouse?: Group;
  robot?: Group;
  private abort?: AbortController;
  private disposed = false;
  private frames = 0;
  private last = performance.now();
  private fps = 0;
  private loadMs = 0;
  private readonly resize: ResizeObserver;
  constructor(readonly host: HTMLElement, readonly status: HTMLElement, readonly metrics: HTMLElement, readonly baseUrl: string) {
    this.renderer = new WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.autoUpdate = false;
    this.renderer.shadowMap.type = PCFSoftShadowMap;
    this.scene.background = new Color('#bfcbd3');
    this.scene.add(new AmbientLight(0xffffff, 1.5));
    const sun = new DirectionalLight(0xfff2df, 2.5); sun.position.set(-8, -6, 20); sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -22, right: 22, top: 22, bottom: -22, far: 60 });
    sun.shadow.bias = -.0002; this.scene.add(sun);
    host.append(this.renderer.domElement);
    this.renderer.domElement.setAttribute('aria-label', 'Interactive warehouse 3D view');
    this.resize = new ResizeObserver(() => {
      const { width, height } = host.getBoundingClientRect();
      this.renderer.setSize(width, height, false); this.camera.aspect = width / Math.max(height, 1); this.camera.updateProjectionMatrix();
    });
    this.resize.observe(host);
    this.renderer.setAnimationLoop(() => {
      this.controls?.controls.update(); this.renderer.render(this.scene, this.camera); this.frames++;
      const now = performance.now();
      if (now - this.last > 1000) {
        this.fps = this.frames * 1000 / (now - this.last); this.last = now; this.frames = 0;
        const info = this.renderer.info;
        this.metrics.textContent = `${this.fps.toFixed(0)} FPS · ${info.render.calls} draw calls · ${this.warehouse?.userData.primitiveCount ?? 0} shapes · ${(this.loadMs / 1000).toFixed(2)} s load`;
        this.host.dataset.metrics = JSON.stringify({ fps: this.fps, load_ms: this.loadMs,
          calls: info.render.calls, triangles: info.render.triangles, geometries: info.memory.geometries,
          textures: info.memory.textures, shapes: this.warehouse?.userData.primitiveCount,
          camera: this.camera.position.toArray(), target: this.controls?.controls.target.toArray() });
      }
    });
  }
  async load(scenario?: string) {
    this.abort?.abort(); const abort = new AbortController(); this.abort = abort;
    const started = performance.now(); this.status.textContent = 'Loading warehouse…';
    this.host.dataset.loaded = 'false';
    try {
      const response = await fetch(`${this.baseUrl}/api/v1/scene${scenario ? `?scenario=${encodeURIComponent(scenario)}` : ''}`,
        { signal: AbortSignal.any([abort.signal, AbortSignal.timeout(15000)]) });
      if (!response.ok) throw Error(`Scene request failed (${response.status})`);
      const data: unknown = await response.json();
      if (!validate(data)) throw Error('Warehouse scene schema mismatch');
      if (abort.signal.aborted) return;
      this.clear();
      this.warehouse = buildWarehouse(data); this.scene.add(this.warehouse);
      const spawn = data.robot_spawns.find(s => s.active)!;
      this.controls = new CameraController(this.camera, this.renderer.domElement, data.building.length, spawn.pose);
      this.host.dataset.scenario = data.metadata.scenario;
      this.host.dataset.hash = data.metadata.geometry_sha256;
      this.status.textContent = `${data.metadata.scenario} · Loading original AMR…`;
      const robot = await loadRobot(spawn.pose, abort.signal, data.metadata.source_sha256['amr_dimensions.json']!);
      if (abort.signal.aborted || this.disposed) { disposeTree(robot); return; }
      this.robot = robot; this.scene.add(robot); this.renderer.shadowMap.needsUpdate = true; this.loadMs = performance.now() - started;
      this.host.dataset.loaded = 'true'; this.host.dataset.robotSize = JSON.stringify(robot.userData.size);
      this.status.textContent = `${data.metadata.scenario} · AMR 01 · Configured spawn · World / metres`;
    } catch (error) {
      if (!abort.signal.aborted && !this.disposed) this.status.textContent = `Visualization unavailable: ${(error as Error).message}. Retry to reload.`;
    }
  }
  view(name: string) { this.controls?.view(name); }
  structure(visible: boolean) { if (this.warehouse) setStructureVisible(this.warehouse, visible); this.renderer.shadowMap.needsUpdate = true; }
  private clear() {
    this.controls?.dispose(); this.controls = undefined;
    for (const group of [this.warehouse, this.robot]) if (group) { this.scene.remove(group); disposeTree(group); }
    this.warehouse = this.robot = undefined;
  }
  dispose() {
    if (this.disposed) return;
    this.disposed = true; this.abort?.abort(); this.renderer.setAnimationLoop(null);
    this.resize.disconnect(); this.clear();
    this.scene.traverse(object => { if (object instanceof DirectionalLight) object.shadow.dispose(); });
    this.renderer.dispose(); this.renderer.forceContextLoss(); this.renderer.domElement.remove();
  }
}
