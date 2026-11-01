// View modes of the twin: overview (orbit camera, optionally following an entity), an industrial
// camera's first-person view, or a robot's onboard camera. The controller only places the one
// Three.js camera: target poses come from the scene (fixed cameras) or from the backend-
// authoritative robot state (onboard); nothing here moves or simulates anything.
import { Quaternion, Vector3 } from 'three';
import type { RenderView } from './render/InstancedRenderer';
import type { Optics, Pose, Rect } from './cameraPoses';

export type ViewKind = 'overview' | 'industrial' | 'onboard';
/** Desired camera for this frame; ``viewport`` null = whole canvas (overview). */
export interface ViewTarget { pose: Pose; optics: Optics; viewport: Rect | null }

const BLEND_SECONDS = .6;
const LOOK_YAW = 1.6, LOOK_PITCH = .8;
const LOCAL_UP = new Vector3(0, 1, 0), LOCAL_RIGHT = new Vector3(1, 0, 0);
const ease = (k: number) => k * k * (3 - 2 * k);

export class ViewController {
  kind: ViewKind = 'overview';
  /** Industrial camera id or robot id of the first-person view. */
  id: string | null = null;
  /** Explicit, reversible look-around in first-person views: offsets on top of the sensor
   * orientation (camera-local yaw, then pitch); the sensor's own orientation never changes. */
  readonly look = { enabled: false, yaw: 0, pitch: 0 };
  private from: RenderView | null = null;
  private start = 0;
  private last: RenderView | null = null;

  get firstPerson() { return this.kind !== 'overview'; }

  /** Switch mode; the camera blends from wherever it is now (``now`` in seconds). */
  enter(kind: ViewKind, id: string | null, now: number) {
    if (kind === this.kind && id === this.id) return;
    this.from = this.last; this.start = now;
    this.kind = kind; this.id = kind === 'overview' ? null : id;
    this.setLook(false);
  }
  setLook(enabled: boolean) { this.look.enabled = enabled; this.look.yaw = 0; this.look.pitch = 0; }
  drag(dx: number, dy: number) {
    if (!this.look.enabled) return;
    this.look.yaw = Math.max(-LOOK_YAW, Math.min(LOOK_YAW, this.look.yaw - dx * .004));
    this.look.pitch = Math.max(-LOOK_PITCH, Math.min(LOOK_PITCH, this.look.pitch - dy * .004));
  }
  /** Transition still running (then the pose is a blend, not yet exactly the lens). */
  blending(now: number) { return this.from !== null && now - this.start < BLEND_SECONDS; }

  /** Camera of this frame: the target (with look-around offsets), blended from the previous
   * camera for BLEND_SECONDS after a mode change; exact afterwards. */
  frame(target: ViewTarget, now: number, canvas: Rect): RenderView {
    let q = new Quaternion(...(target.pose.quaternion as [number, number, number, number]));
    if (this.firstPerson && (this.look.yaw || this.look.pitch)) {
      q = q.multiply(new Quaternion().setFromAxisAngle(LOCAL_UP, this.look.yaw))
        .multiply(new Quaternion().setFromAxisAngle(LOCAL_RIGHT, this.look.pitch));
    }
    let view: RenderView = { position: [...target.pose.position], quaternion: [q.x, q.y, q.z, q.w], fov: target.optics.fov,
      aspect: target.viewport ? target.optics.aspect : null, near: target.optics.near, far: target.optics.far, viewport: target.viewport };
    const k = this.from ? Math.min(1, (now - this.start) / BLEND_SECONDS) : 1;
    if (this.from && k < 1) view = blend(this.from, view, ease(k), canvas);
    else this.from = null;
    this.last = view;
    return view;
  }
}

function blend(a: RenderView, b: RenderView, k: number, canvas: Rect): RenderView {
  const qa = new Quaternion(...(a.quaternion as [number, number, number, number]));
  const q = qa.slerp(new Quaternion(...(b.quaternion as [number, number, number, number])), k);
  const ra = a.viewport ?? canvas, rb = b.viewport ?? canvas;
  const viewport = { x: ra.x + (rb.x - ra.x) * k, y: ra.y + (rb.y - ra.y) * k, width: ra.width + (rb.width - ra.width) * k,
    height: ra.height + (rb.height - ra.height) * k };
  return { position: a.position.map((v, i) => v + (b.position[i]! - v) * k), quaternion: [q.x, q.y, q.z, q.w],
    fov: a.fov + (b.fov - a.fov) * k, aspect: viewport.width / Math.max(viewport.height, 1), near: Math.min(a.near, b.near),
    far: Math.max(a.far, b.far), viewport };
}
