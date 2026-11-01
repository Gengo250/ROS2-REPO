// ATLAS orbit camera: yaw/pitch/distance around a target (WebGL axes), eased toward a desired
// state with the prototype's constants; drag to orbit, right or shift-drag to pan, wheel to zoom.
import { rosToGl } from './coords';
import type { View } from './scene';

export interface CameraState { yaw: number; pitch: number; distance: number; target: number[] }

export class OrbitCamera {
  readonly state: CameraState = { yaw: .77, pitch: .75, distance: 204, target: [0, 0, 3] };
  desired: CameraState = { yaw: .77, pitch: .75, distance: 204, target: [0, 0, 3] };
  eye = [0, 0, 0];
  follow: (() => number[] | null) | null = null;
  /** Explicit eye/target override (industrial camera point of view); cleared by any interaction. */
  pov: { eye: number[]; target: number[]; fov: number; id?: string } | null = null;
  private dragging = false; private mode: 'orbit' | 'pan' = 'orbit'; private last: number[] | null = null;
  clickStart: number[] | null = null;

  constructor(readonly element: HTMLElement, readonly onInteract: () => void = () => {}) {
    element.addEventListener('contextmenu', e => e.preventDefault());
    element.addEventListener('pointerdown', e => {
      this.dragging = true; this.mode = (e.button === 2 || e.shiftKey) ? 'pan' : 'orbit';
      this.last = [e.clientX, e.clientY]; this.clickStart = [e.clientX, e.clientY];
      element.setPointerCapture(e.pointerId);
    });
    element.addEventListener('pointerup', () => { this.dragging = false; this.last = null; });
    element.addEventListener('pointermove', e => {
      if (!this.dragging || !this.last) return;
      const dx = e.clientX - this.last[0]!, dy = e.clientY - this.last[1]!;
      if (Math.abs(dx) + Math.abs(dy) > 0) { this.pov = null; this.onInteract(); }
      this.last = [e.clientX, e.clientY];
      const d = this.desired;
      if (this.mode === 'orbit') { d.yaw -= dx * .0045; d.pitch = Math.min(1.46, Math.max(.12, d.pitch + dy * .004)); }
      else { this.follow = null; const k = d.distance * .0023, ca = Math.cos(d.yaw), sa = Math.sin(d.yaw); d.target[0] = d.target[0]! + (dx * ca + dy * sa) * k; d.target[2] = d.target[2]! + (dy * ca - dx * sa) * k; }
    });
    element.addEventListener('wheel', e => { e.preventDefault(); this.pov = null; this.desired.distance = Math.min(390, Math.max(3, this.desired.distance * Math.exp(e.deltaY * .0012))); }, { passive: false });
    let pinch = 0;
    element.addEventListener('touchstart', e => { if (e.touches.length === 2) pinch = Math.hypot(e.touches[0]!.clientX - e.touches[1]!.clientX, e.touches[0]!.clientY - e.touches[1]!.clientY); }, { passive: true });
    element.addEventListener('touchmove', e => { if (e.touches.length === 2) { const d = Math.hypot(e.touches[0]!.clientX - e.touches[1]!.clientX, e.touches[0]!.clientY - e.touches[1]!.clientY); if (pinch) this.desired.distance = Math.min(390, Math.max(3, this.desired.distance * pinch / d)); pinch = d; } }, { passive: true });
  }
  setView(view: View) {
    this.follow = null; this.pov = null;
    const t = rosToGl(view.target[0], view.target[1], view.target[2]);
    this.desired = { yaw: view.yaw, pitch: view.pitch, distance: view.distance, target: t };
  }
  focus(x: number, y: number, distance = 75) { this.pov = null; this.desired.target = rosToGl(x, y, 0); this.desired.distance = distance; }
  update(dt: number) {
    const d = this.desired, s = this.state;
    const followed = this.follow?.();
    if (followed) d.target = followed;
    for (const k of ['yaw', 'pitch', 'distance'] as const) s[k] += (d[k] - s[k]) * (1 - Math.exp(-dt * 5));
    for (let k = 0; k < 3; k++) s.target[k] = s.target[k]! + (d.target[k]! - s.target[k]!) * (1 - Math.exp(-dt * 5));
    this.eye = [s.target[0]! + Math.sin(s.yaw) * Math.cos(s.pitch) * s.distance, s.target[1]! + Math.sin(s.pitch) * s.distance,
      s.target[2]! + Math.cos(s.yaw) * Math.cos(s.pitch) * s.distance];
  }
}
