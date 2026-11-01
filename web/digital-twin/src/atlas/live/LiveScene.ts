// Presentation of the backend world with the ATLAS rigs. Nothing here decides behaviour:
// pose, speed, task, carrying and field state are the engine's; the browser only turns them
// into animation (gait phase, task pose blend) and instances.
import { animateGait, drawFieldSamples, drawForklift, drawForkliftMarks, drawHuman, drawHumanFar, drawLooseObject,
  drawRobotTrue, segDyn, cylDyn } from '../rig/atlasRig.js';
import type { DynamicBatch } from '../render/InstancedRenderer';
import type { CampusScene, RoleSpec } from '../scene';
import type { RenderedEntity } from './LiveWorld';
import type { WorldObstacle } from '../../core/types/world-state';

const FIELD_INDEX = { free: 0, warning: 1, protective: 2 } as const;
const AMR = { L: .975, W: .70, cx: .0125 };

interface Animator {
  id: string; role: string; R: RoleSpec; y0: number; x: number; z: number; yaw: number; v: number; look: number;
  phase: number; gaitW: number; A: number; steps: number; carry: boolean; hold: string | null; seed: number;
  skin: string; pants: string; shirt: string; hair: string;
  act: { type: string; t: number; dur: number; ready: boolean; beep?: boolean } | null;
}

/** Field sample poses (WebGL) ahead of a vehicle: along the given world path if any, else straight. */
function samplesAhead(x: number, y: number, yaw: number, length: number, path: [number, number][] | null) {
  const out: { s: number; p: { x: number; z: number; yaw: number } }[] = [];
  if (path && path.length > 1) {
    let best = 0, bestD = Infinity;
    path.forEach((p, i) => { const d = (p[0] - x) ** 2 + (p[1] - y) ** 2; if (d < bestD) { bestD = d; best = i; } });
    if (bestD < 1.5) {
      let s = 0, px = x, py = y, i = best;
      while (s <= length + 1e-6 && i < path.length - 1) {
        const [nx, ny] = path[i + 1]!, seg = Math.hypot(nx - px, ny - py);
        const heading = Math.atan2(ny - py, nx - px);
        for (let k = 0; k * .35 <= seg && s + k * .35 <= length; k++) out.push({ s: s + k * .35, p: { x: px + Math.cos(heading) * k * .35, z: -(py + Math.sin(heading) * k * .35), yaw: heading } });
        s += seg; px = nx; py = ny; i++;
      }
      if (out.length > 1) return out;
    }
  }
  for (let s = 0; s <= length + 1e-6; s += .35) out.push({ s, p: { x: x + Math.cos(yaw) * s, z: -(y + Math.sin(yaw) * s), yaw } });
  return out;
}

export class LiveScene {
  private animators = new Map<string, Animator>();
  private lastFrame = performance.now() / 1000;
  private forklifts: Map<string, { color: string; L: number; W: number; cx: number; rank: number }>;
  constructor(readonly scene: CampusScene) {
    const roles = scene.agents.roles;
    for (const w of scene.agents.workers) {
      const a = w.appearance;
      this.animators.set(w.id, { id: w.id, role: w.role, R: roles[w.role]!, y0: w.level_z, x: w.initial.x, z: -w.initial.y, yaw: w.initial.yaw,
        v: 0, look: 0, phase: a.phase, gaitW: 0, A: .3, steps: 0, carry: w.carry, hold: w.hold, seed: a.seed, skin: a.skin, pants: a.pants,
        shirt: a.shirt, hair: a.hair, act: null });
    }
    this.forklifts = new Map(scene.agents.forklifts.map((f, rank) => [f.id, { color: f.color, L: f.length, W: f.width, cx: f.centre_offset, rank: rank + 8 }]));
  }

  draw(a: DynamicBatch, entities: RenderedEntity[], obstacles: WorldObstacle[], path: [number, number][], goal: number[] | null,
       opts: { people: boolean; vehicles: boolean; fields: boolean; routes: boolean; selected: string | null; eye: number[]; time: number;
         /** Robot seen from its own onboard camera: its field outline would sit on the lens. */ onboard: string | null }) {
    const now = performance.now() / 1000, dt = Math.min(.1, Math.max(0, now - this.lastFrame));
    this.lastFrame = now;
    if (opts.routes && path.length > 1) {
      // Seen from the onboard lens (0.26 m high) the raised strip would fill the image: draw it on the floor.
      const [width, height] = opts.onboard ? [.08, .02] : [.14, .17];
      for (let i = 0; i + 1 < path.length; i++) segDyn(a, path[i]![0], -path[i]![1], path[i + 1]![0], -path[i + 1]![1], width, height, '#53debc', .7);
      if (goal) { cylDyn(a, goal[0]!, .2, -goal[1]!, .32, .04, '#53debc', 0, .9); cylDyn(a, goal[0]!, .7, -goal[1]!, .03, 1.0, '#53debc', 0, 1); }
    }
    for (const e of entities) {
      if (e.type === 'human') {
        if (!opts.people) continue;
        const w = this.animators.get(e.id); if (!w) continue;
        w.x = e.x; w.z = -e.y; w.yaw = e.yaw; w.y0 = e.z; w.v = Math.max(0, e.speed); w.look = e.look; w.carry = e.carrying;
        if (e.activity) {
          const dur = this.scene.agents.acts[e.activity]?.dur ?? 3;
          w.act = { type: e.activity, t: e.activity_progress * dur, dur, ready: e.state === 'working', beep: w.act?.beep };
        } else w.act = null;
        if (!e.stale) animateGait(w, dt, e.wz);
        const far = Math.hypot(opts.eye[0]! - w.x, opts.eye[1]! - w.y0, opts.eye[2]! - w.z) > 70;
        if (far && e.id !== opts.selected) drawHumanFar(a, w); else drawHuman(a, w, opts.time, e.id === opts.selected);
      } else if (e.type === 'amr') {
        const r = { pose: { x: e.x, z: -e.y, yaw: e.yaw }, color: '#53debc', field: FIELD_INDEX[e.field_state], emergT: e.state === 'blocked' ? 1 : 0,
          lift: false, rank: 0, S: AMR, prot: e.field_length };
        drawRobotTrue(a, r, opts.time);
        if (opts.fields && e.id !== opts.onboard) drawFieldSamples(a, r, samplesAhead(e.x, e.y, e.yaw, e.field_length + 1, path));
      } else if (e.type === 'forklift') {
        if (!opts.vehicles) continue;
        const spec = this.forklifts.get(e.id) ?? { color: '#f4b64f', L: 4.55, W: 1.98, cx: .8, rank: 8 };
        const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
        const r = { pose: { x: e.x, z: -e.y, yaw: e.yaw }, color: spec.color, field: FIELD_INDEX[e.field_state], emergT: 0, lift: true,
          rank: spec.rank, S: { L: spec.L, W: spec.W, cx: spec.cx }, prot: e.field_length };
        drawForklift(a, r, opts.time);
        if (opts.fields) { drawForkliftMarks(a, r); drawFieldSamples(a, r, samplesAhead(e.x + c * 0, e.y + s * 0, e.yaw, e.field_length + 2.4, null)); }
      }
    }
    for (const o of obstacles) drawLooseObject(a, { type: o.kind, x: o.x, z: -o.y, yaw: o.yaw, w: o.length, d: o.width, tip: 0 });
  }
}
