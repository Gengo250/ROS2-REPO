import { Object3D, Quaternion, Vector3 } from 'three';
import type { DynamicEntity, Pose, TwinStateV2 } from '../../core/types/twin-state-v2';

type Sample = { entity: DynamicEntity; time: number; wall: number };
/** Bounded interpolation of measured poses only. Never extrapolates. */
export class StateSynchronizer {
  readonly objects = new Map<string, Object3D>();
  readonly buffers = new Map<string, Sample[]>();
  private readonly cursors = new Map<string, number>();
  private session = '';
  private sequence = -1;
  private connected = false;
  private readonly position = new Vector3();
  private readonly quaternion = new Quaternion();
  readonly delayMs = 120;
  rendered: Record<string, { time: number; source_time: number; from_time: number; to_time: number; alpha: number; position: number[]; quaternion: number[]; status: string }> = {};

  register(id: string, object: Object3D) { this.objects.set(id, object); }
  clear() { this.objects.clear(); this.buffers.clear(); this.cursors.clear(); this.rendered = {}; this.session = ''; this.sequence = -1; }
  disconnect() { this.connected = false; this.buffers.clear(); this.cursors.clear(); }
  accept(state: TwinStateV2, wall = performance.now()) {
    const session = `${state.session_id}/${state.simulation.epoch}/${state.world.frame_id}`;
    if (session !== this.session) { this.buffers.clear(); this.cursors.clear(); this.sequence = -1; this.session = session; }
    if (state.sequence <= this.sequence) return;
    this.sequence = state.sequence; this.connected = true;
    const entities = [...state.simulation_robots, ...state.humans, ...state.obstacles];
    const present = new Set(entities.map(entity => entity.id));
    for (const id of this.buffers.keys()) if (!present.has(id)) {
      this.buffers.delete(id); this.cursors.delete(id);
      if (this.rendered[id]) this.rendered[id]!.status = 'offline';
    }
    for (const entity of entities) {
      if (entity.status !== 'online' || !entity.pose || entity.source_timestamp === null || entity.frame_id !== state.world.frame_id) {
        this.buffers.delete(entity.id);
        this.cursors.delete(entity.id);
        if (this.rendered[entity.id]) this.rendered[entity.id]!.status = entity.status;
        continue;
      }
      let samples = this.buffers.get(entity.id) ?? [];
      const last = samples.at(-1);
      if (last && entity.source_timestamp === last.time) continue;
      if (last && (entity.source_timestamp < last.time || entity.frame_id !== last.entity.frame_id)) {
        samples = []; this.cursors.delete(entity.id);
      }
      samples.push({ entity, time: entity.source_timestamp, wall });
      if (samples.length > 20) samples.shift();
      this.buffers.set(entity.id, samples);
    }
  }
  private apply(object: Object3D | undefined, a: Pose, b: Pose, alpha: number) {
    if (!object) return;
    object.position.set(a.position.x, a.position.y, a.position.z);
    this.position.set(b.position.x, b.position.y, b.position.z);
    object.position.lerp(this.position, alpha);
    object.quaternion.set(a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w);
    this.quaternion.set(b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w);
    object.quaternion.slerp(this.quaternion, alpha);
  }
  update(wall = performance.now()) {
    if (!this.connected) return;
    for (const [id, samples] of this.buffers) {
      const latest = samples.at(-1)!;
      const previous = samples.at(-2) ?? latest;
      const rate = latest.wall > previous.wall ? (latest.time - previous.time) / (latest.wall - previous.wall) : 0;
      // Arrival jitter may change the observed rate abruptly. Never rewind a
      // rendered timeline inside an epoch; wait for real samples to catch up.
      const target = Math.min(latest.time, Math.max(this.cursors.get(id) ?? samples[0]!.time,
        latest.time + (wall - latest.wall - this.delayMs) * rate));
      this.cursors.set(id, target);
      let a = samples[0]!, b = a;
      for (const sample of samples) {
        b = sample;
        if (sample.time >= target) break;
        a = sample;
      }
      const alpha = b.time > a.time ? Math.max(0, Math.min(1, (target-a.time)/(b.time-a.time))) : 1;
      const object = this.objects.get(id);
      this.apply(object, a.entity.pose!, b.entity.pose!, alpha);
      const before = new Map((a.entity.components ?? []).map(c => [c.id, c]));
      for (const component of b.entity.components ?? []) {
        const old = before.get(component.id);
        if (old && old.frame_id === component.frame_id)
          this.apply(this.objects.get(component.id), old.pose, component.pose, alpha);
      }
      if (object) this.rendered[id] = { time: a.time + (b.time-a.time)*alpha, source_time: latest.time, from_time: a.time, to_time: b.time, alpha,
        position: object.position.toArray(), quaternion: object.quaternion.toArray(), status: latest.entity.status };
    }
  }
}
