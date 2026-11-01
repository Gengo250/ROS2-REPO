import Ajv2020 from 'ajv/dist/2020';
import addFormats from 'ajv-formats';
import worldSchema from '../../../../../shared/schemas/world-state.v1.schema.json';
import commandSchema from '../../../../../shared/schemas/command-result.v1.schema.json';
import type { WorldEntity, WorldEvent, WorldState } from '../../core/types/world-state';
import type { CommandResult } from '../../core/types/command-result';

const ajv = new Ajv2020({ strict: true, allErrors: false });
addFormats(ajv);
const validWorld = ajv.compile<WorldState>(worldSchema);
const validCommand = ajv.compile<CommandResult>(commandSchema);

type Sample = { sim: number; wall: number; entity: WorldEntity };
export type Connection = 'connecting' | 'connected' | 'disconnected';

/** Interpolated entity for one rendered frame (world frame) plus how fresh it is. */
export interface RenderedEntity extends WorldEntity { stale: boolean }

/**
 * Backend-authoritative world. The browser only interpolates between received samples on the
 * simulation clock (never extrapolates, never rewinds) and marks entities stale when samples stop.
 */
export class LiveWorld {
  connection: Connection = 'disconnected';
  error = '';
  latest: WorldState | null = null;
  latencyMs: number | null = null;
  receivedHz = 0;
  readonly delay = .12;
  private socket: WebSocket | null = null;
  private retry: ReturnType<typeof setTimeout> | undefined;
  private watchdog: ReturnType<typeof setTimeout> | undefined;
  private attempts = 0;
  private stopped = true;
  private session = '';
  private engine = '';
  private buffers = new Map<string, Sample[]>();
  private cursor: number | null = null;
  /** Simulation time the entities are interpolated at (null before the first samples). */
  get renderTime() { return this.cursor; }
  private rate = 1;
  private arrivals: number[] = [];
  private lastEventSeq = 0;
  private listeners = new Set<(events: WorldEvent[]) => void>();
  constructor(readonly api: string, readonly wsUrl: string) {}

  onEvents(listener: (events: WorldEvent[]) => void) { this.listeners.add(listener); return () => this.listeners.delete(listener); }
  start() { if (!this.stopped) return; this.stopped = false; this.connect(); }
  stop() {
    this.stopped = true; clearTimeout(this.retry); this.retry = undefined; this.detach();
    this.connection = 'disconnected';
  }
  private connect() {
    if (this.stopped) return;
    this.connection = 'connecting';
    try {
      const socket = new WebSocket(this.wsUrl);
      this.socket = socket;
      socket.onmessage = event => {
        try { this.accept(JSON.parse(String(event.data))); this.attempts = 0; this.arm(3000); }
        catch (error) { this.fail(`Telemetria inválida: ${(error as Error).message.slice(0, 120)}`); }
      };
      socket.onerror = () => this.fail('Falha de conexão com o Gateway');
      socket.onclose = () => this.fail('Gateway desconectado');
      this.arm(8000);
    } catch { this.fail('Não foi possível conectar ao Gateway'); }
  }
  private arm(ms: number) { clearTimeout(this.watchdog); this.watchdog = setTimeout(() => this.fail('Sem telemetria do Gateway'), ms); }
  private detach() {
    clearTimeout(this.watchdog);
    if (this.socket) { this.socket.onmessage = this.socket.onerror = this.socket.onclose = null; this.socket.close(); this.socket = null; }
  }
  private fail(message: string) {
    this.detach();
    if (this.stopped || this.retry !== undefined) return;
    this.connection = 'disconnected'; this.error = message;
    const delay = Math.min(8000, 400 * 2 ** Math.min(this.attempts++, 5)) * (0.8 + 0.2 * Math.random());
    this.retry = setTimeout(() => { this.retry = undefined; this.connect(); }, delay);
  }

  /** Validate and buffer one snapshot (also used by tests and the REST bootstrap). */
  accept(value: unknown, wall = performance.now() / 1000) {
    if (!validWorld(value)) throw new Error(ajv.errorsText(validWorld.errors));
    const state = value;
    if (this.latest && state.session_id === this.latest.session_id && state.sequence <= this.latest.sequence) return;
    this.connection = 'connected'; this.error = '';
    this.latencyMs = Math.max(0, (Date.now() / 1000 - state.server_wall_time) * 1000);
    this.arrivals.push(wall); while (this.arrivals.length > 40) this.arrivals.shift();
    if (this.arrivals.length > 1) this.receivedHz = (this.arrivals.length - 1) / (this.arrivals.at(-1)! - this.arrivals[0]!);
    const engine = state.simulation.engine_session_id ?? '';
    if (state.session_id !== this.session || engine !== this.engine) {
      this.session = state.session_id; this.engine = engine; this.buffers.clear(); this.cursor = null; this.lastEventSeq = 0;
    }
    this.latest = state;
    const fresh = state.events.filter(e => e.seq > this.lastEventSeq);
    if (fresh.length) { this.lastEventSeq = fresh.at(-1)!.seq; for (const l of this.listeners) l(fresh); }
    const sim = state.simulation.sim_time;
    if (sim === null || state.simulation.availability !== 'online') return;
    const present = new Set(state.entities.map(e => e.id));
    for (const id of [...this.buffers.keys()]) if (!present.has(id)) this.buffers.delete(id);
    for (const entity of state.entities) {
      let samples = this.buffers.get(entity.id) ?? [];
      const last = samples.at(-1);
      if (last && sim <= last.sim) { if (sim < last.sim) samples = []; else continue; }
      samples.push({ sim, wall, entity });
      if (samples.length > 30) samples.shift();
      this.buffers.set(entity.id, samples);
    }
    const all = [...this.buffers.values()][0];
    if (all && all.length > 1) {
      const a = all.at(-2)!, b = all.at(-1)!;
      if (b.wall > a.wall) this.rate += ((b.sim - a.sim) / (b.wall - a.wall) - this.rate) * .2;
    }
  }

  /** Entities interpolated at (simulation time of the newest sample − delay), never extrapolated. */
  entities(wall = performance.now() / 1000): RenderedEntity[] {
    const state = this.latest;
    if (!state) return [];
    const live = this.connection === 'connected' && state.simulation.availability === 'online';
    const any = [...this.buffers.values()][0];
    if (live && any && any.length) {
      const newest = any.at(-1)!;
      const target = Math.min(newest.sim, newest.sim + (wall - newest.wall - this.delay) * Math.max(0, this.rate));
      this.cursor = Math.max(this.cursor ?? target, target);
      this.cursor = Math.min(this.cursor, newest.sim);
    }
    const out: RenderedEntity[] = [];
    for (const entity of state.entities) {
      const samples = this.buffers.get(entity.id);
      if (!live || !samples || this.cursor === null) { out.push({ ...entity, stale: true }); continue; }
      let a = samples[0]!, b = a;
      for (const s of samples) { b = s; if (s.sim >= this.cursor) break; a = s; }
      const k = b.sim > a.sim ? Math.max(0, Math.min(1, (this.cursor - a.sim) / (b.sim - a.sim))) : 1;
      const yaw = a.entity.yaw + Math.atan2(Math.sin(b.entity.yaw - a.entity.yaw), Math.cos(b.entity.yaw - a.entity.yaw)) * k;
      out.push({ ...b.entity, x: a.entity.x + (b.entity.x - a.entity.x) * k, y: a.entity.y + (b.entity.y - a.entity.y) * k,
        yaw, speed: a.entity.speed + (b.entity.speed - a.entity.speed) * k, wz: b.entity.wz, stale: false });
    }
    return out;
  }

  // ---------------------------------------------------------------- commands (backend decides)
  private async command(method: 'POST' | 'DELETE', path: string, body?: unknown): Promise<CommandResult> {
    const response = await fetch(`${this.api}${path}`, { method, headers: body ? { 'Content-Type': 'application/json' } : {},
      body: body ? JSON.stringify(body) : undefined, signal: AbortSignal.timeout(8000) });
    if (response.status === 503) throw new Error((await response.json().catch(() => ({ detail: 'Indisponível' }))).detail ?? 'Indisponível');
    if (response.status === 422) throw new Error('Pedido inválido');
    const data: unknown = await response.json();
    if (!validCommand(data)) throw new Error('Resposta incompatível do Gateway');
    return data;
  }
  addObstacle(kind: string, x: number, y: number, yaw = 0) { return this.command('POST', '/api/v1/obstacles', { kind, x, y, yaw }); }
  removeObstacle(id: string) { return this.command('DELETE', `/api/v1/obstacles/${encodeURIComponent(id)}`); }
  clearObstacles() { return this.command('DELETE', '/api/v1/obstacles'); }
  sendMission(destination: string) { return this.command('POST', '/api/v1/missions', { robot_id: 'amr_01', destination, strategy: 'direct' }); }
  cancelMission() { return this.command('DELETE', '/api/v1/missions/amr_01'); }
}
