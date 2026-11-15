// Computer-vision view of an AMR's onboard camera (LIVE): the frames the perception service
// (vision/scripts/serve_perception.py) actually processed, with that model's detections.
//
// Image and detections arrive in one message (uint32 LE header length, JSON header, JPEG), so a box
// is only ever drawn on the frame it was predicted for; nothing is drawn from scene coordinates or
// ground truth. Boxes are in the frame's pixel coordinates and scaled with the image to the onboard
// viewport, so they stay aligned at any canvas size. Frames of another robot are never shown.
import type { Rect } from '../cameraPoses';

export interface Detection { class_id: number; label: string; confidence: number; bbox_xyxy: [number, number, number, number] }
export interface DetectionFrame {
  type: 'detections'; robot_id: string; frame_id: number; stamp: number; world_sequence: number | null; width: number; height: number;
  model: string; model_name: string; confidence_threshold: number; device: string; detections: Detection[]; fps: number;
  timing_ms: Record<string, number>; state_age_ms: number | null;
}
interface Status {
  enabled: boolean; model: { id: string; run: string }; model_name: string; device: string; confidence_threshold: number;
  robots: Record<string, { fps: number; latency_ms_p50: number | null; latency_ms_p95: number | null; frames: number; dropped_stale: number;
    capture_errors: number; last_error: string | null; objects: number; age_s: number | null }>;
}

const COLOURS: Record<string, string> = { person: '#ff6b6b', forklift: '#ffc14d', amr: '#5bf0d3', pallet: '#e8a35a', box: '#d78cff', cone: '#ff8a3d' };
const STALE_SECONDS = 2.5;
const fmt = (v: number, d = 1) => v.toFixed(d).replace('.', ',');

/** Parse one service message; throws on anything that is not a well-formed frame. */
export function parseFrame(data: ArrayBuffer): { header: DetectionFrame; jpeg: Blob } {
  const length = new DataView(data).getUint32(0, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(data, 4, length))) as DetectionFrame;
  if (header.type !== 'detections' || typeof header.robot_id !== 'string' || !Number.isInteger(header.frame_id)
      || !(header.width > 0 && header.height > 0) || !Array.isArray(header.detections)) throw new Error('invalid perception frame');
  for (const d of header.detections) if (!Array.isArray(d.bbox_xyxy) || d.bbox_xyxy.length !== 4 || !d.bbox_xyxy.every(Number.isFinite)) throw new Error('invalid detection');
  return { header, jpeg: new Blob([new Uint8Array(data, 4 + length)], { type: 'image/jpeg' }) };
}

/** Image → viewport transform: the frame letterboxed into the viewport with its own aspect ratio. */
export function frameTransform(viewport: Rect, width: number, height: number) {
  const scale = Math.min(viewport.width / width, viewport.height / height);
  return { scale, x: viewport.x + (viewport.width - width * scale) / 2, y: viewport.y + (viewport.height - height * scale) / 2 };
}

export class VisionView {
  readonly panel: HTMLElement;
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private socket: WebSocket | null = null;
  private robot: string | null = null;
  private frame: { header: DetectionFrame; bitmap: ImageBitmap; received: number } | null = null;
  private status: Status | null = null;
  private statusError = '';
  private poll: ReturnType<typeof setInterval> | undefined;
  private retry: ReturnType<typeof setTimeout> | undefined;
  private drawn = '';
  enabled = false;
  showBoxes = true;
  showConfidence = false;

  constructor(readonly api: string, parent: HTMLElement) {
    this.canvas = document.createElement('canvas'); this.canvas.className = 'vision-overlay'; this.canvas.hidden = true;
    parent.appendChild(this.canvas);
    this.ctx = this.canvas.getContext('2d')!;
    this.panel = document.createElement('div'); this.panel.className = 'vision-panel'; this.panel.hidden = true;
    this.panel.innerHTML = `<div class="hudtitle">VISÃO COMPUTACIONAL <span class="small-pill" data-v="pill">DESLIGADA</span></div>
      <label class="vrow"><input type="checkbox" data-v="enable"> Detecção de objetos</label>
      <label class="vrow"><input type="checkbox" data-v="boxes" checked> Caixas delimitadoras</label>
      <label class="vrow"><input type="checkbox" data-v="confidence"> Mostrar confiança</label>
      <div class="vrow vslider">Limiar de confiança <b data-v="thr">—</b><input type="range" min="0.05" max="0.95" step="0.05" data-v="threshold"></div>
      <div class="vmeta" data-v="meta">Modelo: —</div>`;
    parent.appendChild(this.panel);
    const q = <T extends HTMLElement>(k: string) => this.panel.querySelector<T>(`[data-v="${k}"]`)!;
    q<HTMLInputElement>('enable').onchange = e => this.setEnabled((e.target as HTMLInputElement).checked);
    q<HTMLInputElement>('boxes').onchange = e => { this.showBoxes = (e.target as HTMLInputElement).checked; this.drawn = ''; };
    q<HTMLInputElement>('confidence').onchange = e => { this.showConfidence = (e.target as HTMLInputElement).checked; this.drawn = ''; };
    q<HTMLInputElement>('threshold').onchange = e => void this.setThreshold(Number((e.target as HTMLInputElement).value));
  }

  /** Robot whose onboard camera is on screen (null: no onboard view → panel and stream off). */
  setRobot(robot: string | null) {
    if (robot === this.robot) return;
    this.robot = robot; this.close(); this.frame = null; this.drawn = '';
    this.panel.hidden = robot === null;
    if (robot && this.enabled) this.open();
  }
  setEnabled(on: boolean) {
    this.enabled = on; this.close(); this.frame = null; this.drawn = '';
    if (on && this.robot) this.open();
    if (on) { void this.refresh(); this.poll = setInterval(() => void this.refresh(), 2000); } else clearInterval(this.poll);
  }
  private open() {
    const robot = this.robot!;
    const socket = new WebSocket(`${this.api.replace(/^http/, 'ws')}/ws/v1/perception?robot_id=${encodeURIComponent(robot)}`);
    socket.binaryType = 'arraybuffer'; this.socket = socket;
    socket.onmessage = async event => {
      try {
        const { header, jpeg } = parseFrame(event.data as ArrayBuffer);
        if (header.robot_id !== this.robot || socket !== this.socket) return;      // never another robot's frame
        if (this.frame && header.frame_id <= this.frame.header.frame_id) return;   // never go back in time
        const bitmap = await createImageBitmap(jpeg);
        if (header.robot_id !== this.robot || socket !== this.socket) { bitmap.close(); return; }
        this.frame?.bitmap.close();
        this.frame = { header, bitmap, received: performance.now() / 1000 }; this.drawn = '';
      } catch (error) { this.statusError = `Quadro inválido: ${(error as Error).message}`; }
    };
    socket.onclose = () => { if (socket === this.socket && this.enabled) { this.socket = null; this.retry = setTimeout(() => { if (this.enabled && this.robot && !this.socket) this.open(); }, 1500); } };
  }
  private close() {
    clearTimeout(this.retry);
    if (this.socket) { this.socket.onclose = this.socket.onmessage = null; this.socket.close(); this.socket = null; }
  }
  private async refresh() {
    try {
      const response = await fetch(`${this.api}/api/v1/perception/status`, { signal: AbortSignal.timeout(3000) });
      if (!response.ok) throw new Error(String(response.status));
      this.status = await response.json() as Status; this.statusError = '';
    } catch { this.status = null; this.statusError = 'Serviço de percepção indisponível (vision/scripts/serve_perception.py)'; }
  }
  private async setThreshold(value: number) {
    try {
      const response = await fetch(`${this.api}/api/v1/perception/config`, { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ confidence_threshold: value }), signal: AbortSignal.timeout(3000) });
      if (!response.ok) throw new Error(String(response.status));
      this.status = await response.json() as Status;
    } catch { this.statusError = 'Limiar não aplicado: serviço indisponível'; }
  }

  /** Panel text and overlay for this rendered frame; ``viewport``: the onboard viewport in CSS px. */
  update(viewport: Rect | null) {
    if (!this.robot || !viewport) { this.canvas.hidden = true; return; }
    // Panel at the top right of the onboard viewport (the view's HUD sits at its top left).
    const left = `${Math.max(viewport.x + 12, viewport.x + viewport.width - 302)}px`, top = `${viewport.y + 12}px`;
    if (this.panel.style.left !== left || this.panel.style.top !== top) Object.assign(this.panel.style, { left, top });
    const q = <T extends HTMLElement>(k: string) => this.panel.querySelector<T>(`[data-v="${k}"]`)!;
    const now = performance.now() / 1000, f = this.frame, robot = this.status?.robots[this.robot];
    const age = f ? now - f.received : null, stale = age !== null && age > STALE_SECONDS;
    const pill = !this.enabled ? 'DESLIGADA' : this.statusError ? 'INDISPONÍVEL' : !f ? 'AGUARDANDO' : stale ? 'DESATUALIZADA' : 'EXECUTANDO';
    q('pill').textContent = pill;
    q<HTMLInputElement>('enable').checked = this.enabled;
    const threshold = this.status?.confidence_threshold ?? f?.header.confidence_threshold;
    if (threshold !== undefined) { q('thr').textContent = fmt(threshold, 2); const slider = q<HTMLInputElement>('threshold'); if (document.activeElement !== slider) slider.value = String(threshold); }
    const meta: string[] = [];
    meta.push(`Modelo: <b>${f?.header.model_name ?? this.status?.model_name ?? 'YOLO26n'}</b> · ${f?.header.model ?? this.status?.model.id ?? '—'}`);
    meta.push(`Robô: <b>${this.robot}</b>${f ? ` · quadro ${f.header.frame_id} · t<sub>sim</sub> ${fmt(f.header.stamp, 2)} s` : ''}`);
    if (this.enabled) {
      meta.push(`FPS de inferência: <b>${robot ? fmt(robot.fps) : f ? fmt(f.header.fps) : '—'}</b> · dispositivo ${f?.header.device ?? this.status?.device ?? '—'}`);
      meta.push(`Latência ponta a ponta: <b>${f ? `${fmt(f.header.timing_ms.end_to_end ?? 0, 0)} ms` : '—'}</b>${robot?.latency_ms_p95 ? ` (p95 ${fmt(robot.latency_ms_p95, 0)} ms)` : ''}`);
      meta.push(`Objetos detectados: <b>${f ? f.header.detections.length : '—'}</b>${f && f.header.detections.length ? ' · ' + summary(f.header.detections) : ''}`);
      if (robot?.last_error) meta.push(`<span class="vwarn">${robot.last_error}</span>`);
      if (this.statusError) meta.push(`<span class="vwarn">${this.statusError}</span>`);
      if (stale) meta.push(`<span class="vwarn">Sem quadros novos há ${fmt(age!, 1)} s: exibindo o último quadro processado</span>`);
    } else meta.push('Ative a detecção para ver as imagens processadas pelo modelo e suas detecções.');
    const html = meta.join('<br>');
    if (q('meta').innerHTML !== html) q('meta').innerHTML = html;
    // ----------------------------------------------------------------- overlay (processed frame + its boxes)
    const show = this.enabled && !!f;
    this.canvas.hidden = !show;
    if (!show) return;
    const ratio = Math.min(devicePixelRatio || 1, 2);
    const key = `${f!.header.frame_id}:${viewport.x}:${viewport.y}:${viewport.width}:${viewport.height}:${ratio}:${this.showBoxes}:${this.showConfidence}:${stale}`;
    if (key === this.drawn) return;
    this.drawn = key;
    Object.assign(this.canvas.style, { left: `${viewport.x}px`, top: `${viewport.y}px`, width: `${viewport.width}px`, height: `${viewport.height}px` });
    this.canvas.width = Math.round(viewport.width * ratio); this.canvas.height = Math.round(viewport.height * ratio);
    const ctx = this.ctx, h = f!.header, local = { x: 0, y: 0, width: viewport.width, height: viewport.height };
    const t = frameTransform(local, h.width, h.height);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.fillStyle = '#04080e'; ctx.fillRect(0, 0, viewport.width, viewport.height);
    ctx.globalAlpha = stale ? .55 : 1;
    ctx.drawImage(f!.bitmap, t.x, t.y, h.width * t.scale, h.height * t.scale);
    ctx.globalAlpha = 1;
    if (this.showBoxes) {
      ctx.font = '600 12px "DM Sans", system-ui, sans-serif'; ctx.lineWidth = 2;
      for (const d of h.detections) {
        const [x0, y0, x1, y1] = d.bbox_xyxy, colour = COLOURS[d.label] ?? '#ffffff';
        const x = t.x + x0 * t.scale, y = t.y + y0 * t.scale, w = (x1 - x0) * t.scale, hh = (y1 - y0) * t.scale;
        ctx.strokeStyle = colour; ctx.strokeRect(x, y, w, hh);
        const text = this.showConfidence ? `${d.label} ${fmt(d.confidence, 2)}` : d.label;
        const tw = ctx.measureText(text).width + 8, ty = y > 16 ? y - 16 : y;
        ctx.fillStyle = colour; ctx.fillRect(x - 1, ty, tw, 16); ctx.fillStyle = '#071019'; ctx.fillText(text, x + 3, ty + 12);
      }
    }
    ctx.fillStyle = 'rgba(4,8,14,.7)'; ctx.fillRect(t.x, t.y + h.height * t.scale - 20, 330, 20);
    ctx.fillStyle = '#c9dee8'; ctx.font = '500 11px "DM Sans", system-ui, sans-serif';
    ctx.fillText(`${h.robot_id} · quadro ${h.frame_id} · ${h.width}×${h.height} · imagem processada pelo modelo`, t.x + 6, t.y + h.height * t.scale - 6);
  }

  dispose() { this.setEnabled(false); this.close(); this.frame?.bitmap.close(); }
}

function summary(detections: Detection[]) {
  const counts = new Map<string, number>();
  for (const d of detections) counts.set(d.label, (counts.get(d.label) ?? 0) + 1);
  return [...counts].map(([k, v]) => `${v} ${k}`).join(', ');
}
