import type { CampusScene } from './scene';

export interface MiniMark { x: number; y: number; kind: 'robot' | 'forklift' | 'worker' | 'obj' | 'camera'; sel?: boolean; stale?: boolean }

/** ATLAS top view (north up), drawn from the scene's world-frame footprints. */
export class Minimap {
  private readonly ctx: CanvasRenderingContext2D;
  private background: HTMLCanvasElement | null = null;
  constructor(readonly canvas: HTMLCanvasElement) { this.ctx = canvas.getContext('2d')!; }
  private X = (x: number) => (x + 77) / 154 * this.canvas.width;
  private Y = (y: number) => (-y + 58) / 116 * this.canvas.height;

  setScene(scene: CampusScene) {
    const bg = document.createElement('canvas'); bg.width = this.canvas.width; bg.height = this.canvas.height;
    const ctx = bg.getContext('2d')!, X = this.X, Y = this.Y, b = scene.building.interior;
    ctx.fillStyle = '#111f2e'; ctx.fillRect(0, 0, bg.width, bg.height);
    ctx.fillStyle = '#273e4e'; ctx.fillRect(X(b.x_min), Y(b.y_max), X(b.x_max) - X(b.x_min), Y(b.y_min) - Y(b.y_max));
    ctx.strokeStyle = '#4c6778'; ctx.lineWidth = 2; ctx.strokeRect(X(b.x_min), Y(b.y_max), X(b.x_max) - X(b.x_min), Y(b.y_min) - Y(b.y_max));
    const colours: Record<string, string> = { 'Estante': '#516f84', 'Célula de produção': '#7fc5dc', 'Esteira de triagem': '#7fc5dc', 'Mesa de inspeção': '#7fc5dc',
      'Estação de recarga': '#83d6cb', 'Palete no chão': '#6f6457', 'Escada do mezanino': '#798bb0', 'Pilar do mezanino': '#798bb0' };
    for (const s of scene.solids) {
      const colour = colours[s.tag]; if (!colour) continue;
      ctx.save(); ctx.translate(X(s.x), Y(s.y)); ctx.rotate(-s.yaw); ctx.fillStyle = colour;
      const w = s.length / 154 * bg.width, h = s.width / 116 * bg.height; ctx.fillRect(-w / 2, -h / 2, w, h); ctx.restore();
    }
    ctx.fillStyle = '#798bb055'; ctx.fillRect(X(50.5), Y(49), X(67.5) - X(50.5), Y(29) - Y(49));
    for (const lane of Object.values(scene.lanes)) {
      ctx.strokeStyle = lane.colour + '55'; ctx.lineWidth = 1.2; ctx.beginPath();
      lane.points.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.closePath(); ctx.stroke();
    }
    ctx.fillStyle = '#c9dee8'; ctx.font = 'bold 14px sans-serif'; ctx.fillText('N', bg.width - 20, 19);
    this.background = bg;
  }

  draw(marks: MiniMark[], path: [number, number][] = [], coverage: [number, number][][] = []) {
    const ctx = this.ctx, X = this.X, Y = this.Y;
    if (this.background) ctx.drawImage(this.background, 0, 0); else ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    for (const poly of coverage) {
      ctx.fillStyle = '#c79bff1c'; ctx.beginPath(); poly.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.closePath(); ctx.fill();
    }
    if (path.length > 1) {
      ctx.strokeStyle = '#5bf0d3'; ctx.lineWidth = 2; ctx.setLineDash([4, 3]); ctx.beginPath();
      path.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.stroke(); ctx.setLineDash([]);
    }
    for (const p of marks) {
      if (p.kind === 'obj') { ctx.fillStyle = '#ff9b5e'; ctx.fillRect(X(p.x) - 2, Y(p.y) - 2, 4, 4); continue; }
      if (p.kind === 'camera') { ctx.fillStyle = '#c79bff'; ctx.beginPath(); ctx.moveTo(X(p.x), Y(p.y) - 4); ctx.lineTo(X(p.x) + 3.5, Y(p.y) + 3); ctx.lineTo(X(p.x) - 3.5, Y(p.y) + 3); ctx.fill(); continue; }
      ctx.beginPath(); ctx.arc(X(p.x), Y(p.y), p.kind === 'robot' ? 4.5 : p.kind === 'forklift' ? 4 : p.sel ? 4.5 : 2.6, 0, Math.PI * 2);
      ctx.fillStyle = p.stale ? '#7d8a97' : p.kind === 'robot' ? '#5bf0d3' : p.kind === 'forklift' ? '#ffc978' : p.sel ? '#ffffff' : '#f48e8c';
      ctx.fill(); if (p.sel && p.kind !== 'worker') { ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke(); }
    }
  }
}
