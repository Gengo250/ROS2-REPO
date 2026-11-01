import './atlas.css';
import { Plane, Raycaster, Vector2, Vector3 } from 'three';
import { DynamicBatch, InstancedRenderer } from './render/InstancedRenderer';
import { OrbitCamera } from './OrbitCamera';
import { buildStatic, fetchScene, LAYER_GROUPS, type CampusScene, type Zone } from './scene';
import { CameraCoverage, cameraAxes, coverage } from './CameraCoverage';
import { Minimap, type MiniMark } from './Minimap';
import { LiveWorld, type RenderedEntity } from './live/LiveWorld';
import { LiveScene } from './live/LiveScene';
import { createDemo, type Demo } from './demo/atlasDemo.js';
import { boxDyn, drawPlacement } from './rig/atlasRig.js';
import { rosToGl } from './coords';
import type { WorldEvent } from '../core/types/world-state';

const $ = <T extends HTMLElement = HTMLElement>(s: string) => document.querySelector<T>(s)!;
const params = new URLSearchParams(location.search);
const mode: 'live' | 'demo' = params.get('mode') === 'demo' ? 'demo' : 'live';
const API = import.meta.env.VITE_API_URL || 'http://127.0.0.1:8000';
const WS = import.meta.env.VITE_WORLD_WS_URL || `${API.replace(/^http/, 'ws')}/ws/v1/world?protocol_version=1.0`;
const FPS_CAP = Math.max(15, Math.min(240, Number(params.get('fps')) || 60));
document.body.className = mode;

const canvas = $<HTMLCanvasElement>('#scene');
const renderer = new InstancedRenderer(canvas);
const camera = new OrbitCamera(canvas, () => { following = false; });
const batch = new DynamicBatch();
const cctv = new CameraCoverage();
renderer.overlay.add(cctv.group);
const minimap = new Minimap($<HTMLCanvasElement>('#minimap'));
const show: Record<string, boolean> = { roof: false, enclosure: false, racks: true, vehicles: true, people: true, routes: true, labels: true, fields: true, cctv: true };
let night = 0, scene: CampusScene, live: LiveWorld | null = null, liveScene: LiveScene | null = null, demo: Demo | null = null;
let tool: string | null = null, lastTool = 'cone', hover: { x: number; z: number; ok: boolean } | null = null;
let selectedId: string | null = null, following = false, active = true, speed = 1;
const counters = { stops: 0, yields: 0, detours: 0, hits: 0 };
const log: { time: string; msg: string }[] = [];
let coverages: [number, number][][] = [];

function toast(message: string) { const e = $('#toast'); e.textContent = message; e.classList.add('show'); clearTimeout((toast as unknown as { t?: number }).t); (toast as unknown as { t?: number }).t = window.setTimeout(() => e.classList.remove('show'), 2600); }
function logEvent(msg: string) { log.unshift({ time: clockText().slice(0, 5), msg }); log.length = Math.min(log.length, 6); }
function clockText() {
  const seconds = mode === 'demo' ? Math.floor((demo?.time ?? 0) * 12) : Math.floor(live?.latest?.simulation.sim_time ?? 0);
  const h = mode === 'demo' ? 8 + Math.floor(seconds / 3600) % 15 : Math.floor(seconds / 3600);
  return `${String(h).padStart(2, '0')}:${String(Math.floor(seconds / 60) % 60).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}
const fmt = (v: number, d = 1) => v.toFixed(d).replace('.', ',');

// ------------------------------------------------------------------ scene and panels
async function loadScene(): Promise<CampusScene> {
  if (mode === 'live') {
    try { return await fetchScene(`${API}/api/v1/world/scene`, AbortSignal.timeout(5000)); }
    catch { banner('Gateway indisponível: a geometria exibida é a cópia local da cena. Nenhum estado do ROS está sendo mostrado.', 'warn'); }
  }
  return fetchScene('/scene/atlas_campus.scene.json');
}
function banner(text: string, kind: 'warn' | 'error' | '' = '') { const b = $('#banner'); b.textContent = text; b.className = `banner${text ? ' show' : ''}${kind === 'warn' ? ' warn' : ''}`; }

const labelNodes: { el: HTMLElement; z: Zone }[] = [];
function buildPanels() {
  $('#cameras').innerHTML = Object.entries(scene.views).map(([id, v], i) =>
    `<button class="camera-btn${i === 0 ? ' active' : ''}" data-view="${id}"><span class="camicon">${['◈', '▤', '▥', '◇', '▦'][i] ?? '◈'}</span> ${v.label} <span class="key">${i + 1}</span></button>`).join('');
  $('#cctv').innerHTML = scene.cameras.map(c => `<button class="camera-btn" data-cctv="${c.id}"><span class="camicon">◉</span> ${c.label}</button>`).join('');
  $('#zones').innerHTML = scene.zones.map(z => `<button data-focus="${z.name}">${z.name.split(' ').map((w, i) => i ? w[0] : w).join(' ')}</button>`).join('');
  $<HTMLSelectElement>('#missionTarget').innerHTML = scene.stations.map(s => `<option value="${s.id}">${s.label} · ${s.zone.toLowerCase()}</option>`).join('');
  $('#stockCount').innerHTML = `${(scene.metadata.stock_volumes / 1000).toFixed(1)}<span>k</span>`;
  for (const z of scene.zones) {
    const el = document.createElement('div'); el.className = 'hudlabel'; el.innerHTML = `<span>${z.icon} ${z.name}</span>`; el.title = z.name;
    el.onclick = () => selectZone(z); $('#labels').appendChild(el); labelNodes.push({ el, z });
  }
  if (mode === 'live') {
    const labels: [string, string][] = [['#lblStops', 'Paradas por campo de proteção'], ['#lblYield', 'Objetos inseridos'], ['#lblDetour', 'Objetos removidos']];
    for (const [id, text] of labels) $(id).textContent = text;
  }
}
function setView(name: string) {
  const v = scene.views[name]; if (!v) return;
  following = false; camera.setView(v); cctv.select(null);
  document.querySelectorAll<HTMLElement>('[data-view]').forEach(b => b.classList.toggle('active', b.dataset.view === name));
  toast('Câmera: ' + v.label);
}
function selectZone(z: Zone) {
  selectedId = null; following = false;
  $('#selection').innerHTML = `<h3>INSPEÇÃO DE SETOR <span class="small-pill">SELECIONADO</span></h3><div class="selected-title">${z.name}</div><div class="selected-meta">${z.desc}<br><span style="color:#73e9ce">● ${z.status}</span><br>${z.metric}</div>`;
  camera.focus(z.pos[0], z.pos[1]); toast('Setor selecionado: ' + z.name);
}
function selectCamera(id: string) {
  const cam = scene.cameras.find(c => c.id === id)!; cctv.select(id);
  const { forward } = cameraAxes(cam);
  const eye = rosToGl(...cam.position), target = rosToGl(cam.position[0] + forward[0]! * 10, cam.position[1] + forward[1]! * 10, cam.position[2] + forward[2]! * 10);
  $('#selection').innerHTML = `<h3>CÂMERA INDUSTRIAL <span class="small-pill">GEOMÉTRICA</span></h3><div class="selected-title">${cam.label}</div><div class="selected-meta">Posição (${fmt(cam.position[0])}; ${fmt(cam.position[1])}) m · altura ${fmt(cam.position[2])} m<br>Orientação ${fmt(cam.yaw * 180 / Math.PI, 0)}° · inclinação ${fmt(cam.pitch * 180 / Math.PI, 0)}°<br>Abertura ${fmt(cam.horizontal_fov * 180 / Math.PI, 0)}° × ${fmt(cam.vertical_fov * 180 / Math.PI, 0)}° · alcance ${cam.range} m<br>Resolução prevista ${cam.resolution[0]}×${cam.resolution[1]} · tópico futuro <code>${cam.future_topic}</code><br><span style="color:#ffc978">● Sem stream de imagem nem detecção (cobertura geométrica)</span></div><button class="followbtn" id="povBtn">Ver pela câmera</button>`;
  $('#povBtn').onclick = () => { camera.pov = { eye, target, fov: cam.vertical_fov * 180 / Math.PI, id: cam.id }; following = false; toast('Ponto de vista: ' + cam.label); };
  camera.focus(cam.position[0], cam.position[1], 60);
}

// ------------------------------------------------------------------ selection panel
function entityPanel(e: RenderedEntity) {
  const robot = live?.latest?.robots.find(r => r.id === e.id);
  const kind = e.type === 'amr' ? 'ROBÔ MÓVEL · NAV2' : e.type === 'forklift' ? 'EMPILHADEIRA' : 'FUNCIONÁRIO';
  const field = { free: ['#4fe3c1', 'livre'], warning: ['#ffbf45', 'alerta'], protective: ['#ff5145', 'proteção'] }[e.field_state];
  let meta = `${e.type === 'human' ? scene.agents.roles[scene.agents.workers.find(w => w.id === e.id)?.role ?? '']?.name ?? '' : kind}<br>`;
  meta += `<span style="color:#73e9ce">● ${e.stale ? 'Último estado conhecido' : e.state}${e.activity ? ' · ' + (scene.agents.acts[e.activity]?.label ?? e.activity) : ''}${e.reason ? ' · ' + e.reason : ''}</span><br>`;
  meta += `Velocidade ${fmt(Math.abs(e.speed))} m/s · posição (${fmt(e.x)}; ${fmt(e.y)}) m`;
  if (e.type !== 'human') meta += `<br>Campo de proteção <span style="color:${field[0]}">${field[1]}</span> · ${fmt(e.field_length)} m`;
  if (robot) meta += `<br>Interlock: ${robot.interlock ?? 'sem diagnóstico'}${robot.mission ? `<br>Missão ${robot.mission.destination}: ${robot.mission.status}` : ''}`;
  $('#selection').innerHTML = `<h3>${kind} <span class="small-pill">${e.stale ? 'DESATUALIZADO' : 'AO VIVO'}</span></h3><div class="selected-title">${e.id}</div><div class="selected-meta">${meta}</div><button class="followbtn" id="followBtn">${following ? 'Parar de acompanhar' : 'Acompanhar com a câmera'}</button>`;
}
function demoPanel(r: { id: string } & Record<string, unknown>, isWorker: boolean) {
  const meta = isWorker ? `${(r.R as { name: string }).name}<br><span style="color:#73e9ce">● ${demo!.workerStatus(r)}</span><br>Velocidade ${fmt(r.v as number)} m/s<br>Passos nesta sessão ${Math.floor(r.steps as number)}`
    : `${(r.S as { name: string }).name}<br><span style="color:${['#4fe3c1', '#ffbf45', '#ff5145'][r.field as number]}">● ${demo!.vehStatus(r)}</span><br>Velocidade ${fmt(r.v as number)} de ${fmt(r.speed as number)} m/s<br>Campo de proteção ${fmt(r.prot as number)} m<br>Percorrido ${Math.round(r.odo as number)} m`;
  $('#selection').innerHTML = `<h3>${isWorker ? 'FUNCIONÁRIO' : 'VEÍCULO'} <span class="small-pill">DEMO</span></h3><div class="selected-title">${r.id}</div><div class="selected-meta">${meta}</div><button class="followbtn" id="followBtn">${following ? 'Parar de acompanhar' : 'Acompanhar com a câmera'}</button>`;
}

// ------------------------------------------------------------------ LIVE wiring
function describe(e: WorldEvent): string | null {
  switch (e.kind) {
    case 'contact': return `Contato: ${e.entity} × ${e.other}`;
    case 'released': return `${e.entity}: contato liberado`;
    case 'protective_field': return `${e.entity}: objeto no campo de proteção`;
    case 'protective_stop': return e.detail.startsWith('in-place')
      ? `${e.entity}: giro bloqueado, ${e.other} na área de varredura`
      : `${e.entity}: parada de segurança, ${e.other} no campo de proteção`;
    case 'turned_back': return `${e.entity} esperou um veículo e voltou pela rota`;
    case 'obstacle_added': return `${e.detail === 'box' ? 'Caixa' : e.detail === 'pallet' ? 'Palete' : 'Cone'} inserido no piso (${e.entity})`;
    case 'obstacle_removed': return `Objeto ${e.entity} removido`;
    case 'paused': return 'Simulação pausada';
    case 'resumed': return 'Simulação retomada';
    default: return `${e.kind}: ${e.entity}`;
  }
}
let lastMission = '';
function startLive() {
  live = new LiveWorld(API, WS);
  liveScene = new LiveScene(scene);
  live.onEvents(events => {
    for (const e of events) {
      if (e.kind === 'protective_field' || e.kind === 'protective_stop') counters.stops++;
      if (e.kind === 'contact') counters.hits++;
      if (e.kind === 'obstacle_added') counters.yields++;
      if (e.kind === 'obstacle_removed') counters.detours++;
      const text = describe(e); if (text) logEvent(text);
    }
  });
  live.start();
  $('#missionSend').onclick = async () => {
    const target = $<HTMLSelectElement>('#missionTarget').value;
    try { const r = await live!.sendMission(target); toast(r.accepted ? `Missão enviada ao Nav2: ${target}` : `Missão recusada: ${r.reason}`); }
    catch (error) { toast(`Missão não enviada: ${(error as Error).message}`); }
  };
  $('#missionCancel').onclick = async () => {
    try { const r = await live!.cancelMission(); toast(r.accepted ? 'Cancelamento solicitado ao Nav2' : `Nada a cancelar: ${r.reason}`); }
    catch (error) { toast(`Falha ao cancelar: ${(error as Error).message}`); }
  };
}
function liveStatus() {
  const s = live!.latest, pill = $('#status'), text = $('#statusText');
  let label = 'CONECTANDO', cls = 'stale', msg = '';
  if (live!.connection !== 'connected') { label = 'GATEWAY DESCONECTADO'; cls = 'offline'; msg = `${live!.error || 'Sem conexão com o Gateway'}. Exibindo o último estado recebido; o navegador não simula nada.`; }
  else if (s) {
    const sim = s.simulation;
    if (s.connection.ros !== 'online') { label = 'ROS 2 DESCONECTADO'; cls = 'offline'; msg = 'O Gateway perdeu o ROS 2. Os estados exibidos são os últimos conhecidos.'; }
    else if (sim.status === 'running') { label = 'AMBIENTE ATIVO'; cls = ''; }
    else if (sim.status === 'paused') { label = 'SIMULAÇÃO PAUSADA'; cls = 'stale'; msg = 'O motor de simulação está pausado.'; }
    else if (sim.status === 'stalled') { label = 'SEM DADOS RECENTES'; cls = 'stale'; msg = `Motor de simulação sem atualizações há ${fmt(sim.age_seconds ?? 0)} s. Estados congelados no último valor.`; }
    else { label = 'SIMULAÇÃO INDISPONÍVEL'; cls = 'offline'; msg = 'Nenhum motor de simulação publicando. Inicie o modo lite (ros2 launch amr_lite_sim lite_navigation.launch.py).'; }
    if (!msg && sim.scene_sha256 && sim.scene_sha256 !== scene.metadata.scene_sha256) { msg = 'A cena do motor difere da geometria carregada. Recarregue a página.'; cls = 'stale'; }
    if (!msg && s.connection.registration !== 'verified') msg = 'Registro mundo/mapa não verificado: trajetórias do Nav2 não são desenhadas.';
    $('#modeBadge').textContent = `LIVE ROS · RTF ${sim.real_time_factor === null ? '—' : fmt(sim.real_time_factor, 2)}`;
    const m = s.robots[0]?.mission;
    const status = m ? `<b>${m.destination}</b> · ${m.status}${m.distance_remaining_m !== null ? ` · faltam ${fmt(m.distance_remaining_m)} m` : ''}${m.recoveries ? ` · ${m.recoveries} recuperações` : ''}${m.detail ? `<br>${m.detail}` : ''}` : 'Nenhuma missão ativa.';
    $('#missionStatus').innerHTML = `${status}<br>Interlock: ${s.robots[0]?.interlock ?? 'sem diagnóstico do Nav2'}`;
    const key = m ? `${m.id}:${m.status}` : '';
    if (m && key !== lastMission) { lastMission = key; logEvent(`Missão ${m.destination}: ${m.status}`); }
    $('#robotCount').textContent = String(s.entities.filter(e => e.type === 'amr').length).padStart(2, '0');
    $('#livePerf').innerHTML = `Fator de tempo real: <b>${sim.real_time_factor === null ? '—' : fmt(sim.real_time_factor, 3)}</b> · tempo simulado <b>${fmt(sim.sim_time ?? 0, 1)} s</b>`;
  }
  pill.className = `status ${cls}`; text.textContent = label;
  banner(msg, cls === 'offline' ? 'error' : msg ? 'warn' : '');
}

// ------------------------------------------------------------------ DEMO wiring
function startDemo() {
  demo = createDemo(scene, { clock: clockText, toast });
  $('#modeBadge').textContent = 'SIMULAÇÃO DEMONSTRATIVA'; $('#statusText').textContent = 'AMBIENTE DEMO'; $('#legendPill').textContent = 'DEMO';
  $('#credit').innerHTML = 'Three.js · shaders ATLAS · <strong>simulação no navegador</strong>';
  $('#robotCount').textContent = String(demo.robots.length).padStart(2, '0');
  // Fixed-step demo loop, independent of the render loop (keeps running at any frame rate).
  let accumulator = 0, lastTick = performance.now();
  window.setInterval(() => {
    const now = performance.now(), dt = Math.min(.25, (now - lastTick) / 1000); lastTick = now;
    if (!active) return;
    accumulator += dt * speed;
    while (accumulator >= 1 / 60) { demo!.step(1 / 60); accumulator -= 1 / 60; }
  }, 1000 / 60);
  $('#sensors').onchange = e => demo!.setSensors((e.target as HTMLInputElement).checked);
  $('#play').onclick = () => { active = !active; $('#play').textContent = active ? 'Ⅱ' : '▶'; toast(active ? 'Simulação retomada' : 'Simulação pausada'); };
  $<HTMLInputElement>('#speed').oninput = e => { speed = Number((e.target as HTMLInputElement).value); $('#speedLabel').textContent = speed.toFixed(2).replace(/0$/, '').replace('.', ',') + '×'; };
}

// ------------------------------------------------------------------ picking and tools
const raycaster = new Raycaster(), floor = new Plane(new Vector3(0, 1, 0), 0), mouse = new Vector2(), hit = new Vector3();
function floorPoint(cx: number, cy: number) {
  const r = canvas.getBoundingClientRect();
  mouse.set((cx - r.left) / r.width * 2 - 1, -((cy - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(mouse, renderer.camera);
  return raycaster.ray.intersectPlane(floor, hit) ? { x: hit.x, z: hit.z } : null;
}
function screen(x: number, y: number, z: number) {
  const v = new Vector3(x, y, z).project(renderer.camera);
  return v.z > 1 ? null : [(v.x + 1) / 2 * canvas.clientWidth, (1 - v.y) / 2 * canvas.clientHeight];
}
function setTool(type: string | null) {
  tool = tool === type ? null : type; hover = null;
  document.querySelectorAll<HTMLElement>('[data-tool]').forEach(b => b.classList.toggle('active', b.dataset.tool === tool));
  canvas.style.cursor = tool ? 'crosshair' : '';
  if (tool) toast('Clique no piso para posicionar: ' + (scene.obstacle_catalogue[tool]?.label ?? tool).toLowerCase());
}
async function toolClick(x: number, z: number) {
  if (mode === 'demo') { toast(demo!.toolClick(tool!, x, z)); return; }
  const wx = x, wy = -z;
  const existing = live?.latest?.obstacles.find(o => Math.abs(o.x - wx) < o.length / 2 + .2 && Math.abs(o.y - wy) < o.width / 2 + .2);
  try {
    const r = existing ? await live!.removeObstacle(existing.id) : await live!.addObstacle(tool!, wx, wy, 0);
    toast(r.accepted ? (existing ? `Objeto ${existing.id} removido pelo backend` : `${scene.obstacle_catalogue[tool!]!.label} aceito pelo backend (${r.obstacle?.id})`) : `Recusado pelo backend: ${r.reason}`);
  } catch (error) { toast(`Pedido não enviado: ${(error as Error).message}`); }
}
canvas.addEventListener('pointermove', e => {
  if (!tool) return;
  const p = floorPoint(e.clientX, e.clientY);
  hover = p ? { ...p, ok: mode === 'demo' ? demo!.placementCheck(tool, p.x, p.z).ok : true } : null;
});
canvas.addEventListener('pointerup', e => {
  const start = camera.clickStart; if (!start || Math.hypot(e.clientX - start[0]!, e.clientY - start[1]!) >= 6) return;
  if (tool) { const p = floorPoint(e.clientX, e.clientY); if (p) void toolClick(p.x, p.z); return; }
  const r = canvas.getBoundingClientRect(), cx = e.clientX - r.left, cy = e.clientY - r.top;
  const candidates: { id: string; at: number[]; radius: number }[] = [];
  if (mode === 'live') for (const en of lastEntities) candidates.push({ id: en.id, at: [en.x, en.z + (en.type === 'human' ? 1.1 : .6), -en.y], radius: en.type === 'human' ? 26 : 34 });
  else { for (const w of demo!.workers) candidates.push({ id: w.id, at: [w.x, w.y0 + 1.1, w.z], radius: 26 }); for (const v of [...demo!.robots, ...demo!.forklifts]) candidates.push({ id: v.id, at: [v.pose.x, 1, v.pose.z], radius: 34 }); }
  let best: string | null = null, bestD = Infinity;
  for (const c of candidates) { const p = screen(c.at[0]!, c.at[1]!, c.at[2]!); if (!p) continue; const d = Math.hypot(cx - p[0]!, cy - p[1]!); if (d < c.radius && d < bestD) { bestD = d; best = c.id; } }
  if (best) { selectedId = best; following = true; const d = camera.desired; d.distance = Math.min(d.distance, 18); d.pitch = Math.min(d.pitch, .66); toast('Acompanhando ' + best); return; }
  let zone: Zone | null = null, zd = 48;
  for (const { z } of labelNodes) { const p = screen(...rosToGl(z.pos[0], z.pos[1], z.pos[2])); if (!p) continue; const d = Math.hypot(cx - p[0]!, cy - p[1]!); if (d < zd) { zd = d; zone = z; } }
  if (zone) selectZone(zone);
});

// ------------------------------------------------------------------ controls
function bindControls() {
  $('#cameras').onclick = e => { const b = (e.target as HTMLElement).closest<HTMLElement>('[data-view]'); if (b) setView(b.dataset.view!); };
  $('#cctv').onclick = e => { const b = (e.target as HTMLElement).closest<HTMLElement>('[data-cctv]'); if (b) selectCamera(b.dataset.cctv!); };
  $('#zones').onclick = e => { const b = (e.target as HTMLElement).closest<HTMLElement>('[data-focus]'); if (b) { const z = scene.zones.find(x => x.name === b.dataset.focus); if (z) selectZone(z); } };
  $('#tools').onclick = e => { const b = (e.target as HTMLElement).closest<HTMLElement>('[data-tool]'); if (b) { lastTool = b.dataset.tool!; setTool(lastTool); } };
  $('#clearObjs').onclick = async () => {
    if (mode === 'demo') { const n = demo!.clearUser(); toast(n ? (n === 1 ? '1 objeto removido' : n + ' objetos removidos') : 'Nenhum objeto posicionado'); return; }
    try { const r = await live!.clearObstacles(); toast(r.removed ? `${r.removed} objeto(s) removido(s) pelo backend` : 'Nenhum objeto posicionado'); }
    catch (error) { toast(`Pedido não enviado: ${(error as Error).message}`); }
  };
  document.querySelectorAll<HTMLInputElement>('input[data-layer]').forEach(el => el.onchange = () => {
    const layer = el.dataset.layer!; show[layer] = el.checked;
    for (const name of LAYER_GROUPS[layer] ?? []) renderer.setLayerVisible(name, el.checked);
    if (layer === 'cctv') cctv.group.visible = el.checked;
    if (demo) demo.opts.people = show.people!, demo.opts.vehicles = show.vehicles!;
    toast((el.checked ? 'Camada exibida: ' : 'Camada oculta: ') + el.parentElement!.firstChild!.textContent!.trim());
  });
  $<HTMLInputElement>('[data-night]').onchange = e => { night = (e.target as HTMLInputElement).checked ? 1 : 0; toast(night ? 'Iluminação noturna ativada' : 'Iluminação diurna ativada'); };
  $('#selection').onclick = e => { if ((e.target as HTMLElement).id === 'followBtn') following = !following; };
  $('#followNext').onclick = () => cycleWorker();
  $('#screenshot').onclick = () => { const a = document.createElement('a'); a.download = 'atlas-digital-twin.png'; a.href = canvas.toDataURL('image/png'); a.click(); toast('Captura salva'); };
  for (const b of document.querySelectorAll<HTMLButtonElement>('[data-mode]')) {
    b.classList.toggle('active', b.dataset.mode === mode);
    b.onclick = () => { if (b.dataset.mode !== mode) { const p = new URLSearchParams(location.search); p.set('mode', b.dataset.mode!); location.search = p.toString(); } };
  }
  window.addEventListener('keydown', e => {
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName ?? '')) return;
    if (e.code === 'Space' && mode === 'demo') { e.preventDefault(); $('#play').click(); }
    const views = Object.keys(scene.views);
    if (/Digit[1-9]/.test(e.code)) { const v = views[Number(e.code.slice(-1)) - 1]; if (v) setView(v); }
    if (e.code === 'KeyF') cycleWorker();
    if (e.code === 'KeyO') setTool(tool || lastTool);
    if (e.code === 'Escape') { if (tool) setTool(tool); else setView(views[0]!); }
  });
}
function cycleWorker() {
  if (!show.people) return;
  const ids = mode === 'live' ? lastEntities.filter(e => e.type === 'human').map(e => e.id) : demo!.workers.map(w => w.id as string);
  if (!ids.length) return;
  selectedId = ids[(ids.indexOf(selectedId ?? '') + 1) % ids.length]!; following = true;
  const d = camera.desired; d.distance = Math.min(d.distance, 16); d.pitch = Math.min(d.pitch, .62); toast('Acompanhando ' + selectedId);
}

// ------------------------------------------------------------------ render loop (presentation only)
let lastEntities: RenderedEntity[] = [];
let lastDraw = 0, lastPresent = 0, frame = 0, ratio = Math.min(devicePixelRatio || 1, 1.5), frameEma = 1000 / FPS_CAP, fps = 0, fpsFrames = 0, fpsStart = performance.now();
const frameTimes: number[] = [];
let instancesStatic = 0;
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) renderer.resize(w, h, ratio);
}
function followTarget(): number[] | null {
  if (!following || !selectedId) return null;
  if (mode === 'live') { const e = lastEntities.find(x => x.id === selectedId); return e ? [e.x, e.z + 1, -e.y] : null; }
  const w = demo!.workers.find(x => x.id === selectedId); if (w) return [w.x, w.y0 + 1, w.z];
  const v = [...demo!.robots, ...demo!.forklifts].find(x => x.id === selectedId); return v ? [v.pose.x, 1, v.pose.z] : null;
}
function render(now: number) {
  requestAnimationFrame(render);
  // Frame cap that keeps its phase: on a 144 Hz display a 60 FPS cap alternates 2 and 3
  // display frames instead of always waiting 3 (which would give 48 FPS).
  const interval = 1000 / FPS_CAP, elapsed = now - lastDraw;
  if (elapsed < interval) return;
  const dt = Math.min(.1, (now - lastPresent) / 1000);
  if (lastPresent) { frameTimes.push(now - lastPresent); if (frameTimes.length > 240) frameTimes.shift(); }
  lastDraw = now - (elapsed % interval); lastPresent = now; frame++;
  resize();
  camera.follow = followTarget;
  camera.update(dt);
  const sky = night ? [.034, .065, .108] : [.075, .129, .185];
  batch.reset();
  let path: [number, number][] = [];
  let dim = 0;
  if (mode === 'live' && live && liveScene) {
    lastEntities = live.entities();
    const s = live.latest, robot = s?.robots[0];
    path = (robot?.path ?? []) as [number, number][];
    const goal = robot?.mission && ['pending', 'active'].includes(robot.mission.status) ? robot.mission.goal : null;
    liveScene.draw(batch, lastEntities, s?.obstacles ?? [], path, goal, { people: show.people!, vehicles: show.vehicles!, fields: show.fields!,
      routes: show.routes!, selected: selectedId, eye: camera.eye, time: now / 1000 });
    dim = live.connection !== 'connected' || s?.simulation.availability !== 'online' ? .55 : 0;
  } else if (demo) {
    demo.draw(batch, { people: show.people, vehicles: show.vehicles, fields: show.fields, selected: demo.workers.find(w => w.id === selectedId) ?? null,
      eye: camera.eye, tool, hover });
  }
  if (show.cctv) for (const c of scene.cameras) {
    if (camera.pov?.id === c.id) continue;  // looking through this camera: do not draw its housing
    const [x, y, z] = rosToGl(...c.position), selectedCamera = cctv.selected === c.id;
    boxDyn(batch, x, y, z, .34, .2, .22, selectedCamera ? '#ffd07e' : '#c79bff', c.yaw, .5);
    boxDyn(batch, x + Math.cos(c.yaw) * .2, y - .05, z - Math.sin(c.yaw) * .2, .08, .14, .14, '#1b2129', c.yaw, 0);
  }
  if (mode === 'live' && tool && hover) drawPlacement(batch, hover, scene.obstacle_catalogue[tool]!.length, scene.obstacle_catalogue[tool]!.width);
  renderer.uploadDynamic(batch);
  const pov = camera.pov;
  if (pov) { renderer.camera.fov = pov.fov; renderer.camera.updateProjectionMatrix(); renderer.render(pov.eye, pov.target, sky, night, dim); }
  else { if (renderer.camera.fov !== 47) { renderer.camera.fov = 47; renderer.camera.updateProjectionMatrix(); } renderer.render(camera.eye, camera.state.target, sky, night, dim); }
  // Adaptive resolution: lower the pixel ratio while the delivered rate stays well below the
  // cap; raise it again once the cap is held. Never below 0.75.
  frameEma += ((frameTimes.at(-1) ?? frameEma) - frameEma) * .05;
  if (frame % 90 === 0) {
    const delivered = 1000 / frameEma, best = Math.min(devicePixelRatio || 1, 1.5);
    if (delivered < FPS_CAP * .8 && ratio > .75) ratio = Math.max(.75, ratio - .125);
    else if (delivered > FPS_CAP * .95 && ratio < best) ratio = Math.min(best, ratio + .125);
  }
  fpsFrames++;
  if (now - fpsStart > 1000) { fps = fpsFrames * 1000 / (now - fpsStart); fpsFrames = 0; fpsStart = now; publishMetrics(); }
  if (frame % 3 === 0) updateLabels();
  if (frame % 9 === 0) updateMinimap(path);
  if (frame % 12 === 0) updatePanels();
}
function updateLabels() {
  for (const { z, el } of labelNodes) {
    const p = screen(...rosToGl(z.pos[0], z.pos[1], z.pos[2]));
    if (p && show.labels && camera.state.distance < 310 && p[0]! > 0 && p[0]! < canvas.clientWidth && p[1]! > 70 && p[1]! < canvas.clientHeight) {
      el.style.display = 'block'; el.style.left = p[0] + 'px'; el.style.top = p[1] + 'px';
    } else el.style.display = 'none';
  }
}
function updateMinimap(path: [number, number][]) {
  const marks: MiniMark[] = [];
  if (mode === 'live') {
    for (const e of lastEntities) if (e.type !== 'human' || (show.people && e.z < 1)) marks.push({ x: e.x, y: e.y, kind: e.type === 'amr' ? 'robot' : e.type === 'forklift' ? 'forklift' : 'worker', sel: e.id === selectedId, stale: e.stale });
    for (const o of live?.latest?.obstacles ?? []) marks.push({ x: o.x, y: o.y, kind: 'obj' });
  } else if (demo) {
    for (const r of demo.robots) marks.push({ x: r.pose.x, y: -r.pose.z, kind: 'robot', sel: r.id === selectedId });
    for (const r of demo.forklifts) marks.push({ x: r.pose.x, y: -r.pose.z, kind: 'forklift', sel: r.id === selectedId });
    for (const w of demo.workers) marks.push({ x: w.x, y: -w.z, kind: 'worker', sel: w.id === selectedId });
    for (const o of demo.loose) marks.push({ x: o.x, y: -o.z, kind: 'obj' });
  }
  for (const c of scene.cameras) marks.push({ x: c.position[0], y: c.position[1], kind: 'camera' });
  minimap.draw(marks, path, show.cctv ? coverages : []);
}
function updatePanels() {
  $('#clock').textContent = clockText();
  if (mode === 'live') liveStatus();
  const t = mode === 'demo' && demo ? demo.traffic : counters;
  $('#cntStops').textContent = String(t.stops); $('#cntYield').textContent = String(t.yields); $('#cntDetour').textContent = String(t.detours); $('#cntHits').textContent = String(t.hits);
  const entries = mode === 'demo' && demo ? demo.traffic.log : log;
  $('#eventlog').innerHTML = entries.length ? entries.map(e => `<div><b>${e.time}</b>${e.msg}</div>`).join('') : '<div class="empty">Nenhuma ocorrência registrada ainda.</div>';
  if (selectedId) {
    if (mode === 'live') { const e = lastEntities.find(x => x.id === selectedId); if (e) entityPanel(e); }
    else if (demo) { const w = demo.workers.find(x => x.id === selectedId); const v = [...demo.robots, ...demo.forklifts].find(x => x.id === selectedId); if (w) demoPanel(w, true); else if (v) demoPanel(v, false); }
  }
}
function publishMetrics() {
  const sorted = [...frameTimes].sort((a, b) => a - b), p95 = sorted[Math.floor(sorted.length * .95)] ?? 0;
  const info = renderer.renderer.info;
  const metrics = { mode, fps, frame_ms_p95: p95, fps_cap: FPS_CAP, pixel_ratio: ratio, draw_calls: info.render.calls, triangles: info.render.triangles,
    static_instances: instancesStatic, dynamic_instances: batch.total, connection: live?.connection ?? null, ws_hz: live?.receivedHz ?? null,
    ws_latency_ms: live?.latencyMs ?? null, sim_status: live?.latest?.simulation.status ?? null, rtf: live?.latest?.simulation.real_time_factor ?? null,
    entities: mode === 'live' ? lastEntities.length : (demo ? demo.workers.length + demo.robots.length + demo.forklifts.length : 0),
    obstacles: mode === 'live' ? live?.latest?.obstacles.length ?? 0 : demo?.loose.length ?? 0, js_heap_mb: (performance as unknown as { memory?: { usedJSHeapSize: number } }).memory?.usedJSHeapSize ? (performance as unknown as { memory: { usedJSHeapSize: number } }).memory.usedJSHeapSize / 1e6 : null };
  document.body.dataset.metrics = JSON.stringify(metrics);
  $('#perf').innerHTML = `Desempenho: <b>${fps.toFixed(0)} FPS</b> (limite ${FPS_CAP}) · quadro p95 <b>${p95.toFixed(1)} ms</b><br>${info.render.calls} chamadas · ${(info.render.triangles / 1000).toFixed(0)}k triângulos · resolução ${ratio.toFixed(2)}×` +
    (mode === 'live' ? `<br>WebSocket <b>${(live?.receivedHz ?? 0).toFixed(1)} Hz</b> · latência <b>${live?.latencyMs === null || live?.latencyMs === undefined ? '—' : live.latencyMs.toFixed(0) + ' ms'}</b>` : '<br>Simulação do navegador: passo fixo 60 Hz');
}

// ------------------------------------------------------------------ boot
async function boot() {
  try {
    scene = await loadScene();
    $('#sceneTitle').textContent = scene.metadata.title;
    instancesStatic = buildStatic(renderer, scene);
    for (const [layer, names] of Object.entries(LAYER_GROUPS)) for (const name of names) renderer.setLayerVisible(name, show[layer] ?? true);
    cctv.build(scene.cameras); coverages = scene.cameras.map(c => coverage(c));
    minimap.setScene(scene);
    buildPanels(); bindControls();
    camera.setView(scene.views.overview!); camera.state.yaw = camera.desired.yaw; camera.state.pitch = camera.desired.pitch;
    camera.state.distance = camera.desired.distance; camera.state.target = [...camera.desired.target];
    if (mode === 'live') startLive(); else startDemo();
    document.body.dataset.ready = 'true';
    $('#busy').style.opacity = '0'; setTimeout(() => $('#busy').remove(), 780);
    requestAnimationFrame(render);
  } catch (error) {
    $('#busyText').textContent = 'FALHA AO CARREGAR: ' + String((error as Error).message).toUpperCase();
  }
}
void boot();
window.addEventListener('pagehide', () => { live?.stop(); renderer.dispose(); });
