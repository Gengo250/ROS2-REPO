import './atlas.css';
import { Plane, Raycaster, Vector2, Vector3 } from 'three';
import { DynamicBatch, InstancedRenderer } from './render/InstancedRenderer';
import { OrbitCamera, type CameraState } from './OrbitCamera';
import { buildStatic, fetchScene, LAYER_GROUPS, WALL_MODES, wallVisibility, type CampusScene, type WallMode, type Zone } from './scene';
import { fitViewport, industrialOptics, industrialPose, lookPose, onboardOptics, onboardPose, type MountedCamera, type Optics, type Pose, type Rect } from './cameraPoses';
import { ViewController, type ViewKind, type ViewTarget } from './ViewController';
import type { RenderView } from './render/InstancedRenderer';
import { Minimap, type MiniMark } from './Minimap';
import { LiveWorld, type RenderedEntity } from './live/LiveWorld';
import { LiveScene, type RobotRoute } from './live/LiveScene';
import { createDemo, type Demo } from './demo/atlasDemo.js';
import { boxDyn, drawPlacement } from './rig/atlasRig.js';
import { rosToGl } from './coords';
import { VisionView } from './vision/VisionView';
import type { WorldEvent, WorldState } from '../core/types/world-state';

const $ = <T extends HTMLElement = HTMLElement>(s: string) => document.querySelector<T>(s)!;
const params = new URLSearchParams(location.search);
const mode: 'live' | 'demo' = params.get('mode') === 'demo' ? 'demo' : 'live';
const API = import.meta.env.VITE_API_URL || 'http://127.0.0.1:8000';
const WS = import.meta.env.VITE_WORLD_WS_URL || `${API.replace(/^http/, 'ws')}/ws/v1/world?protocol_version=1.0`;
const FPS_CAP = Math.max(15, Math.min(240, Number(params.get('fps')) || 60));
/** Perception service (vision/scripts/serve_perception.py): processed onboard frames + detections. */
const PERCEPTION = import.meta.env.VITE_PERCEPTION_URL || 'http://127.0.0.1:8100';
document.body.className = mode;

const canvas = $<HTMLCanvasElement>('#scene');
const renderer = new InstancedRenderer(canvas);
const camera = new OrbitCamera(canvas, () => { following = false; });
const batch = new DynamicBatch();
const view = new ViewController();
const minimap = new Minimap($<HTMLCanvasElement>('#minimap'));
// routes: AMR navigation overlays only (LIVE: Nav2 plan and goal; DEMO: route ahead of each AMR).
const show: Record<string, boolean> = { roof: false, racks: true, vehicles: true, people: true, routes: true, labels: true, fields: true };
/** Upper building walls: presentation only (scene.ts, wallVisibility); the plinth is always drawn. */
let walls: WallMode = 'auto', wallsShown: Record<string, boolean> = {};
let vision: VisionView | null = null;
let night = 0, scene: CampusScene, live: LiveWorld | null = null, liveScene: LiveScene | null = null, demo: Demo | null = null;
let tool: string | null = null, lastTool = 'cone', hover: { x: number; z: number; ok: boolean } | null = null;
let selectedId: string | null = null, following = false, active = true, speed = 1;
const counters = { stops: 0, yields: 0, detours: 0, hits: 0 };
const log: { time: string; msg: string }[] = [];
/** Onboard RGB-D camera of the AMR model (URDF export); null if the file could not be read. */
let onboardMount: MountedCamera | null = null;
type OnboardBase = { x: number; y: number; z: number; yaw: number };
/** Last lens of each robot's onboard camera, keyed by robot id: a robot whose state stops arriving
 * keeps its own last pose (shown as stale), never another robot's. */
const onboardPoses = new Map<string, { base: OnboardBase; pose: Pose; optics: Optics }>();
let onboardBase: OnboardBase | null = null;
/** Selected AMR in DEMO (in LIVE the mission selector holds it: selectedAmrId()). */
let demoAmr: string | null = null;
/** Area between the panels where first-person views are framed, and the last rendered view. */
let stage: Rect = { x: 0, y: 0, width: 1, height: 1 }, rendered: RenderView | null = null;
let selectedCamera: string | null = null, targetViewport: Rect | null = null, hudHtml = '';
/** Inspection panel as authored in index.html, shown again when a camera selection ends. */
const SELECTION_IDLE = $('#selection').innerHTML;
/** Overview the last first-person view was entered from; "back to overview" (Esc) returns to it. */
let overviewBefore: { orbit: CameraState; following: boolean } | null = null;
const STATE_LABEL: Record<string, string> = { moving: 'Em movimento', stopped: 'Parado', blocked: 'Bloqueado' };

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
  $<HTMLSelectElement>('#missionRobot').innerHTML = scene.spawns.map(s => `<option value="${s.robot_id}">${s.robot_id.toUpperCase().replace('_', ' ')}</option>`).join('');
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
  following = false; camera.setView(v); enterView('overview', null); clearCameraSelection();
  document.querySelectorAll<HTMLElement>('[data-view]').forEach(b => b.classList.toggle('active', b.dataset.view === name));
  toast('Câmera: ' + v.label);
}
function selectZone(z: Zone) {
  clearCameraSelection(); selectedId = null; following = false;
  $('#selection').innerHTML = `<h3>INSPEÇÃO DE SETOR <span class="small-pill">SELECIONADO</span></h3><div class="selected-title">${z.name}</div><div class="selected-meta">${z.desc}<br><span style="color:#73e9ce">● ${z.status}</span><br>${z.metric}</div>`;
  camera.focus(z.pos[0], z.pos[1]); toast('Setor selecionado: ' + z.name);
}
function selectCamera(id: string) {
  const cam = scene.cameras.find(c => c.id === id)!;
  selectedId = null; selectedCamera = id;
  document.querySelectorAll<HTMLElement>('[data-cctv]').forEach(b => b.classList.toggle('active', b.dataset.cctv === id));
  $('#selection').innerHTML = `<h3>CÂMERA INDUSTRIAL <span class="small-pill">INSTALADA</span></h3><div class="selected-title">${cam.label}</div><div class="selected-meta">Lente em (${fmt(cam.position[0])}; ${fmt(cam.position[1])}) m · altura ${fmt(cam.position[2])} m<br>Orientação ${fmt(cam.yaw * 180 / Math.PI, 0)}° · inclinação ${fmt(cam.pitch * 180 / Math.PI, 0)}°<br>Abertura ${fmt(cam.horizontal_fov * 180 / Math.PI, 0)}° × ${fmt(cam.vertical_fov * 180 / Math.PI, 0)}° · resolução prevista ${cam.resolution[0]}×${cam.resolution[1]}<br>Tópico futuro <code>${cam.future_topic}</code><br><span style="color:#ffc978">● Sem stream de imagem nem detecção: a visão é uma renderização 3D da posição da lente</span></div><button class="followbtn" id="viewCameraBtn">Visualizar câmera <span class="key">C</span></button>`;
  $('#viewCameraBtn').onclick = () => enterView('industrial', id);
  // Already looking through a camera: switch straight to the chosen one.
  if (view.kind === 'industrial') enterView('industrial', id); else camera.focus(cam.position[0], cam.position[1], 60);
}
/** End the industrial camera selection: inspection panel, list highlight and housing colour. */
function clearCameraSelection() {
  if (!selectedCamera) return;
  selectedCamera = null;
  document.querySelectorAll<HTMLElement>('[data-cctv]').forEach(b => b.classList.remove('active'));
  $('#selection').innerHTML = SELECTION_IDLE;
}

// ------------------------------------------------------------------ view modes
/** Switch the main view (overview, an industrial camera, a robot's onboard camera). */
function enterView(kind: ViewKind, id: string | null) {
  if (kind === 'onboard' && !onboardMount) { toast('Modelo da câmera embarcada indisponível (robots/amr.cameras.json)'); return; }
  // A robot that never reached the backend state has no pose to place its lens from: never show
  // another robot's camera instead (an onboard view of a different robot is left for the overview).
  if (kind === 'onboard' && !onboardAvailable(id!)) {
    if (view.kind === 'onboard') backToOverview();
    toast(`Câmera embarcada de ${amrLabel(id!)} indisponível: robô ausente do estado do backend`); return;
  }
  // Leaving the overview: remember its orbit so "back to overview" returns exactly there.
  if (!view.firstPerson && kind !== 'overview') overviewBefore = { orbit: { ...camera.desired, target: [...camera.desired.target] }, following };
  // Leaving an industrial camera ends its selection (otherwise its panel stays on screen).
  if (view.kind === 'industrial' && kind !== 'industrial') clearCameraSelection();
  if (kind !== 'overview') following = false;
  view.enter(kind, id, performance.now() / 1000);
  camera.override = kind === 'overview' ? null : (dx, dy) => view.drag(dx, dy);
  canvas.classList.toggle('fp', kind !== 'overview'); canvas.classList.remove('look');
  if (kind === 'industrial') { const cam = scene.cameras.find(c => c.id === id)!; toast('Câmera industrial: ' + cam.label); }
  if (kind === 'onboard') toast(`Câmera embarcada: ${amrLabel(id!)}`);
  updateHud(true);
}
function nextCamera() {
  const ids = scene.cameras.map(c => c.id), current = view.kind === 'industrial' ? ids.indexOf(view.id!) : -1;
  const next = ids[(current + 1) % ids.length]!;
  // Into the lens first: selecting from the overview would also move the orbit onto the camera,
  // and Esc would then return there instead of to the overview the user left.
  enterView('industrial', next); selectCamera(next);
}
/** Esc / "Visão geral": leave the first-person view for the overview it was entered from. */
function backToOverview() {
  const before = overviewBefore;
  enterView('overview', null);
  if (before) { camera.desired = before.orbit; following = before.following; }
}
/** AMRs of the current mode (backend entities in LIVE, the prototype fleet in DEMO). */
function amrIds(): string[] { return mode === 'live' ? lastEntities.filter(e => e.type === 'amr').map(e => e.id) : demo!.robots.map(r => r.id as string); }
const amrLabel = (id: string) => id.toUpperCase().replace('_', ' ');
/** The selected AMR, one for the whole interface (mission selector, scene click, AMR panel and HUD
 * buttons, V): in LIVE the mission selector holds it, in DEMO ``demoAmr``. */
function selectedAmrId(): string | null {
  if (mode === 'live') return missionRobot() || null;
  return demoAmr && amrIds().includes(demoAmr) ? demoAmr : amrIds()[0] ?? null;
}
/** A pose for this robot's lens exists (backend entity, or its own last pose in this session). */
function onboardAvailable(id: string) {
  return mode === 'live' ? lastEntities.some(e => e.id === id && e.type === 'amr') || onboardPoses.has(id) : amrIds().includes(id);
}
/** Make ``id`` the selected AMR everywhere: mission selector, inspection panel and, while an onboard
 * camera is on screen, the camera itself (straight to that robot's lens, no stop at the overview). */
function selectAmr(id: string) {
  if (mode === 'live') {
    const select = $<HTMLSelectElement>('#missionRobot');
    if ([...select.options].some(o => o.value === id)) select.value = id;
  } else demoAmr = id;
  // Following someone who is not an AMR ends with the new selection; following an AMR carries over.
  if (selectedId !== id && !(selectedId && amrIds().includes(selectedId))) following = false;
  selectedId = id;
  if (view.kind === 'onboard' && view.id !== id) enterView('onboard', id);
}
function viewAmr(action: string, id: string) {
  selectAmr(id);
  if (action === 'onboard') { enterView('onboard', id); return; }
  enterView('overview', null); selectedId = id;
  const e = mode === 'live' ? lastEntities.find(x => x.id === id) : null, r = mode === 'demo' ? demo!.robots.find(x => x.id === id) : null;
  const at = e ? [e.x, e.y] : r ? [r.pose.x, -r.pose.z] : null;
  if (action === 'follow') { following = true; const d = camera.desired; d.distance = Math.min(d.distance, 18); d.pitch = Math.min(d.pitch, .66); toast('Acompanhando ' + id); }
  else { following = false; if (at) camera.focus(at[0]!, at[1]!, 24); toast('Visão externa: ' + id); }
}
/** Visualization buttons of an AMR panel; the active one reflects the current view. */
function viewModes(id: string) {
  const active = view.kind === 'onboard' && view.id === id ? 'onboard' : following && selectedId === id && view.kind === 'overview' ? 'follow' : 'external';
  const button = (key: string, label: string) => `<button data-viewmode="${key}" data-robot="${id}" class="${active === key ? 'active' : ''}">${label}</button>`;
  return `<div class="viewmodes"><div class="vmlabel">VISUALIZAÇÃO</div>${button('external', 'Visão externa')}${button('onboard', 'Câmera embarcada')}${button('follow', 'Acompanhar AMR')}</div>`;
}

// ------------------------------------------------------------------ selection panel
function entityPanel(e: RenderedEntity) {
  const robot = live?.latest?.robots.find(r => r.id === e.id);
  const kind = e.type === 'amr' ? 'ROBÔ MÓVEL · NAV2' : e.type === 'forklift' ? 'EMPILHADEIRA' : 'FUNCIONÁRIO';
  const field = { free: ['#4fe3c1', 'livre'], warning: ['#ffbf45', 'alerta'], protective: ['#ff5145', 'proteção'] }[e.field_state];
  let meta = `${e.type === 'human' ? scene.agents.roles[scene.agents.workers.find(w => w.id === e.id)?.role ?? '']?.name ?? '' : kind}<br>`;
  meta += `<span style="color:#73e9ce">● ${e.stale ? 'Último estado conhecido' : STATE_LABEL[e.state] ?? e.state}${e.activity ? ' · ' + (scene.agents.acts[e.activity]?.label ?? e.activity) : ''}${e.reason ? ' · ' + reasonText(e.reason) : ''}</span><br>`;
  meta += `Velocidade ${fmt(Math.abs(e.speed))} m/s · posição (${fmt(e.x)}; ${fmt(e.y)}) m`;
  if (e.type !== 'human') meta += `<br>Campo de proteção <span style="color:${field[0]}">${field[1]}</span> · ${fmt(e.field_length)} m`;
  if (robot) meta += `<br>Interlock: ${robot.interlock ?? 'sem diagnóstico'}${robot.mission ? `<br>Missão ${robot.mission.destination}: ${missionText(robot.mission)}` : ''}`;
  const controls = e.type === 'amr' ? viewModes(e.id) : `<button class="followbtn" id="followBtn">${following ? 'Parar de acompanhar' : 'Acompanhar com a câmera'}</button>`;
  $('#selection').innerHTML = `<h3>${kind} <span class="small-pill">${e.stale ? 'DESATUALIZADO' : 'AO VIVO'}</span></h3><div class="selected-title">${e.id}</div><div class="selected-meta">${meta}</div>${controls}`;
}
function demoPanel(r: { id: string } & Record<string, unknown>, isWorker: boolean) {
  const meta = isWorker ? `${(r.R as { name: string }).name}<br><span style="color:#73e9ce">● ${demo!.workerStatus(r)}</span><br>Velocidade ${fmt(r.v as number)} m/s<br>Passos nesta sessão ${Math.floor(r.steps as number)}`
    : `${(r.S as { name: string }).name}<br><span style="color:${['#4fe3c1', '#ffbf45', '#ff5145'][r.field as number]}">● ${demo!.vehStatus(r)}</span><br>Velocidade ${fmt(r.v as number)} de ${fmt(r.speed as number)} m/s<br>Campo de proteção ${fmt(r.prot as number)} m<br>Percorrido ${Math.round(r.odo as number)} m`;
  const controls = !isWorker && r.kind === 'amr' ? viewModes(r.id) : `<button class="followbtn" id="followBtn">${following ? 'Parar de acompanhar' : 'Acompanhar com a câmera'}</button>`;
  $('#selection').innerHTML = `<h3>${isWorker ? 'FUNCIONÁRIO' : 'VEÍCULO'} <span class="small-pill">DEMO</span></h3><div class="selected-title">${r.id}</div><div class="selected-meta">${meta}</div>${controls}`;
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
const missionLog = new Map<string, string>();
type MissionState = NonNullable<WorldState['robots'][number]['mission']>;
const robotName = (id: string | null | undefined) => id ? id.toUpperCase().replace('_', ' ') : 'outro AMR';
/** User-facing explanation of a mission state (fleet coordination included); Nav2 internals stay out. */
function missionText(m: MissionState): string {
  switch (m.status) {
    case 'pending': return 'Enviando a missão ao Nav2';
    case 'active': return 'Navegando';
    case 'recovering': return 'Recalculando trajetória';
    case 'succeeded': return 'Destino alcançado';
    case 'canceled': return 'Missão cancelada';
    case 'rejected': return 'Missão recusada';
    case 'failed': return 'Falha na navegação: destino não alcançado';
    case 'blocked': return `Bloqueio persistente${m.blocking_robot ? ` com o ${robotName(m.blocking_robot)}` : ''}. Intervenção necessária.`;
    case 'waiting':
      if (m.waiting_for === 'station') {
        // Reserved (its owner is on the way) or occupied (a robot is physically on it).
        const st = live?.latest?.stations?.find(s => s.id === m.waiting_on);
        const holder = m.blocking_robot ? ` (${st?.owner === m.blocking_robot && !(st.occupants as string[]).includes(m.blocking_robot) ? 'reservada para o' : 'ocupada pelo'} ${robotName(m.blocking_robot)})` : '';
        return `Aguardando liberação da estação ${m.waiting_on}${holder}${m.queue_position ? ` · posição ${m.queue_position} na fila` : ''}`;
      }
      if (m.waiting_for === 'corridor') return `Aguardando passagem do ${robotName(m.blocking_robot)} no corredor ${m.waiting_on}`;
      return `Aguardando passagem do ${robotName(m.blocking_robot ?? m.waiting_on)}`;
    default: return m.status;
  }
}
/** Engine reason of an entity state, in Portuguese where it names a contact. */
function reasonText(reason: string) {
  return reason.startsWith('contact:') ? `contato com ${reason.slice(8).startsWith('amr_') ? robotName(reason.slice(8)) : reason.slice(8)}` : reason;
}
/** Station of the selected robot's mission as a shared resource (owner and queue). */
function stationText(id: string): string {
  const st = live?.latest?.stations?.find(s => s.id === id);
  if (!st) return '';
  const parts = [st.owner ? `reservada para o ${robotName(st.owner)}` : st.occupants.length ? `ocupada pelo ${robotName(st.occupants[0])}` : 'livre'];
  if (st.queue.length) parts.push(`fila: ${st.queue.map(robotName).join(', ')}`);
  return `Estação ${id}: ${parts.join(' · ')}`;
}
/** Robot the mission panel addresses (selector; follows the AMR selected in the scene). */
function missionRobot() { return $<HTMLSelectElement>('#missionRobot').value; }
/** Nav2 plan and active goal of every AMR in the latest world state, keyed by robot id. */
function robotRoutes(): Map<string, RobotRoute> {
  const out = new Map<string, RobotRoute>();
  for (const r of live?.latest?.robots ?? []) {
    const goal = r.mission && ['pending', 'active'].includes(r.mission.status) ? r.mission.goal : null;
    out.set(r.id, { path: r.path as [number, number][], goal });
  }
  return out;
}
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
    const robot = missionRobot(), target = $<HTMLSelectElement>('#missionTarget').value;
    try { const r = await live!.sendMission(robot, target); toast(r.accepted ? `Missão de ${robot} enviada ao Nav2: ${target}` : `Missão de ${robot} recusada: ${r.reason}`); }
    catch (error) { toast(`Missão não enviada: ${(error as Error).message}`); }
  };
  $('#missionCancel').onclick = async () => {
    const robot = missionRobot();
    try { const r = await live!.cancelMission(robot); toast(r.accepted ? `Cancelamento de ${robot} solicitado ao Nav2` : `Nada a cancelar em ${robot}: ${r.reason}`); }
    catch (error) { toast(`Falha ao cancelar: ${(error as Error).message}`); }
  };
  // Choosing a robot selects it everywhere (and switches the onboard camera if one is on screen).
  $<HTMLSelectElement>('#missionRobot').onchange = () => { const id = missionRobot(); if (id) selectAmr(id); updatePanels(); };
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
    const nav = s.robots.find(r => r.id === missionRobot()), m = nav?.mission;
    const driving = m && ['active', 'recovering'].includes(m.status);
    const status = !nav ? `${missionRobot()} ausente do estado do backend.` : m ? `<b>${m.destination}</b> · ${missionText(m)}${driving && m.distance_remaining_m !== null ? ` · faltam ${fmt(m.distance_remaining_m)} m` : ''}${m.recoveries ? ` · ${m.recoveries} ${m.recoveries === 1 ? 'recuperação' : 'recuperações'}` : ''}${stationText(m.destination) ? `<br>${stationText(m.destination)}` : ''}` : 'Nenhuma missão ativa.';
    $('#missionStatus').innerHTML = `${status}<br>Interlock: ${nav?.interlock ?? 'sem diagnóstico do Nav2'}`;
    // Mission changes of every AMR go to the event log, each with its robot.
    for (const r of s.robots) {
      const key = r.mission ? `${r.mission.id}:${r.mission.status}:${r.mission.waiting_for ?? ''}:${r.mission.blocking_robot ?? ''}` : '';
      if (r.mission && key !== missionLog.get(r.id)) { missionLog.set(r.id, key); logEvent(`${robotName(r.id)} · ${r.mission.destination}: ${missionText(r.mission)}`); }
    }
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
/** Viewport of the last rendered frame (CSS px, canvas coordinates): the whole canvas, or the
 * camera's framed area in first-person views. Picking and labels map through it. */
function viewportRect(): Rect { return rendered?.viewport ?? { x: 0, y: 0, width: canvas.clientWidth, height: canvas.clientHeight }; }
function floorPoint(cx: number, cy: number) {
  const r = canvas.getBoundingClientRect(), v = viewportRect(), px = cx - r.left - v.x, py = cy - r.top - v.y;
  if (px < 0 || py < 0 || px > v.width || py > v.height) return null;
  mouse.set(px / v.width * 2 - 1, -(py / v.height) * 2 + 1);
  raycaster.setFromCamera(mouse, renderer.camera);
  return raycaster.ray.intersectPlane(floor, hit) ? { x: hit.x, z: hit.z } : null;
}
function screen(x: number, y: number, z: number) {
  const p = new Vector3(x, y, z).project(renderer.camera), v = viewportRect();
  return p.z > 1 ? null : [v.x + (p.x + 1) / 2 * v.width, v.y + (1 - p.y) / 2 * v.height];
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
  // In a first-person view a click only selects (the view stays on the chosen camera).
  // An AMR selected in an onboard view also switches to its camera (selectAmr).
  const pick = (id: string) => { if (amrIds().includes(id)) selectAmr(id); else selectedId = id; };
  if (best && view.firstPerson) { toast('Selecionado ' + best); pick(best); return; }
  if (best) { clearCameraSelection(); pick(best); following = true; const d = camera.desired; d.distance = Math.min(d.distance, 18); d.pitch = Math.min(d.pitch, .66); toast('Acompanhando ' + best); return; }
  if (view.firstPerson) return;
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
    if (demo) demo.opts.people = show.people!, demo.opts.vehicles = show.vehicles!;
    toast((el.checked ? 'Camada exibida: ' : 'Camada oculta: ') + el.parentElement!.firstChild!.textContent!.trim());
  });
  $('#wallModes').onclick = e => { const b = (e.target as HTMLElement).closest<HTMLElement>('[data-walls]'); if (b && WALL_MODES.includes(b.dataset.walls as WallMode)) setWalls(b.dataset.walls as WallMode); };
  $<HTMLInputElement>('[data-night]').onchange = e => { night = (e.target as HTMLInputElement).checked ? 1 : 0; toast(night ? 'Iluminação noturna ativada' : 'Iluminação diurna ativada'); };
  $('#selection').onclick = e => {
    const target = e.target as HTMLElement, choice = target.closest<HTMLElement>('[data-viewmode]');
    if (target.id === 'followBtn') following = !following;
    if (choice) viewAmr(choice.dataset.viewmode!, choice.dataset.robot!);
  };
  $('#viewHud').onclick = e => {
    const b = (e.target as HTMLElement).closest<HTMLElement>('[data-hud]'); if (!b) return;
    const action = b.dataset.hud!;
    if (action === 'overview') backToOverview();
    else if (action === 'next') nextCamera();
    else if (action === 'look') { view.setLook(!view.look.enabled); canvas.classList.toggle('look', view.look.enabled); toast(view.look.enabled ? 'Olhar ao redor: arraste no cenário (a orientação do sensor não muda)' : 'Vista alinhada ao sensor'); }
    else if (action === 'amr') viewAmr('onboard', b.dataset.robot!);
    else if (view.kind === 'onboard') viewAmr(action, view.id!);
    updateHud(true);
  };
  if (mode === 'live') $('#missionOnboard').onclick = () => { const id = selectedAmrId(); if (id) viewAmr('onboard', id); };
  $('#followNext').onclick = () => cycleWorker();
  $('#screenshot').onclick = () => { const a = document.createElement('a'); a.download = 'atlas-digital-twin.png'; a.href = canvas.toDataURL('image/png'); a.click(); toast('Captura salva'); };
  for (const b of document.querySelectorAll<HTMLButtonElement>('[data-mode]')) {
    b.classList.toggle('active', b.dataset.mode === mode);
    b.onclick = () => { if (b.dataset.mode !== mode) { const p = new URLSearchParams(location.search); p.set('mode', b.dataset.mode!); location.search = p.toString(); } };
  }
  window.addEventListener('keydown', e => {
    // Typing in a text field or a select keeps its keys; a focused checkbox or slider does not
    // swallow the view shortcuts (Esc after toggling a layer or the detection, for example).
    // The robot selector keeps focus after a choice: V and Esc still reach the views from it.
    const focused = document.activeElement as HTMLInputElement | null;
    if (focused?.tagName === 'SELECT' && ['KeyV', 'Escape'].includes(e.code)) focused.blur();
    else if (focused && (['TEXTAREA', 'SELECT'].includes(focused.tagName) || (focused.tagName === 'INPUT' && !['checkbox', 'radio', 'range'].includes(focused.type)))) return;
    if (e.code === 'Space' && mode === 'demo') { e.preventDefault(); $('#play').click(); }
    const views = Object.keys(scene.views);
    if (/Digit[1-9]/.test(e.code)) { const v = views[Number(e.code.slice(-1)) - 1]; if (v) setView(v); }
    if (e.code === 'KeyF') cycleWorker();
    if (e.code === 'KeyO') setTool(tool || lastTool);
    if (e.code === 'KeyC') nextCamera();
    if (e.code === 'KeyV') { const id = selectedAmrId(); if (id) viewAmr('onboard', id); }
    if (e.code === 'Escape') { if (tool) setTool(tool); else if (view.firstPerson) backToOverview(); else setView(views[0]!); }
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
  if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) { renderer.resize(w, h, ratio); stage = stageRect(); }
}
/** Area of the canvas left free by the panels: first-person views are framed inside it. */
function stageRect(): Rect {
  const W = canvas.clientWidth, H = canvas.clientHeight, gap = 14;
  const top = $('.topbar').getBoundingClientRect().bottom + gap, left = $('.leftpanel').getBoundingClientRect(), right = $('.rightpanel').getBoundingClientRect();
  const x0 = left.width ? left.right + gap : gap, x1 = right.width ? right.left - gap : W - gap;
  const bottom = H - $('.ribbon').getBoundingClientRect().height - 21 - gap;
  return { x: x0, y: top, width: Math.max(80, x1 - x0), height: Math.max(60, bottom - top) };
}
/** Desired camera of this frame from the current mode. Onboard poses come from the same
 * interpolated backend state that is drawn (or the DEMO's own state); a robot that vanished
 * keeps its own last pose (onboardPoses). */
function viewTarget(): ViewTarget {
  if (view.kind === 'industrial') {
    const cam = scene.cameras.find(c => c.id === view.id)!, optics = industrialOptics(cam);
    return { pose: industrialPose(cam), optics, viewport: fitViewport(stage, optics.aspect) };
  }
  if (view.kind === 'onboard' && onboardMount) {
    const id = view.id!;
    let base: OnboardBase | null = null, mount = onboardMount;
    if (mode === 'live') { const e = lastEntities.find(x => x.id === id && x.type === 'amr'); if (e) base = { x: e.x, y: e.y, z: e.z, yaw: e.yaw }; }
    else {
      // DEMO vehicles are the prototype's (1.64 m), not the URDF robot: same optics, orientation and
      // lens height, lens at the vehicle's nominal front.
      const r = demo?.robots.find(x => x.id === id);
      if (r) { base = { x: r.pose.x, y: -r.pose.z, z: 0, yaw: r.pose.yaw }; mount = { ...onboardMount, position: [r.S.cx + r.S.L / 2, 0, onboardMount.position[2]!] }; }
    }
    if (base) onboardPoses.set(id, { base, pose: onboardPose(base, mount), optics: onboardOptics(mount) });
    const lens = onboardPoses.get(id);
    if (lens) { onboardBase = lens.base; return { pose: lens.pose, optics: lens.optics, viewport: fitViewport(stage, lens.optics.aspect) }; }
  }
  return { pose: lookPose(camera.eye, camera.state.target), optics: { fov: 47, aspect: canvas.clientWidth / Math.max(canvas.clientHeight, 1), near: .2, far: 700 }, viewport: null };
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
  let routes = new Map<string, RobotRoute>();
  let dim = 0;
  if (mode === 'live' && live && liveScene) {
    lastEntities = live.entities();
    const s = live.latest;
    routes = robotRoutes();
    liveScene.draw(batch, lastEntities, s?.obstacles ?? [], routes, { people: show.people!, vehicles: show.vehicles!, fields: show.fields!,
      routes: show.routes!, selected: selectedId, eye: camera.eye, time: now / 1000, onboard: view.kind === 'onboard' ? view.id : null });
    dim = live.connection !== 'connected' || s?.simulation.availability !== 'online' ? .55 : 0;
  } else if (demo) {
    demo.draw(batch, { people: show.people, vehicles: show.vehicles, fields: show.fields, routes: show.routes, selected: demo.workers.find(w => w.id === selectedId) ?? null,
      eye: camera.eye, tool, hover, onboard: view.kind === 'onboard' ? view.id : null });
  }
  // Installed industrial cameras (physical models): the lens is at the scene position, the housing
  // behind it. The camera being looked through is not drawn.
  for (const c of scene.cameras) {
    if (view.kind === 'industrial' && view.id === c.id) continue;
    const [x, y, z] = rosToGl(...c.position), dx = Math.cos(c.yaw), dz = -Math.sin(c.yaw);
    boxDyn(batch, x - dx * .25, y, z - dz * .25, .34, .2, .22, c.id === selectedCamera ? '#ffd07e' : '#c79bff', c.yaw, .5);
    boxDyn(batch, x - dx * .04, y - .02, z - dz * .04, .08, .14, .14, '#1b2129', c.yaw, 0);
  }
  if (mode === 'live' && tool && hover) drawPlacement(batch, hover, scene.obstacle_catalogue[tool]!.length, scene.obstacle_catalogue[tool]!.width);
  renderer.uploadDynamic(batch);
  const target = viewTarget(); targetViewport = target.viewport;
  rendered = view.frame(target, now / 1000, { x: 0, y: 0, width: canvas.clientWidth, height: canvas.clientHeight });
  applyWalls(rendered.position);
  renderer.render(rendered, sky, night, dim);
  // Onboard camera of a LIVE AMR: its computer-vision panel (frames the model processed + detections).
  if (vision) { const onboard = view.kind === 'onboard' && !view.blending(now / 1000); vision.setRobot(view.kind === 'onboard' ? view.id : null); vision.update(onboard ? rendered.viewport : null, live?.renderTime ?? null); }
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
  if (frame % 9 === 0) updateMinimap(routes);
  if (frame % 12 === 0) updatePanels();
}
/** Upper walls for the camera of this frame (WebGL position: x east, z = -north). */
function applyWalls(position: number[]) {
  const next = wallVisibility(scene.building.shell, walls, { x: position[0]!, y: -position[2]! });
  for (const [layer, visible] of Object.entries(next)) if (wallsShown[layer] !== visible) renderer.setLayerVisible(layer, visible);
  wallsShown = next;
}
function setWalls(mode: WallMode) {
  walls = mode;
  document.querySelectorAll<HTMLElement>('[data-walls]').forEach(b => b.classList.toggle('active', b.dataset.walls === mode));
  toast({ auto: 'Paredes: corte automático das faces voltadas para a câmera', full: 'Paredes: galpão fechado', cut: 'Paredes: corte na altura do fechamento físico (3 m)' }[mode]);
}
function updateLabels() {
  for (const { z, el } of labelNodes) {
    const p = screen(...rosToGl(z.pos[0], z.pos[1], z.pos[2]));
    if (p && show.labels && !view.firstPerson && camera.state.distance < 310 && p[0]! > 0 && p[0]! < canvas.clientWidth && p[1]! > 70 && p[1]! < canvas.clientHeight) {
      el.style.display = 'block'; el.style.left = p[0] + 'px'; el.style.top = p[1] + 'px';
    } else el.style.display = 'none';
  }
}
function updateMinimap(routes: Map<string, RobotRoute>) {
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
  for (const c of scene.cameras) marks.push({ x: c.position[0], y: c.position[1], kind: 'camera', sel: view.kind === 'industrial' && view.id === c.id });
  minimap.draw(marks, [...routes].map(([id, r]) => ({ path: r.path, sel: id === selectedId })));
}
/** Discreet identifier of the first-person view, anchored to the camera's framed viewport. */
function updateHud(force = false) {
  const hud = $('#viewHud');
  if (!view.firstPerson) { hud.hidden = true; hudHtml = ''; return; }
  const look = `<button data-hud="look" class="${view.look.enabled ? 'active' : ''}">${view.look.enabled ? 'Alinhar ao sensor' : 'Olhar ao redor'}</button>`;
  let html: string;
  if (view.kind === 'industrial') {
    const c = scene.cameras.find(x => x.id === view.id)!;
    html = `<div class="hudtitle">CÂMERA INDUSTRIAL <span class="small-pill">PRIMEIRA PESSOA 3D</span></div><div class="hudname">${c.label}</div>`
      + `<div class="hudmeta">Lente a ${fmt(c.position[2])} m · inclinação ${fmt(c.pitch * 180 / Math.PI, 0)}° · ${fmt(c.horizontal_fov * 180 / Math.PI, 0)}° × ${fmt(c.vertical_fov * 180 / Math.PI, 0)}° · renderização 3D, não é imagem de câmera</div>`
      + `<div class="hudbtns"><button data-hud="overview">Voltar à visão geral <span class="key">ESC</span></button><button data-hud="next">Próxima câmera <span class="key">C</span></button>${look}</div>`;
  } else {
    const id = view.id!, m = onboardMount!;
    let status = '', stale = false;
    if (mode === 'live') {
      const e = lastEntities.find(x => x.id === id), mission = live?.latest?.robots.find(r => r.id === id)?.mission;
      stale = !e || e.stale;
      const navigating = mission && ['pending', 'active'].includes(mission.status);
      status = e ? `${navigating ? 'Navegando' : STATE_LABEL[e.state] ?? e.state} · ${fmt(Math.abs(e.speed), 2)} m/s${mission ? ` · destino ${mission.destination} (${missionText(mission)})` : ''}` : 'Robô ausente do estado do backend';
    } else {
      const r = demo!.robots.find(x => x.id === id);
      status = r ? `${demo!.vehStatus(r)} · ${fmt(r.v, 2)} m/s · AMR demonstrativo: lente na frente do veículo do protótipo` : '';
    }
    html = `<div class="hudtitle">CÂMERA EMBARCADA <span class="small-pill">PRIMEIRA PESSOA 3D</span></div><div class="hudname">${amrLabel(id)} · ${m.id}</div>`
      + `<div class="hudmeta">${status}<br><code>${m.frame_id}</code> (URDF) · ${fmt(m.horizontal_fov * 180 / Math.PI, 0)}° · ${m.resolution[0]}×${m.resolution[1]} · ${vision?.enabled && mode === 'live' ? 'imagem: quadros sintéticos processados pelo YOLO26n (painel de visão)' : 'renderização 3D da cena'}</div>`
      + (stale ? '<div class="hudwarn">DADOS DESATUALIZADOS · última pose recebida do backend</div>' : '')
      + `<div class="hudbtns">${[...amrIds()].sort().map(r => `<button data-hud="amr" data-robot="${r}" class="${r === id ? 'active' : ''}" title="Câmera embarcada de ${r}">${amrLabel(r)}</button>`).join('')}</div>`
      + `<div class="hudbtns"><button data-hud="external">Visão externa</button><button data-hud="follow">Acompanhar AMR</button>${look}<button data-hud="overview">Visão geral <span class="key">ESC</span></button></div>`;
  }
  const v = targetViewport ?? stage;
  hud.style.left = `${v.x + 12}px`; hud.style.top = `${v.y + 12}px`;
  if (force || html !== hudHtml) { hud.innerHTML = html; hudHtml = html; }
  hud.hidden = false;
}
function updatePanels() {
  $('#clock').textContent = clockText();
  updateHud();
  if (mode === 'live') { liveStatus(); onboardButton(); }
  const t = mode === 'demo' && demo ? demo.traffic : counters;
  $('#cntStops').textContent = String(t.stops); $('#cntYield').textContent = String(t.yields); $('#cntDetour').textContent = String(t.detours); $('#cntHits').textContent = String(t.hits);
  const entries = mode === 'demo' && demo ? demo.traffic.log : log;
  $('#eventlog').innerHTML = entries.length ? entries.map(e => `<div><b>${e.time}</b>${e.msg}</div>`).join('') : '<div class="empty">Nenhuma ocorrência registrada ainda.</div>';
  if (selectedId) {
    if (mode === 'live') { const e = lastEntities.find(x => x.id === selectedId); if (e) entityPanel(e); }
    else if (demo) { const w = demo.workers.find(x => x.id === selectedId); const v = [...demo.robots, ...demo.forklifts].find(x => x.id === selectedId); if (w) demoPanel(w, true); else if (v) demoPanel(v, false); }
  }
}
/** "Ver pela câmera" names the selected AMR and shows when its camera is the one on screen. */
function onboardButton() {
  const id = selectedAmrId(), on = !!id && view.kind === 'onboard' && view.id === id;
  const html = `${on ? 'Exibindo a câmera do' : 'Ver pela câmera do'} ${id ? amrLabel(id) : 'AMR'} <span class="key">V</span>`;
  const b = $('#missionOnboard'); if (b.innerHTML !== html) b.innerHTML = html;
  b.classList.toggle('active', on);
}
function publishMetrics() {
  const sorted = [...frameTimes].sort((a, b) => a - b), p95 = sorted[Math.floor(sorted.length * .95)] ?? 0;
  const info = renderer.renderer.info;
  const metrics = { mode, fps, frame_ms_p95: p95, fps_cap: FPS_CAP, pixel_ratio: ratio, draw_calls: info.render.calls, triangles: info.render.triangles,
    static_instances: instancesStatic, dynamic_instances: batch.total, walls, walls_shown: Object.keys(wallsShown).filter(k => wallsShown[k]), connection: live?.connection ?? null, ws_hz: live?.receivedHz ?? null,
    ws_latency_ms: live?.latencyMs ?? null, sim_status: live?.latest?.simulation.status ?? null, rtf: live?.latest?.simulation.real_time_factor ?? null,
    entities: mode === 'live' ? lastEntities.length : (demo ? demo.workers.length + demo.robots.length + demo.forklifts.length : 0),
    obstacles: mode === 'live' ? live?.latest?.obstacles.length ?? 0 : demo?.loose.length ?? 0, view: view.kind, view_id: view.id,
    viewport: rendered?.viewport ?? null, camera_position: rendered?.position.map(v => +v.toFixed(4)) ?? null,
    // Robot pose the onboard camera was placed from in this same frame (validation of the mount).
    onboard_base: view.kind === 'onboard' ? onboardBase : null, render_sim_time: live?.renderTime ?? null,
    camera_forward: rendered ? new Vector3(0, 0, -1).applyQuaternion(renderer.camera.quaternion).toArray().map(v => +v.toFixed(7)) : null, js_heap_mb: (performance as unknown as { memory?: { usedJSHeapSize: number } }).memory?.usedJSHeapSize ? (performance as unknown as { memory: { usedJSHeapSize: number } }).memory.usedJSHeapSize / 1e6 : null };
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
    // Onboard camera mount exported from the URDF (export_robot_cameras.py); optional.
    onboardMount = await fetch('/robots/amr.cameras.json').then(r => r.ok ? r.json() : null).then(d => d?.cameras?.[0] ?? null).catch(() => null);
    minimap.setScene(scene);
    buildPanels(); bindControls();
    stage = stageRect(); window.addEventListener('resize', () => { stage = stageRect(); });
    camera.setView(scene.views.overview!); camera.state.yaw = camera.desired.yaw; camera.state.pitch = camera.desired.pitch;
    camera.state.distance = camera.desired.distance; camera.state.target = [...camera.desired.target];
    if (mode === 'live') { startLive(); vision = new VisionView(PERCEPTION, document.body); } else startDemo();
    document.body.dataset.ready = 'true';
    $('#busy').style.opacity = '0'; setTimeout(() => $('#busy').remove(), 780);
    requestAnimationFrame(render);
  } catch (error) {
    $('#busyText').textContent = 'FALHA AO CARREGAR: ' + String((error as Error).message).toUpperCase();
  }
}
void boot();
window.addEventListener('pagehide', () => { live?.stop(); vision?.dispose(); renderer.dispose(); });
