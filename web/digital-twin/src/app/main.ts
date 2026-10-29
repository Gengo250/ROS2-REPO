import '../styles/main.css';
import { observeNavigation } from '../core/api/navigation';
import { observeRoutes } from '../core/api/routes';
import { WarehouseViewer } from '../rendering/core/WarehouseViewer';
import { ApiClient } from '../core/api/client';
import { TwinStore } from '../core/state/store';
import { TwinSocket } from '../core/websocket/client';
import type { Robot } from '../core/types/twin-state';

const root = document.querySelector<HTMLElement>('#app')!;
root.innerHTML = `
  <header><div><p class="eyebrow">WAREHOUSE / PHASE 05</p><h1>Digital Twin</h1>
  <p class="subtitle">Industrial environment · Live diagnostics</p></div><span class="tag">READ-ONLY TELEMETRY</span></header>
  <section class="viewer-panel" aria-label="Warehouse visualization">
    <div class="viewer-toolbar"><div><h2>Warehouse / 3D</h2><span id="scene-status" role="status">Initializing renderer…</span></div>
      <div class="camera-buttons"><button data-view="overview">Overview</button><button data-view="robot">Robot focus</button>
        <button data-view="top">Top-down</button><button data-view="overview">Reset camera</button></div></div>
    <div id="warehouse"></div>
    <div class="viewer-bottom"><label>Scene preview <select id="scene-select"><option value="">Active configuration</option>
      <option>normal</option><option>person_crossing</option><option>obstacle_in_aisle</option><option>corridor_blocked</option></select></label>
      <label><input id="structure" type="checkbox" checked> Walls & beams</label><button id="scene-retry">Retry</button>
      <span id="render-metrics"></span></div>
    <p class="view-note">Drag to orbit · Right-drag to pan · Scroll to zoom. The active scene follows Gazebo ground truth. Other scenarios show configured previews. Scene preview does not change Gazebo.</p>
  </section>
  <section class="system" aria-label="System connectivity">
    <article><h2>Gateway</h2><p id="gateway">Disconnected</p></article>
    <article><h2>ROS 2</h2><p id="ros">Unknown</p></article>
    <article><h2>Gazebo</h2><p id="gazebo">Unknown</p></article>
    <article><h2>Active scenario</h2><p id="scenario">Unknown</p></article>
  </section>
  <p id="notice" role="status">Connecting to Gateway…</p>
  <section class="telemetry"><h2>Navigation · Read-only</h2><div id="navigation-status" role="status">Waiting for navigation diagnostics.</div></section>
  <section class="telemetry"><h2>Warehouse route · Read-only</h2><div id="route-status" role="status">Waiting for route status.</div>
    <p class="frame-note">Graph lines: green validated, amber unmapped, red blocked. Route planned in the Nav2 map frame, drawn through the verified map/world registration.</p></section>
  <section class="telemetry"><h2>Simulation ground truth · World</h2><div id="dynamic-entities">Waiting for simulation poses.</div></section>
  <section class="telemetry"><div class="section-heading"><h2>Robot telemetry</h2>
    <span>Connected robots: <strong id="count">0</strong></span></div>
    <div id="robots"><p class="empty">Waiting for real robot telemetry.</p></div>
  </section>
  <footer><span id="last-update">Last update: —</span><span id="rate">Publication rate: —</span></footer>
  <p class="frame-note">Pose source: ROS odometry. Positions are local to the reported frame.
    Navigation uses its own map-frame localization. The 3D view uses separate Gazebo world ground truth.</p>`;

const element = (id: string) => document.getElementById(id)!;
let viewer: WarehouseViewer | undefined;
try {
  viewer = new WarehouseViewer(element('warehouse'), element('scene-status'), element('render-metrics'),
    import.meta.env.VITE_API_URL || 'http://127.0.0.1:8000');
  void viewer.load();
} catch (error) { element('scene-status').textContent = `WebGL unavailable: ${(error as Error).message}`; }
const selection = element('scene-select') as HTMLSelectElement;
const structure = element('structure') as HTMLInputElement;
const loadScene = () => { structure.checked = true; void viewer?.load(selection.value || undefined); };
selection.addEventListener('change', loadScene);
element('scene-retry').addEventListener('click', loadScene);
structure.addEventListener('change', () => viewer?.structure(structure.checked));
document.querySelectorAll<HTMLButtonElement>('[data-view]').forEach(button =>
  button.addEventListener('click', () => viewer?.view(button.dataset.view!)));
let configuredScenario: string | null | undefined;
const store = new TwinStore();
const api = new ApiClient(import.meta.env.VITE_API_URL || 'http://127.0.0.1:8000');
const socket = new TwinSocket(import.meta.env.VITE_WS_URL ||
  'ws://127.0.0.1:8000/ws/v2/twin?protocol_version=2.0', store);
const abort = new AbortController();
observeNavigation(import.meta.env.VITE_API_URL || 'http://127.0.0.1:8000', element('navigation-status'), abort.signal);
observeRoutes(import.meta.env.VITE_API_URL || 'http://127.0.0.1:8000', viewer?.routes, element('route-status'), abort.signal);
const cards = new Map<string, HTMLElement>();
const number = (value: number | undefined) => value === undefined ? '—' : value.toFixed(4);
const title = (value: string) => value.charAt(0).toUpperCase() + value.slice(1);

function robotCard(robot: Robot, connected: boolean) {
  let card = cards.get(robot.id);
  if (!card) {
    card = document.createElement('article');
    card.className = 'robot-card';
    card.innerHTML = `<div class="section-heading"><h3></h3><span data-value="status"></span></div>
      <div class="robot-meta" data-value="frames"></div><div class="measurements">
      <div><h4>Position · m</h4><dl><dt>X</dt><dd data-value="x"></dd>
      <dt>Y</dt><dd data-value="y"></dd><dt>Z</dt><dd data-value="z"></dd></dl></div>
      <div><h4>Velocity · m/s, rad/s</h4><dl><dt>Linear (x, y, z)</dt><dd data-value="linear"></dd>
      <dt>Angular (x, y, z)</dt><dd data-value="angular"></dd></dl></div></div>
      <p class="robot-meta" data-value="receipt"></p>`;
    card.querySelector('h3')!.textContent = `Robot: ${robot.id}`;
    cards.set(robot.id, card);
  }
  const set = (key: string, value: string) => {
    card!.querySelector<HTMLElement>(`[data-value="${key}"]`)!.textContent = value;
  };
  set('status', connected ? title(robot.status) : 'Disconnected · last known values');
  set('frames', `Pose frame: ${robot.frame_id ?? 'unknown'} · Velocity frame: ${robot.child_frame_id ?? 'unknown'} · Source: ${robot.pose_source}`);
  for (const axis of ['x', 'y', 'z'] as const) set(axis, number(robot.pose?.position[axis]));
  for (const kind of ['linear', 'angular'] as const) {
    set(kind, robot.velocity ? ['x', 'y', 'z'].map((a) =>
      number(robot.velocity![kind][a as 'x' | 'y' | 'z'])).join(', ') : '—');
  }
  set('receipt', robot.received_at ? `Last odometry: ${robot.received_at} · Age: ${number(robot.age_seconds ?? undefined)} s` : 'No odometry received');
  return card;
}

const unsubscribe = store.subscribe(() => {
  const connected = store.connection === 'connected';
  const state = store.twin;
  if (state && 'simulation' in state) {
    viewer?.accept(state, connected);
    element('dynamic-entities').replaceChildren(...[...state.simulation_robots, ...state.humans, ...state.obstacles].map(entity => {
      const row = document.createElement('p');
      row.textContent = `${entity.id} · ${connected ? entity.status : 'disconnected'} · ${entity.frame_id} · ` +
        (entity.pose ? `(${number(entity.pose.position.x)}, ${number(entity.pose.position.y)}, ${number(entity.pose.position.z)}) m · simulation ${number(entity.source_timestamp ?? undefined)} s` : 'No ground truth received');
      return row;
    }));
  }
  element('gateway').textContent = title(store.connection);
  element('gateway').dataset.connected = String(connected);
  element('ros').textContent = connected && store.status ? title(store.status.ros) : 'Unknown';
  element('gazebo').textContent = connected && state ? title(state.world.status) : 'Unavailable';
  element('scenario').textContent = state?.world.scenario ?? 'Unknown';
  if (state && configuredScenario !== state.world.scenario) {
    const previousScenario = configuredScenario;
    configuredScenario = state.world.scenario;
    if (previousScenario !== undefined && !selection.value && configuredScenario) void viewer?.load(configuredScenario);
  }
  element('notice').textContent = connected
    ? (state?.robots.some((r) => r.status === 'online') ? 'Receiving live telemetry.' : 'Gateway connected. Waiting for available odometry.')
    : (store.error || 'Connecting to Gateway…');
  element('count').textContent = String(connected ? state?.robots.filter((r) => r.status === 'online').length ?? 0 : 0);
  if (state) {
    const active = new Set(state.robots.map((r) => r.id));
    for (const id of cards.keys()) if (!active.has(id)) cards.delete(id);
    element('robots').replaceChildren(...state.robots.map((r) => robotCard(r, connected)));
    element('last-update').textContent = `Last update: ${state.timestamp} · #${state.sequence}`;
    element('rate').textContent = `Publication rate: ${connected ? state.connection.measured_publication_hz.toFixed(1) : '—'} Hz`;
  }
});

// Bootstrap REST independently; WebSocket snapshots remain authoritative once live.
void api.status(abort.signal).then((status) => {
  if (store.connection !== 'connected') store.setStatus(status);
}).catch(() => { /* WebSocket retry owns visible connection state. */ });
socket.start();
function stop() { viewer?.dispose(); abort.abort(); socket.stop(); unsubscribe(); window.removeEventListener('pagehide', stop); }
window.addEventListener('pagehide', stop);
if (import.meta.hot) import.meta.hot.dispose(stop);
