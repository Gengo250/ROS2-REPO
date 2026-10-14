import '../styles/main.css';
import { ApiClient } from '../core/api/client';
import { TwinStore } from '../core/state/store';
import { TwinSocket } from '../core/websocket/client';
import type { Robot } from '../core/types/twin-state';

const root = document.querySelector<HTMLElement>('#app')!;
root.innerHTML = `
  <header><div><p class="eyebrow">WAREHOUSE / PHASE 01</p><h1>Digital Twin</h1>
  <p class="subtitle">Live connection diagnostics</p></div><span class="tag">READ-ONLY TELEMETRY</span></header>
  <section class="system" aria-label="System connectivity">
    <article><h2>Gateway</h2><p id="gateway">Disconnected</p></article>
    <article><h2>ROS 2</h2><p id="ros">Unknown</p></article>
    <article><h2>Gazebo</h2><p id="gazebo">Unknown</p></article>
    <article><h2>Active scenario</h2><p id="scenario">Unknown</p></article>
  </section>
  <p id="notice" role="status">Connecting to Gateway…</p>
  <section class="telemetry"><div class="section-heading"><h2>Robot telemetry</h2>
    <span>Connected robots: <strong id="count">0</strong></span></div>
    <div id="robots"><p class="empty">Waiting for real robot telemetry.</p></div>
  </section>
  <footer><span id="last-update">Last update: —</span><span id="rate">Publication rate: —</span></footer>
  <p class="frame-note">Pose source: ROS odometry. Positions are local to the reported frame.
    Global warehouse localization is not available in Phase 1.</p>`;

const element = (id: string) => document.getElementById(id)!;
const store = new TwinStore();
const api = new ApiClient(import.meta.env.VITE_API_URL || 'http://127.0.0.1:8000');
const socket = new TwinSocket(import.meta.env.VITE_WS_URL ||
  'ws://127.0.0.1:8000/ws/v1/twin?protocol_version=1.0', store);
const abort = new AbortController();
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
  element('gateway').textContent = title(store.connection);
  element('gateway').dataset.connected = String(connected);
  element('ros').textContent = connected && store.status ? title(store.status.ros) : 'Unknown';
  element('gazebo').textContent = connected && state ? title(state.world.status) : 'Unavailable';
  element('scenario').textContent = state?.world.scenario ?? 'Unknown';
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
function stop() { abort.abort(); socket.stop(); unsubscribe(); window.removeEventListener('pagehide', stop); }
window.addEventListener('pagehide', stop);
if (import.meta.hot) import.meta.hot.dispose(stop);
