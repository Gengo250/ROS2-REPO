// Camera render worker page (camera.html): renders onboard-camera frames on request.
//
// URL parameters: ``channel`` (WebSocket of the requesting service), optional ``gateway`` (HTTP base of
// the Digital Twin Gateway: LIVE mode) and ``scene`` (scene URL when there is no Gateway).
//  * render  {job_id, robot_id, width, height, night, ids, dt, state}: render the given world snapshot
//    (dataset generation drives the lite engine itself and sends its exact state);
//  * capture {job_id, robot_id, width, height, night}: render the newest backend world-state received
//    from the Gateway (LIVE). The frame is stamped with that state's simulation time and sequence.
//    Refused (error reply, no image) when the backend state is not online or the engine's scene differs.
// Reply: one binary message = uint32 LE header length + JSON header + RGBA pixels [+ RGBA instance codes]
// (rows bottom-up), or a JSON {type:'error'} text message.
import { CaptureRenderer, type CaptureState } from './capture';
import { fetchScene, type CampusScene } from '../atlas/scene';
import type { MountedCamera } from '../atlas/cameraPoses';
import { LiveWorld } from '../atlas/live/LiveWorld';

const params = new URLSearchParams(location.search);
const channelUrl = params.get('channel');
const gateway = params.get('gateway');
const status = document.getElementById('status')!;
const say = (text: string) => { status.textContent = text; };

interface Job { type: 'render' | 'capture'; job_id: number; robot_id: string; width: number; height: number; night?: boolean; ids?: boolean;
  dt?: number; state?: CaptureState }

async function boot() {
  if (!channelUrl) throw new Error('missing channel parameter');
  const scene: CampusScene = gateway ? await fetchScene(`${gateway}/api/v1/world/scene`) : await fetchScene(params.get('scene') ?? '/scene/atlas_campus.scene.json');
  const mount: MountedCamera = (await (await fetch('/robots/amr.cameras.json', { cache: 'no-cache' })).json()).cameras[0];
  const canvas = document.getElementById('gl') as HTMLCanvasElement;
  const capture = new CaptureRenderer(canvas, scene, mount);
  const gl = capture.instanced.renderer.getContext();
  const info = gl.getExtension('WEBGL_debug_renderer_info');
  const webgl = info ? String(gl.getParameter(info.UNMASKED_RENDERER_WEBGL)) : String(gl.getParameter(gl.RENDERER));
  let live: LiveWorld | null = null;
  if (gateway) { live = new LiveWorld(gateway, `${gateway.replace(/^http/, 'ws')}/ws/v1/world?protocol_version=1.0`); live.start(); }
  let lastSim: number | null = null;
  const socket = new WebSocket(channelUrl);
  socket.binaryType = 'arraybuffer';
  const reply = (header: Record<string, unknown>, ...buffers: Uint8Array[]) => {
    const head = new TextEncoder().encode(JSON.stringify(header));
    const out = new Uint8Array(4 + head.length + buffers.reduce((n, b) => n + b.length, 0));
    new DataView(out.buffer).setUint32(0, head.length, true); out.set(head, 4);
    let offset = 4 + head.length; for (const b of buffers) { out.set(b, offset); offset += b.length; }
    socket.send(out);
  };
  const fail = (job: Job | null, reason: string) => socket.send(JSON.stringify({ type: 'error', job_id: job?.job_id ?? null, robot_id: job?.robot_id ?? null, reason }));
  socket.onopen = () => {
    socket.send(JSON.stringify({ type: 'hello', webgl, mode: gateway ? 'live' : 'jobs', scene_sha256: scene.metadata.scene_sha256,
      static_instances: capture.staticInstanceCount, user_agent: navigator.userAgent }));
    say(`ready · ${webgl}`);
  };
  socket.onclose = () => say('channel closed');
  socket.onmessage = event => {
    let job: Job | null = null;
    try {
      job = JSON.parse(String(event.data)) as Job;
      let state: CaptureState, meta: Record<string, unknown> = {};
      if (job.type === 'render') {
        if (!job.state) throw new Error('render job without state');
        state = job.state;
      } else if (job.type === 'capture') {
        const s = live?.latest;
        if (!live || !s) throw new Error('no world state from the Gateway yet');
        if (live.connection !== 'connected' || s.simulation.availability !== 'online' || s.simulation.sim_time === null)
          throw new Error(`backend world state not online (${live.connection}, ${s.simulation.availability})`);
        if (s.simulation.scene_sha256 && s.simulation.scene_sha256 !== scene.metadata.scene_sha256)
          throw new Error('engine scene differs from the loaded scene geometry');
        state = { entities: s.entities as CaptureState['entities'], obstacles: s.obstacles as CaptureState['obstacles'], sim_time: s.simulation.sim_time };
        meta = { world_sequence: s.sequence, world_session: s.session_id, engine_session: s.simulation.engine_session_id,
          world_timestamp: s.timestamp, server_wall_time: s.server_wall_time };
      } else return;
      const dt = job.dt ?? (lastSim === null ? 0 : Math.max(0, Math.min(.5, state.sim_time - lastSim)));
      lastSim = state.sim_time;
      const frame = capture.capture(state, job.robot_id, { width: job.width, height: job.height, night: !!job.night, ids: !!job.ids, dt });
      reply({ type: 'frame', job_id: job.job_id, robot_id: job.robot_id, sim_time: state.sim_time, width: frame.width, height: frame.height,
        rows: 'bottom_up', has_ids: !!frame.ids, instances: frame.instances, camera: frame.camera, base: frame.base,
        render_ms: frame.render_ms, scene_sha256: scene.metadata.scene_sha256, ...meta }, frame.rgb, ...(frame.ids ? [frame.ids] : []));
    } catch (error) {
      fail(job, String((error as Error).message ?? error).slice(0, 300));
    }
  };
}

boot().catch(error => { say(`failed: ${(error as Error).message}`); console.error(error); });
