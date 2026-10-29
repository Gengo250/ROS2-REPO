import { parseRouteGraph, parseRouteStatus } from '../types/validation';
import type { RouteStatus } from '../types/route-status';
import type { RouteOverlay } from '../../rendering/routes/RouteOverlay';

/** Independent read-only route extension: graph fetched once per version, status polled. */
export function observeRoutes(base: string, overlay: RouteOverlay | undefined, host: HTMLElement, signal: AbortSignal) {
  const get = async (path: string) => {
    const response = await fetch(`${base}${path}`, { signal: AbortSignal.any([signal, AbortSignal.timeout(3000)]) });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json() as Promise<unknown>;
  };
  let graphLoaded = false;
  async function poll() {
    if (signal.aborted) return;
    try {
      if (!graphLoaded) { overlay?.setGraph(parseRouteGraph(await get('/api/v1/routes/graph'))); graphLoaded = true; }
      const status = parseRouteStatus(await get('/api/v1/routes/status'));
      if (overlay?.graph && status.route && status.route.graph_version !== overlay.graph.graph_version) graphLoaded = false;
      overlay?.setStatus(status);
      host.replaceChildren(...describe(status, overlay?.rendered.reason ?? null));
    } catch { if (!signal.aborted) host.textContent = graphLoaded ? 'Route status unavailable.' : 'Route graph unavailable.'; }
    if (!signal.aborted) setTimeout(() => void poll(), 1000);
  }
  void poll();
}

function describe(status: RouteStatus, overlayReason: string | null): HTMLElement[] {
  const line = (text: string) => { const p = document.createElement('p'); p.textContent = text; return p; };
  const r = status.route;
  if (!r) return [line('No route has been reported.')];
  const rows = [line(`${r.status} · ${r.origin} → ${r.destination} · ${r.algorithm} · ${status.availability}${status.age_seconds !== null ? ` (${status.age_seconds.toFixed(1)} s)` : ''}`),
    line(`Objectives ${r.completed_waypoints}/${r.total_waypoints} · Progress ${(r.execution_progress * 100).toFixed(0)}%` +
      (r.estimated_distance_m !== null ? ` · Route ${r.estimated_distance_m.toFixed(1)} m` : '') +
      (r.distance_remaining_m !== null ? ` · Remaining ${r.distance_remaining_m.toFixed(1)} m` : '') + ` · Replans ${r.replans}`)];
  if (r.blocked_connections.length) rows.push(line(`Closed: ${r.blocked_connections.join(', ')}`));
  if (r.failure_reason) rows.push(line(`Reason: ${r.failure_reason}${r.detail ? ` · ${r.detail}` : ''}`));
  if (overlayReason) rows.push(line(`Overlay hidden: ${overlayReason}`));
  return rows;
}
