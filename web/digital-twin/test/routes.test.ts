import { describe, it, expect } from 'vitest';
import { mapToWorld, worldToMap } from '../src/rendering/coordinates/CoordinateSystem';
import { RouteOverlay, completedIndex, renderBlocker } from '../src/rendering/routes/RouteOverlay';
import { parseRouteGraph, parseRouteStatus } from '../src/core/types/validation';
import type { RouteGraph } from '../src/core/types/route-graph';
import type { RouteExecution, RouteStatus } from '../src/core/types/route-status';

const registration = { id: 'reg1', parent_frame: 'world' as const, child_frame: 'map' as const, x: 11.0375, y: 0.065, yaw: 0.0145, valid: true };
const graph: RouteGraph = {
  protocol_version: '1.0', message_type: 'route_graph', graph_version: 'g1', map_version: 'm1', frame_id: 'world', registration,
  nodes: [{ id: 'a', type: 'INTERSECTION', position: [-8.7, 0], yaw: null, status: 'validated', reason: '' },
    { id: 'b', type: 'INTERSECTION', position: [-4.5, 0], yaw: null, status: 'validated', reason: '' },
    { id: 's', type: 'STATION', position: [-4.5, 5], yaw: 1.57, status: 'unmapped', reason: '' }],
  connections: [{ id: 'a--b', source: 'a', target: 'b', directions: ['a->b', 'b->a'], kind: 'corridor', corridor_id: 'main', width: 4, distance: 4.2, status: 'validated', zones: [] },
    { id: 'b--s', source: 'b', target: 's', directions: ['b->s', 's->b'], kind: 'station_access', corridor_id: null, width: 2, distance: 5, status: 'unmapped', zones: [] }],
  stations: [{ id: 's', label: 'S', dock_node: 's', status: 'unmapped', reason: '' }],
};
const toMap = (x: number, y: number) => worldToMap(registration, x, y);
const route: RouteExecution = {
  protocol_version: '1.0', message_type: 'route_status', route_id: 'r1', status: 'EXECUTING', origin: 'current', destination: 'b',
  algorithm: 'astar', graph_version: 'g1', map_version: 'm1', registration_id: 'reg1', selected_nodes: ['robot_start', 'a', 'b'],
  selected_edges: ['robot_start->a', 'a->b'], map_frame_path: [toMap(-11, 0), toMap(-8.7, 0), toMap(-4.5, 0)],
  map_frame_waypoints: [[...toMap(-8.7, 0), 0.0145], [...toMap(-4.5, 0), 0.0145]], waypoint_nodes: ['a', 'b'], current_waypoint: 1,
  completed_waypoints: 1, total_waypoints: 2, estimated_distance_m: 6.5, distance_remaining_m: 4.2, execution_progress: .35, replans: 0,
  blocked_connections: ['b--s'], navigation_action_status: 'EXECUTING', failure_reason: null, detail: null, stamp: 1, updated_at: '2026-10-29T12:00:00+00:00',
};
const status = (r: RouteExecution | null): RouteStatus => ({ protocol_version: '1.0', message_type: 'route_status_snapshot', availability: 'online', received_at: '2026-10-29T12:00:00+00:00', age_seconds: .2, route: r });

describe('map/world registration for route rendering', () => {
  it('inverts the verified SE(2) registration, including rotation', () => {
    for (const [x, y] of [[0, 0], [-11, 0], [8.1, 8.5], [-8.7, -8.5]] as const) {
      const [mx, my] = worldToMap(registration, x, y);
      const [wx, wy] = mapToWorld(registration, mx, my);
      expect(Math.hypot(wx - x, wy - y)).toBeLessThan(1e-9);
    }
    // Same numbers the Python registration produces for the AMR spawn (world -11, 0).
    const [mx, my] = worldToMap(registration, -11, 0);
    expect(mx).toBeCloseTo(0.03866, 4); expect(my).toBeCloseTo(-0.09450, 4);
    // A pure +11 m translation would misplace a far point by ≈ 0.27 m: rotation matters.
    const [fx, fy] = mapToWorld(registration, 19.1, 0.2);
    expect(Math.hypot(fx - (19.1 - 11), fy - 0.2)).toBeGreaterThan(.1);
  });
});

describe('route overlay', () => {
  it('validates contracts and renders map-frame routes in world coordinates', () => {
    expect(parseRouteGraph(graph)).toBe(graph);
    expect(() => parseRouteStatus({ ...status(route), protocol_version: '2.0' })).toThrow();
    const overlay = new RouteOverlay();
    overlay.setGraph(graph);
    overlay.setStatus(parseRouteStatus(status(route)));
    expect(overlay.rendered.visible).toBe(true);
    expect(overlay.rendered.world_path[0]![0]).toBeCloseTo(-11, 6);
    expect(overlay.rendered.world_path[2]![0]).toBeCloseTo(-4.5, 6);
    expect(overlay.rendered.completed_index).toBe(1);
    expect(completedIndex({ ...route, status: 'COMPLETED' })).toBe(2);
    overlay.dispose();
  });
  it('refuses a misleading overlay when registration or versions do not match', () => {
    expect(renderBlocker(undefined, route)).toMatch(/graph unavailable/);
    expect(renderBlocker({ ...graph, registration: { ...registration, valid: false } }, route)).toMatch(/registration not verified/);
    expect(renderBlocker(graph, { ...route, registration_id: 'other' })).toMatch(/different map\/world registration/);
    expect(renderBlocker(graph, { ...route, map_version: 'm2' })).toMatch(/version differs/);
    const overlay = new RouteOverlay(); overlay.setGraph(graph);
    overlay.setStatus(status({ ...route, graph_version: 'stale' }));
    expect(overlay.rendered.visible).toBe(false);
    expect(overlay.rendered.reason).toMatch(/version differs/);
    overlay.dispose();
  });
});
