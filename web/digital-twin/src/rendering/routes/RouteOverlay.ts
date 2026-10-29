import { BoxGeometry, BufferGeometry, Color, ConeGeometry, CylinderGeometry, Group, InstancedMesh, Matrix4, Mesh, MeshBasicMaterial, Quaternion, Vector3 } from 'three';
import type { RouteGraph } from '../../core/types/route-graph';
import type { RouteExecution, RouteStatus } from '../../core/types/route-status';
import { mapToWorld } from '../coordinates/CoordinateSystem';
import { disposeTree } from '../core/resources';

const STATUS_COLOURS: Record<string, string> = { validated: '#1f9d55', unmapped: '#e3a008', blocked: '#d92d20', invalid: '#9b2fae' };
const CAPACITY = 512;
const Z = 0.04;

/** Index into the route polyline up to which the robot has passed confirmed objectives. */
export function completedIndex(route: RouteExecution): number {
  if (route.status === 'COMPLETED') return route.map_frame_path.length - 1;
  if (route.completed_waypoints <= 0) return 0;
  const node = route.waypoint_nodes[Math.min(route.completed_waypoints, route.waypoint_nodes.length) - 1];
  return Math.max(0, node === undefined ? 0 : route.selected_nodes.indexOf(node));
}

/** Why a route must not be drawn over the world-frame scene, or null when it is consistent. */
export function renderBlocker(graph: RouteGraph | undefined, route: RouteExecution | null): string | null {
  if (!graph) return 'Route graph unavailable';
  if (!graph.registration.valid) return 'Map/world registration not verified';
  if (!route) return 'No route reported';
  if (route.registration_id !== graph.registration.id) return 'Route uses a different map/world registration';
  if (route.graph_version !== graph.graph_version || route.map_version !== graph.map_version) return 'Route graph or map version differs from the displayed graph';
  return null;
}

/** Read-only overlay: static graph built once per version, route geometry updated in place. */
export class RouteOverlay {
  readonly group = new Group();
  private readonly graphLayer = new Group();
  private readonly routeLayer = new Group();
  private readonly segment = new BoxGeometry(1, 1, 1);
  private readonly marker = new CylinderGeometry(0.16, 0.16, 0.06, 16);
  private readonly done: InstancedMesh;
  private readonly remaining: InstancedMesh;
  private readonly blocked: InstancedMesh;
  private readonly waypoints: InstancedMesh;
  private readonly start = new Mesh(new ConeGeometry(0.25, 0.5, 16), new MeshBasicMaterial({ color: '#16a34a' }));
  private readonly goal = new Mesh(new ConeGeometry(0.25, 0.5, 16), new MeshBasicMaterial({ color: '#2563eb' }));
  graph?: RouteGraph;
  rendered: { visible: boolean; reason: string | null; route_id: string | null; status: string | null; world_path: number[][]; completed_index: number } =
    { visible: false, reason: 'Route graph unavailable', route_id: null, status: null, world_path: [], completed_index: 0 };

  constructor() {
    this.group.name = 'route-overlay';
    const instanced = (colour: string, geometry: BufferGeometry = this.segment) => {
      const mesh = new InstancedMesh(geometry, new MeshBasicMaterial({ color: colour, transparent: true, opacity: .9 }), CAPACITY);
      mesh.count = 0; mesh.frustumCulled = false; this.routeLayer.add(mesh); return mesh;
    };
    this.done = instanced('#64748b'); this.remaining = instanced('#2563eb');
    this.blocked = instanced('#d92d20'); this.waypoints = instanced('#f8fafc', this.marker);
    for (const cone of [this.start, this.goal]) { cone.rotation.x = Math.PI / 2; this.routeLayer.add(cone); }
    this.routeLayer.visible = false;
    this.group.add(this.graphLayer, this.routeLayer);
  }

  setGraph(graph: RouteGraph) {
    if (this.graph?.graph_version === graph.graph_version) return;
    disposeTree(this.graphLayer);
    this.graph = graph;
    const nodes = new Map(graph.nodes.map(n => [n.id, n]));
    const byStatus = new Map<string, [number, number, number, number, number][]>();
    for (const c of graph.connections) {
      const a = nodes.get(c.source)!.position, b = nodes.get(c.target)!.position;
      byStatus.set(c.status, [...(byStatus.get(c.status) ?? []), [a[0], a[1], b[0], b[1], .06]]);
    }
    for (const [status, segments] of byStatus) {
      const mesh = new InstancedMesh(this.segment.clone(), new MeshBasicMaterial({ color: STATUS_COLOURS[status], transparent: true, opacity: .55 }), segments.length);
      segments.forEach((s, i) => mesh.setMatrixAt(i, segmentMatrix(s[0], s[1], s[2], s[3], s[4], Z / 2)));
      mesh.name = `graph-${status}`; mesh.frustumCulled = false; this.graphLayer.add(mesh);
    }
    const stations = graph.nodes.filter(n => n.type === 'STATION');
    const docks = new InstancedMesh(new CylinderGeometry(.3, .3, .04, 20), new MeshBasicMaterial(), stations.length);
    stations.forEach((n, i) => {
      docks.setMatrixAt(i, new Matrix4().compose(new Vector3(n.position[0], n.position[1], Z / 2),
        new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2), new Vector3(1, 1, 1)));
      docks.setColorAt(i, new Color(STATUS_COLOURS[n.status]));
    });
    docks.name = 'graph-stations'; docks.frustumCulled = false; this.graphLayer.add(docks);
  }

  setStatus(status: RouteStatus) {
    const route = status.route;
    const reason = renderBlocker(this.graph, route);
    this.rendered = { visible: false, reason, route_id: route?.route_id ?? null, status: route?.status ?? null, world_path: [], completed_index: 0 };
    if (reason || !route || !this.graph || route.map_frame_path.length < 2) { this.routeLayer.visible = false; return; }
    const r = this.graph.registration;
    const world = route.map_frame_path.map(p => mapToWorld(r, p[0], p[1]));
    const split = completedIndex(route);
    let d = 0, k = 0;
    for (let i = 0; i + 1 < world.length && i < CAPACITY; i++) {
      const [a, b] = [world[i]!, world[i + 1]!];
      if (i < split) this.done.setMatrixAt(d++, segmentMatrix(a[0], a[1], b[0], b[1], .14, Z));
      else this.remaining.setMatrixAt(k++, segmentMatrix(a[0], a[1], b[0], b[1], .18, Z));
    }
    this.done.count = d; this.remaining.count = k;
    const nodes = new Map(this.graph.nodes.map(n => [n.id, n]));
    let b = 0;
    for (const id of route.blocked_connections) {
      const c = this.graph.connections.find(item => item.id === id);
      if (!c || b >= CAPACITY) continue;
      const p = nodes.get(c.source)!.position, q = nodes.get(c.target)!.position;
      this.blocked.setMatrixAt(b++, segmentMatrix(p[0], p[1], q[0], q[1], .3, Z + .01));
    }
    this.blocked.count = b;
    const points = route.map_frame_waypoints.slice(0, CAPACITY).map(w => mapToWorld(r, w[0], w[1]));
    points.forEach((p, i) => this.waypoints.setMatrixAt(i, new Matrix4().compose(new Vector3(p[0], p[1], Z + .03),
      new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2), new Vector3(1, 1, 1))));
    this.waypoints.count = points.length;
    for (const mesh of [this.done, this.remaining, this.blocked, this.waypoints]) mesh.instanceMatrix.needsUpdate = true;
    this.start.position.set(world[0]![0], world[0]![1], .3);
    this.goal.position.set(world.at(-1)![0], world.at(-1)![1], .3);
    (this.goal.material as MeshBasicMaterial).color.set(route.status === 'COMPLETED' ? '#16a34a' : ['FAILED', 'BLOCKED', 'CANCELED'].includes(route.status) ? '#d92d20' : '#2563eb');
    this.routeLayer.visible = true;
    this.rendered = { ...this.rendered, visible: true, world_path: world, completed_index: split };
  }

  dispose() { disposeTree(this.group); this.segment.dispose(); this.marker.dispose(); }
}

function segmentMatrix(x0: number, y0: number, x1: number, y1: number, width: number, z: number) {
  const length = Math.hypot(x1 - x0, y1 - y0);
  return new Matrix4().compose(new Vector3((x0 + x1) / 2, (y0 + y1) / 2, z),
    new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.atan2(y1 - y0, x1 - x0)), new Vector3(Math.max(length, 1e-3), width, .02));
}
