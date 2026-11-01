import { instancesToGl } from './coords';
import type { InstancedRenderer, Shape } from './render/InstancedRenderer';

export interface Pose6 extends Array<number> { 0: number; 1: number; 2: number; 3: number; 4: number; 5: number }
export interface Solid { id: string; tag: string; x: number; y: number; length: number; width: number; yaw: number }
export interface Zone { name: string; pos: [number, number, number]; icon: string; desc: string; status: string; metric: string }
export interface View { label: string; yaw: number; pitch: number; distance: number; target: [number, number, number] }
export interface Station { id: string; label: string; kind: string; pose: number[]; zone: string }
export interface IndustrialCamera { id: string; label: string; position: [number, number, number]; yaw: number; pitch: number;
  horizontal_fov: number; vertical_fov: number; near: number; range: number; resolution: [number, number]; future_topic: string }
export interface RoleSpec { name: string; vest: string | null; helmet: string | null; hold?: string; shirt?: string; pants?: string; vmax: [number, number] }
export interface WorkerSpec { id: string; role: string; level_z: number; path: [number, number][]; acts: string[][]; face: (number | string | null)[];
  initial: { x: number; y: number; yaw: number; waypoint: number; direction: number }; max_speed: number; carry: boolean; hold: string | null;
  appearance: { skin: string; pants: string; shirt: string; hair: string; phase: number; seed: number; side_sign: number } }
export interface CampusScene {
  schema_version: string; scene_id: string;
  metadata: { title: string; geometry_sha256: string; scene_sha256: string; stock_volumes: number };
  building: { interior: { x_min: number; x_max: number; y_min: number; y_max: number }; size: [number, number]; height: number };
  layers: Record<string, Partial<Record<Shape, number[]>>>;
  solids: Solid[]; lanes: Record<string, { id: string; colour: string; points: [number, number][] }>;
  zones: Zone[]; views: Record<string, View>; stations: Station[]; spawns: { id: string; robot_id: string; pose: number[] }[];
  cameras: IndustrialCamera[];
  agents: { workers: WorkerSpec[]; forklifts: { id: string; loop: string; speed: number; start_distance: number; color: string;
    length: number; width: number; centre_offset: number }[]; roles: Record<string, RoleSpec>; acts: Record<string, { dur: number; label: string }> };
  obstacle_catalogue: Record<string, { label: string; length: number; width: number; height: number }>;
}

/** Layer toggles of the prototype, plus the LIVE wall cladding (hidden by default: ATLAS look). */
export const LAYER_GROUPS: Record<string, string[]> = {
  roof: ['roof'], racks: ['racks', 'cargo'], vehicles: ['equipment'], routes: ['routes'], enclosure: ['enclosure'],
};

export async function fetchScene(url: string, signal?: AbortSignal): Promise<CampusScene> {
  const response = await fetch(url, { signal });
  if (!response.ok) throw new Error(`Cena indisponível (${response.status})`);
  const scene = await response.json() as CampusScene;
  if (scene.schema_version !== '1.0' || scene.scene_id !== 'atlas_campus') throw new Error('Versão de cena incompatível');
  return scene;
}

export function buildStatic(renderer: InstancedRenderer, scene: CampusScene) {
  renderer.clearStatic();
  let instances = 0;
  for (const [layer, shapes] of Object.entries(scene.layers)) {
    const converted: Partial<Record<Shape, Float32Array>> = {};
    for (const [shape, values] of Object.entries(shapes)) {
      converted[shape as Shape] = instancesToGl(values!);
      instances += values!.length / 13;
    }
    renderer.setStaticLayer(layer, converted);
  }
  return instances;
}
