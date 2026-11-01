// Types for the DEMO simulation (atlasDemo.js): the prototype's browser-side traffic and crew.
import type { Batch, Rig } from '../rig/atlasRig.js';
import type { CampusScene } from '../scene';
export const DEMO_LOOSE: Record<string, { w: number; d: number; label: string }>;
export interface Demo {
  robots: Rig[]; forklifts: Rig[]; workers: Rig[]; loose: Rig[];
  traffic: { stops: number; yields: number; detours: number; hits: number; log: { time: string; msg: string }[] };
  opts: { people: boolean; vehicles: boolean };
  readonly time: number;
  step(dt: number): void;
  draw(a: Batch, opts: { people?: boolean; vehicles?: boolean; fields?: boolean; selected?: Rig | null; eye?: number[] | null;
    tool?: string | null; hover?: { x: number; z: number; ok: boolean } | null }): void;
  setSensors(on: boolean): void;
  placementCheck(type: string, x: number, z: number): { ok: boolean; msg?: string };
  toolClick(type: string, x: number, z: number): string;
  clearUser(): number;
  vehStatus(r: Rig): string;
  workerStatus(w: Rig): string;
}
export function createDemo(scene: CampusScene, options?: { clock?: () => string; toast?: (message: string) => void }): Demo;
