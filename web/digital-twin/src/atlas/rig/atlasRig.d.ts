// Types for the ATLAS drawing module (atlasRig.js). Entities are the prototype's plain objects.
export interface Batch { cube: { push(...v: number[]): void }; cylinder: { push(...v: number[]): void }; sphere: { push(...v: number[]): void } }
export type Rig = any; // prototype worker/vehicle records (see atlasRig.js for the fields read)
export const FIELD: string[];
export function animateGait(w: Rig, dt: number, yawRate: number): void;
export function drawHuman(a: Batch, w: Rig, time: number, selected: boolean): void;
export function drawHumanFar(a: Batch, w: Rig): void;
export function drawRobot(a: Batch, r: Rig, time: number): void;
export function drawRobotTrue(a: Batch, r: Rig, time: number): void;
export function drawForklift(a: Batch, r: Rig, time: number): void;
export function drawForkliftMarks(a: Batch, r: Rig): void;
export function drawFieldSamples(a: Batch, r: Rig, samples: { s: number; p: { x: number; z: number; yaw: number } }[]): void;
export function drawLooseObject(a: Batch, ob: Rig): void;
export function drawPlacement(a: Batch, hover: { x: number; z: number; ok: boolean; yaw?: number } | null, w: number, d: number): void;
export function segDyn(a: Batch, x1: number, z1: number, x2: number, z2: number, w: number, y: number, col: string, em: number): void;
export function cylDyn(a: Batch, x: number, y: number, z: number, r: number, h: number, col: string, yaw?: number, emit?: number): void;
export function boxDyn(a: Batch, x: number, y: number, z: number, w: number, h: number, d: number, col: string, yaw?: number, emit?: number): void;
