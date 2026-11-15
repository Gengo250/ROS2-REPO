// Labelled objects of the static scene, derived from the compiled scene itself (no hand-placed boxes):
//  * floor pallets: every 'Palete no chão' solid (the builder's floor pallet envelope) and the cargo
//    instances standing inside its footprint below 2 m (wooden base plus its load) → one pallet;
//  * boxes on the inspection tables: every cargo instance on a 'Mesa de inspeção' footprint at
//    table-top height → one box each.
// Racked loads stay part of the rack structure (background). User obstacles and the moving bodies
// (people, forklifts, other AMRs) are labelled at capture time from the world state.
import type { Shape } from '../atlas/render/InstancedRenderer';
import type { CampusScene, Solid } from '../atlas/scene';

export interface LabelledInstance { id: string; kind: 'pallet' | 'box'; layer: string; shape: Shape; instances: number[] }

const STRIDE = 13;

function inside(solid: Solid, x: number, y: number, margin = .05) {
  const c = Math.cos(solid.yaw), s = Math.sin(solid.yaw), dx = x - solid.x, dy = y - solid.y;
  const u = c * dx + s * dy, v = -s * dx + c * dy;
  return Math.abs(u) <= solid.length / 2 + margin && Math.abs(v) <= solid.width / 2 + margin;
}

export function staticInstances(scene: CampusScene): LabelledInstance[] {
  const cargo = scene.layers.cargo?.cube ?? [];
  const out: LabelledInstance[] = [];
  const centre = (i: number) => [cargo[i * STRIDE]!, cargo[i * STRIDE + 1]!, cargo[i * STRIDE + 2]!] as const;
  const n = cargo.length / STRIDE;
  for (const solid of scene.solids) {
    if (solid.tag === 'Palete no chão') {
      const members: number[] = [];
      for (let i = 0; i < n; i++) { const [x, y, z] = centre(i); if (z < 2 && inside(solid, x, y)) members.push(i); }
      if (members.length) out.push({ id: `${solid.id}_pallet`, kind: 'pallet', layer: 'cargo', shape: 'cube', instances: members });
    } else if (solid.tag === 'Mesa de inspeção') {
      let k = 0;
      for (let i = 0; i < n; i++) {
        const [x, y, z] = centre(i);
        if (z > .7 && z < 1.5 && inside(solid, x, y)) out.push({ id: `${solid.id}_box_${k++}`, kind: 'box', layer: 'cargo', shape: 'cube', instances: [i] });
      }
    }
  }
  return out;
}
