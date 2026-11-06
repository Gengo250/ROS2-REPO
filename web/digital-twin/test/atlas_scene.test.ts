import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { HIDDEN_LAYERS, LAYER_GROUPS, wallVisibility, type CampusScene } from '../src/atlas/scene';
import { DynamicBatch } from '../src/atlas/render/InstancedRenderer';
import { createDemo } from '../src/atlas/demo/atlasDemo.js';

const scene = JSON.parse(readFileSync(new URL('../public/scene/atlas_campus.scene.json', import.meta.url), 'utf8')) as CampusScene;
const shell = scene.building.shell!;

/** World-frame eye of an orbit preset (OrbitCamera: yaw about +Y, WebGL z = -north). */
function eyeOf(view: { yaw: number; pitch: number; distance: number; target: [number, number, number] }) {
  const gx = view.target[0] + Math.cos(view.pitch) * Math.sin(view.yaw) * view.distance;
  const gz = -view.target[1] + Math.cos(view.pitch) * Math.cos(view.yaw) * view.distance;
  return { x: gx, y: -gz };
}

describe('building walls (presentation of the compiled shell)', () => {
  it('the compiled scene carries the four walls and their layers', () => {
    expect(Object.keys(shell.sides).sort()).toEqual(['east', 'north', 'south', 'west']);
    for (const side of Object.values(shell.sides)) expect(scene.layers[side.layer]?.cube?.length).toBeGreaterThan(0);
    expect(scene.layers.enclosure?.cube?.length).toBe(12 * 13);
    expect(shell.dock_doors.centres_x).toHaveLength(8);
  });
  it('full and cut modes ignore the camera; auto cuts only the walls facing an outside camera', () => {
    const inside = { x: 0, y: 0 }, southWest = { x: -150, y: -150 };
    for (const eye of [inside, southWest]) {
      expect(Object.values(wallVisibility(shell, 'full', eye))).toEqual([true, true, true, true]);
      expect(Object.values(wallVisibility(shell, 'cut', eye))).toEqual([false, false, false, false]);
    }
    expect(Object.values(wallVisibility(shell, 'auto', inside))).toEqual([true, true, true, true]);
    expect(wallVisibility(shell, 'auto', southWest)).toEqual({ walls_west: false, walls_east: true, walls_north: true, walls_south: false });
    // The default overview looks over two walls; the other two stay closed.
    const overview = wallVisibility(shell, 'auto', eyeOf(scene.views.overview!));
    expect(Object.values(overview).filter(Boolean)).toHaveLength(2);
  });
});

describe('route ownership', () => {
  it('the scripted vehicle loops are never a toggled or drawn layer', () => {
    expect(HIDDEN_LAYERS).toContain('routes');
    expect(Object.values(LAYER_GROUPS).flat()).not.toContain('routes');
    expect(Object.keys(scene.lanes).length).toBeGreaterThan(0);          // still available to the simulation
  });
  it('DEMO draws a route only for AMRs (entity kind), never for forklifts', () => {
    const demo = createDemo(scene);
    for (let i = 0; i < 120; i++) demo.step(1 / 60);
    const count = (routes: boolean, vehicles = true) => { const b = new DynamicBatch(); demo.draw(b, { routes, vehicles, people: false, fields: false }); return b.cube.count; };
    const amrs = demo.robots.filter(r => r.kind === 'amr').length;
    expect(amrs).toBe(demo.robots.length);
    expect(demo.forklifts.every(f => f.kind === 'lift')).toBe(true);
    const overlay = count(true) - count(false);
    expect(overlay).toBeGreaterThan(0);
    expect(overlay % amrs).toBe(0);                                         // the same route length per AMR
    // Hiding the forklifts removes their bodies only: the route overlay does not change.
    expect(count(true, false) - count(false, false)).toBe(overlay);
  });
});
