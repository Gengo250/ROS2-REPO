#!/usr/bin/env node
// Compile the ATLAS campus into the single authoritative scene file (ROS world frame).
// Usage: node compile.mjs            write config/atlas_campus.scene.json (+ web copy)
//        node compile.mjs --check    fail if either written file differs from a fresh compile
'use strict';
import { createHash } from 'node:crypto';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildCampus, LAYERS } from './atlas_campus.mjs';
import { navigation, enclosure } from './campus_navigation.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const OUTPUTS = [resolve(here, '../config/atlas_campus.scene.json'),
  resolve(here, '../../../web/digital-twin/public/scene/atlas_campus.scene.json')];
const r4 = v => Math.round(v * 1e4) / 1e4 || 0;

// ATLAS/WebGL (x east, y up, z south) -> ROS world (x east, y north, z up). Yaw is unchanged:
// ATLAS rotates local +x to (cos yaw, -sin yaw) in its x/z plane, i.e. (cos yaw, sin yaw) in x/y.
const point = ([x, z]) => [r4(x), r4(-z)];
function instances(values) {
  const out = [];
  for (let i = 0; i < values.length; i += 13) {
    const [gx, gy, gz, sx, sy, sz, r, g, b, e, yaw, tilt, roll] = values.slice(i, i + 13);
    // x, y, z, length (local x), width (local y), height, r, g, b, emission, yaw, tilt, roll
    out.push(r4(gx), r4(-gz), r4(gy), r4(sx), r4(sz), r4(sy), r4(r), r4(g), r4(b), r4(e), r4(yaw), r4(tilt), r4(roll));
  }
  return out;
}

const atlas = buildCampus();
const layers = Object.fromEntries(LAYERS.map(layer => [layer,
  Object.fromEntries(Object.entries(atlas.groups[layer]).filter(([, v]) => v.length).map(([shape, v]) => [shape, instances(v)]))]));
// LIVE addition (not in the prototype): physical wall cladding, hidden by default in the browser.
const wall = [0x8f / 255, 0xa3 / 255, 0xb4 / 255].map(r4);
layers.enclosure = { cube: enclosure().flatMap(([x, y, z, l, w, h, yaw]) => [x, y, z, l, w, h].map(r4).concat(wall, [0, r4(yaw), 0, 0])) };
const solids = atlas.solids.map((s, i) => ({ id: `solid_${String(i).padStart(3, '0')}`, tag: s.tag,
  x: r4(s.x), y: r4(-s.z), length: r4(s.w), width: r4(s.d), yaw: r4(s.yaw) }));
const lanes = Object.fromEntries(Object.entries(atlas.LOOPS).map(([id, l]) => [id,
  { id, one_way: true, closed: true, colour: l.color, points: l.pts.map(point) }]));
const zones = [
  { name: 'ARMAZENAGEM NORTE', pos: [-43, 9, -40], icon: '▤', desc: 'Porta-paletes em quatro níveis, corredores estruturados e posições de picking.', status: 'OPERAÇÃO NORMAL', metric: '8 corredores de picking' },
  { name: 'ARMAZENAGEM SUL', pos: [42, 9, 7], icon: '▤', desc: 'Alocação de volumes, paletes e separação de pedidos em alta densidade.', status: 'EM OPERAÇÃO', metric: '4 níveis de armazenagem' },
  { name: 'EXPEDIÇÃO', pos: [0, 6, 43], icon: '▥', desc: 'Oito docas, área de consolidação de carga e veículos estacionados.', status: '6 DOCAS DISPONÍVEIS', metric: '8 docas / 4 caminhões' },
  { name: 'PRODUÇÃO', pos: [7, 7, -30], icon: '◇', desc: 'Células industriais, esteiras automatizadas e equipamentos de inspeção.', status: 'LINHA ATIVA', metric: '8 células de processamento' },
  { name: 'MEZANINO', pos: [57, 9, -39], icon: '▦', desc: 'Controle administrativo elevado, estações operacionais e acesso por escada.', status: 'CONTROLE LOCAL', metric: '12 postos administrativos' },
  { name: 'RECEBIMENTO', pos: [-38, 6, 35], icon: '▱', desc: 'Zona de entrada, conferência e armazenamento temporário de mercadorias.', status: 'RECEBENDO CARGAS', metric: 'Zona de triagem ativa' },
].map(z => ({ ...z, pos: [r4(z.pos[0]), r4(-z.pos[2]), r4(z.pos[1])] }));
// ATLAS orbit presets converted to explicit world-frame eye/target points.
const views = Object.fromEntries(Object.entries({
  overview: { label: 'Visão panorâmica', yaw: .78, pitch: .76, distance: 208, target: [0, 0, 7] },
  storage: { label: 'Armazenagem', yaw: 1.18, pitch: .88, distance: 113, target: [-38, 1, -10] },
  docks: { label: 'Docas de carga', yaw: .10, pitch: .59, distance: 113, target: [0, 0, 46] },
  production: { label: 'Produção e esteiras', yaw: 1.28, pitch: .63, distance: 87, target: [7, 1, -27] },
  office: { label: 'Mezanino e controle', yaw: -.22, pitch: .70, distance: 76, target: [54, 4, -38] },
}).map(([id, v]) => [id, { label: v.label, yaw: v.yaw, pitch: v.pitch, distance: v.distance,
  target: [r4(v.target[0]), r4(-v.target[2]), r4(v.target[1])] }]));
const workers = atlas.workers.map(w => ({
  id: w.id, role: w.role, level_z: r4(w.y0), path: w.path.map(point), acts: w.acts, face: w.face,
  initial: { x: r4(w.x), y: r4(-w.z), yaw: r4(w.yaw), waypoint: w.wp, direction: w.dir },
  max_speed: r4(w.vmax), carry: w.carry, hold: w.hold,
  appearance: { skin: w.skin, pants: w.pants, shirt: w.shirt, hair: w.hair, phase: r4(w.phase), seed: r4(w.seed), side_sign: w.sideSign },
}));
const lift = atlas.VSPEC.lift;
const forklifts = atlas.forklifts.map(f => ({ ...f, start_distance: r4(f.start_distance),
  length: lift.L, width: lift.W, centre_offset: lift.cx, acceleration: lift.accel, braking: lift.brake,
  emergency_braking: lift.emerg, lateral_acceleration: lift.aLat, gap: lift.gap, curvature_window: lift.k }));

const geometry = { layers, solids };
const geometrySha = createHash('sha256').update(JSON.stringify(geometry)).digest('hex');
const scene = {
  schema_version: '1.0',
  scene_id: 'atlas_campus',
  metadata: {
    title: 'Centro Logístico Aurora',
    source: 'ATLAS_Galpao_Industrial_3D.html builders (seed 42930), src/atlas_scene/compiler',
    geometry_sha256: geometrySha,
    stock_volumes: atlas.stockCount,
  },
  coordinates: { frame: 'world', x: 'east', y: 'north', z: 'up', units: 'metres_radians',
    webgl: 'X = x, Y = z, Z = -y; yaw unchanged (rotation about +Y)' },
  building: { interior: { x_min: -72.65, x_max: 72.65, y_min: -53.5, y_max: 53.65 }, size: [146, 108], height: 14.5 },
  instance_fields: ['x', 'y', 'z', 'length', 'width', 'height', 'r', 'g', 'b', 'emission', 'yaw', 'tilt', 'roll'],
  layers,
  solids,
  lanes,
  zones,
  views,
  agents: { workers, forklifts, roles: atlas.ROLES, acts: atlas.ACTS },
  ...navigation,
  obstacle_catalogue: {
    cone: { label: 'Cone', length: .42, width: .42, height: .7 },
    box: { label: 'Caixa', length: .62, width: .52, height: .5 },
    pallet: { label: 'Palete', length: 1.22, width: 1.02, height: 1.1 },
  },
};
scene.metadata.scene_sha256 = createHash('sha256').update(JSON.stringify(scene)).digest('hex');
const text = JSON.stringify(scene) + '\n';

if (process.argv.includes('--check')) {
  for (const path of OUTPUTS) {
    if (await readFile(path, 'utf8').catch(() => '') !== text) throw new Error(`Scene drift: ${path}; run compile.mjs`);
  }
  console.log('scene up to date', geometrySha.slice(0, 16));
} else {
  for (const path of OUTPUTS) { await mkdir(dirname(path), { recursive: true }); await writeFile(path, text); }
  const counts = Object.fromEntries([...LAYERS, 'enclosure'].map(l => [l, Object.values(layers[l]).reduce((s, v) => s + v.length / 13, 0)]));
  console.log(JSON.stringify({ geometry_sha256: geometrySha, bytes: text.length, instances: counts, solids: solids.length,
    workers: workers.length, forklifts: forklifts.length }));
}
