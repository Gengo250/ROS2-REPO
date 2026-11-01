// LIVE-only campus data that the demo prototype did not need: the physical wall
// enclosure, robot spawn, stations, industrial cameras and the navigation description.
// Everything here is in the ROS world frame (x east, y north, z up), metres and radians.
'use strict';

// The prototype draws cut-away walls (columns plus high panels) so the camera can see in.
// A real building is closed at robot height, so a 3 m cladding layer is added for
// collisions and floor-level scanners. The browser hides it by default (ATLAS look).
export function enclosure() {
  const boxes = []; // x, y, z, length, width, height, yaw
  const height = 3, z = height/2;
  boxes.push([-73, 0, z, .7, 108, height, 0], [73, 0, z, .7, 108, height, 0], [0, 54, z, 146, .7, height, 0]);
  // Dock wall: fill only between the eight dock doors (x = -56 + 16 i, 8 m wide).
  const doors = Array.from({ length: 8 }, (_, i) => -56+16*i);
  const gaps = [[-72.65, doors[0]-4.2], ...doors.slice(0, -1).map((x, i) => [x+4.2, doors[i+1]-4.2]), [doors.at(-1)+4.2, 72.65]];
  for (const [a, b] of gaps) boxes.push([(a+b)/2, -53.875, z, b-a, .25, height, 0]);
  return boxes;
}

const RACK_ROWS_WEST = [-64, -58, -52, -46, -40, -34, -28, -22];
const RACK_ROWS_EAST = [18, 24, 30, 36, 42, 48, 54, 60];
const mid = (a, b) => (a+b)/2;
// Aisles between neighbouring rack rows (6 m pitch, rack footprint ±1.87 m → 2.26 m clear).
const aisleXs = rows => rows.slice(0, -1).map((x, i) => mid(x, rows[i+1]));

export const navigation = {
  // Home pose: the open central hall, facing north along the spine.
  spawns: [{ id: 'amr_01_spawn', robot_id: 'amr_01', active: true, pose: [2.0, 0.0, 0.0, 0, 0, Math.PI/2] }],
  // Goal poses are where the robot centre stops. Every station is checked for a free
  // footprint plus clearance by the atlas_scene tests.
  stations: [
    ...[27, 33, 39, 45].map((y, i) => ({ id: `charger_0${i+1}`, label: `RECARGA ${i+1}`, kind: 'charging',
      pose: [63.4, y, 0, 0, 0, 0], zone: 'MEZANINO' })),
    { id: 'pick_north_w1', label: 'PICKING N-O 1', kind: 'pickup', pose: [-55, 36, 0, 0, 0, Math.PI/2], zone: 'ARMAZENAGEM NORTE' },
    { id: 'pick_north_w2', label: 'PICKING N-O 2', kind: 'pickup', pose: [-37, 36, 0, 0, 0, Math.PI/2], zone: 'ARMAZENAGEM NORTE' },
    { id: 'pick_north_e1', label: 'PICKING N-L 1', kind: 'pickup', pose: [39, 36, 0, 0, 0, Math.PI/2], zone: 'ARMAZENAGEM NORTE' },
    { id: 'pick_south_w1', label: 'PICKING S-O 1', kind: 'pickup', pose: [-61, 0, 0, 0, 0, -Math.PI/2], zone: 'ARMAZENAGEM SUL' },
    { id: 'pick_south_w2', label: 'PICKING S-O 2', kind: 'pickup', pose: [-25, 0, 0, 0, 0, -Math.PI/2], zone: 'ARMAZENAGEM SUL' },
    { id: 'pick_south_e1', label: 'PICKING S-L 1', kind: 'pickup', pose: [27, 0, 0, 0, 0, -Math.PI/2], zone: 'ARMAZENAGEM SUL' },
    { id: 'production_w', label: 'PRODUÇÃO', kind: 'dropoff', pose: [-12, 33, 0, 0, 0, 0], zone: 'PRODUÇÃO' },
    ...[-56, -24, 8, 40].map((x, i) => ({ id: `dock_0${2*i+1}`, label: `DOCA ${2*i+1}`, kind: 'dropoff',
      pose: [x, -49.5, 0, 0, 0, -Math.PI/2], zone: 'EXPEDIÇÃO' })),
    { id: 'receiving_01', label: 'RECEBIMENTO', kind: 'pickup', pose: [-45, -45.5, 0, 0, 0, Math.PI], zone: 'RECEBIMENTO' },
  ],
  // Industrial monitoring cameras: mount position, aiming point on the floor, optics.
  // Geometric coverage only; no image stream is simulated.
  cameras: [
    ['cam_01', 'CAM-01 · Armazenagem N-O', [-71, 51, 9], [-45, 36]],
    ['cam_02', 'CAM-02 · Armazenagem N-L', [36, 53.4, 9], [36, 33]],
    ['cam_03', 'CAM-03 · Produção', [-1, 53.4, 9], [-1, 33]],
    ['cam_04', 'CAM-04 · Armazenagem S-O', [-72.4, 0, 9], [-45, 2]],
    ['cam_05', 'CAM-05 · Armazenagem S-L', [72.4, 0, 9], [42, 2]],
    ['cam_06', 'CAM-06 · Hall central', [2, 22.5, 10], [2, 0]],
    ['cam_07', 'CAM-07 · Expedição O', [-40, -53.4, 8], [-35, -38]],
    ['cam_08', 'CAM-08 · Expedição L', [40, -53.4, 8], [35, -38]],
  ].map(([id, label, mount, target]) => {
    const dx = target[0]-mount[0], dy = target[1]-mount[1];
    return { id, label, position: mount, yaw: Math.round(Math.atan2(dy, dx)*1e4)/1e4,
      pitch: Math.round(Math.atan2(mount[2], Math.hypot(dx, dy))*1e4)/1e4,
      horizontal_fov: 1.48, vertical_fov: 0.9, near: 0.5, range: 45, resolution: [1920, 1080],
      future_topic: `/cameras/${id}/image_raw` };
  }),
  navigation: {
    // Axis-aligned corridors (centre, size); the graph builder places nodes where they
    // cross and validates every node and edge against the SLAM map.
    corridors: [
      { id: 'north_cross_aisle', centre: [0, 22.5], size: [138, 7], capacity: 3 },
      { id: 'south_cross_aisle', centre: [0, -20.5], size: [138, 8], capacity: 3 },
      { id: 'north_wall_aisle', centre: [-10, 49.3], size: [118, 7], capacity: 2 },
      { id: 'dock_aisle', centre: [0, -45.5], size: [136, 7], capacity: 3 },
      // Between the wall and the extinguishers at x = ±68; corner pillars at y = ±51 bound them.
      { id: 'west_wall_aisle', centre: [-70.4, 0], size: [4.4, 94], capacity: 1 },
      { id: 'east_wall_aisle', centre: [70.4, 0], size: [4.4, 94], capacity: 1 },
      // Front block: passages from the south cross aisle down to the dock aisle.
      { id: 'front_passage_w', centre: [-46, -33.5], size: [3, 17], capacity: 1 },
      { id: 'front_passage_c', centre: [-13, -33.5], size: [5, 17], capacity: 2 },
      { id: 'charger_aisle', centre: [63.4, 34], size: [3.2, 24], capacity: 1 },
      { id: 'production_access', centre: [-19.1, 29.5], size: [2.1, 9], capacity: 1 },
      { id: 'production_aisle', centre: [-4, 33], size: [30, 4.8], capacity: 1 },
      { id: 'spine_west', centre: [-1, 1.5], size: [4, 34], capacity: 2 },
      { id: 'spine_east', centre: [5, 1.5], size: [4, 34], capacity: 2 },
      ...aisleXs(RACK_ROWS_WEST).concat(aisleXs(RACK_ROWS_EAST)).flatMap(x => [
        { id: `aisle_n_${x < 0 ? 'w' : 'e'}${Math.abs(x)}`, centre: [x, 36], size: [2.26, 18.4], capacity: 1 },
        { id: `aisle_s_${x < 0 ? 'w' : 'e'}${Math.abs(x)}`, centre: [x, 1.5], size: [2.26, 33.4], capacity: 1 },
      ]),
    ],
    // Coarse footprints that Nav2 must never enter (keepout mask); see the scene "solids".
    keepout_tags: ['Estante', 'Palete no chão', 'Esteira de triagem', 'Célula de produção', 'Mesa de inspeção',
      'Estação de recarga', 'Pilar do mezanino', 'Escada do mezanino', 'Grade de proteção', 'Extintor', 'Pilar'],
  },
};
