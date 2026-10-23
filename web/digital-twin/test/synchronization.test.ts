import { it, expect } from 'vitest';
import { Group } from 'three';
import { StateSynchronizer } from '../src/rendering/synchronization/StateSynchronizer';
import type { TwinStateV2, DynamicEntity } from '../src/core/types/twin-state-v2';

function entity(time: number, x: number, angle = 0): DynamicEntity {
  return { id: 'amr_01', gazebo_name: 'warehouse_amr', type: 'robot', frame_id: 'world',
    pose: { position: { x, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(angle/2), w: Math.cos(angle/2) } },
    velocity: null, source_timestamp: time, received_at: '2026-10-22T00:00:00Z', age_seconds: 0,
    status: 'online', pose_source: '/digital_twin/simulation/poses', components: [] };
}
function state(sequence: number, e: DynamicEntity, epoch = 0) {
  return { session_id: 'session', sequence, simulation: { epoch }, world: { frame_id: 'world' },
    simulation_robots: [e], humans: [], obstacles: [] } as unknown as TwinStateV2;
}
it('interpolates position and shortest quaternion arc then clamps at newest real pose', () => {
  const sync = new StateSynchronizer(), robot = new Group(); sync.register('amr_01', robot);
  sync.accept(state(1, entity(1, 0)), 1000);
  sync.accept(state(2, entity(1.1, 1, Math.PI/2)), 1100);
  sync.update(1170);
  expect(robot.position.x).toBeCloseTo(.5);
  expect(robot.quaternion.z).toBeCloseTo(Math.sin(Math.PI/8));
  sync.update(10000); expect(robot.position.x).toBe(1);
  expect(sync.buffers.get('amr_01')).toHaveLength(2);
});
it('freezes on disconnect, rejects old sequence and resets across epochs/frames', () => {
  const sync = new StateSynchronizer(), robot = new Group(); sync.register('amr_01', robot);
  sync.accept(state(2, entity(3, 2)), 1000); sync.update(1000);
  sync.accept(state(1, entity(2, 10)), 1100); sync.update(2000); expect(robot.position.x).toBe(2);
  sync.disconnect(); sync.update(4000); expect(robot.position.x).toBe(2);
  sync.accept(state(3, entity(.1, -11), 1), 5000); sync.update(5000); expect(robot.position.x).toBe(-11);
  sync.accept(state(4, {...entity(.2, 88), frame_id: 'odom'}, 1), 5100);
  sync.update(5200); expect(robot.position.x).toBe(-11);
  sync.clear(); expect(sync.objects.size).toBe(0);
});
it('bounds history and independently stops stale entities', () => {
  const sync = new StateSynchronizer(); sync.register('amr_01', new Group());
  for (let i=1;i<100;i++) sync.accept(state(i, entity(i, i)), i*100);
  expect(sync.buffers.get('amr_01')).toHaveLength(20);
  sync.accept(state(100, {...entity(100, 100), status:'stale'}),10000);
  expect(sync.buffers.size).toBe(0);
});

it('does not rewind on arrival jitter and freezes entities omitted from a snapshot', () => {
  const sync = new StateSynchronizer(), robot = new Group(); sync.register('amr_01', robot);
  sync.accept(state(1, entity(1, 0)), 1000);
  sync.accept(state(2, entity(1.1, 1)), 1100);
  sync.update(1300); expect(robot.position.x).toBe(1);
  sync.accept(state(3, entity(1.2, 2)), 1310);
  sync.accept(state(4, entity(1.3, 3)), 1320);
  sync.update(1320); expect(robot.position.x).toBeGreaterThanOrEqual(1);
  const missing = state(5, entity(1.4, 4)); missing.simulation_robots = [];
  sync.accept(missing, 1330); sync.update(2000);
  expect(robot.position.x).toBe(1); expect(sync.buffers.size).toBe(0);
  expect(sync.rendered.amr_01!.status).toBe('offline');
  sync.accept(state(6, entity(.1, -11), 1), 2100); sync.update(2100);
  expect(robot.position.x).toBe(-11);
});

it('applies actual limb poses through original SDF hierarchy without duplicate geometry', async () => {
  const { buildHumans } = await import('../src/rendering/humans/HumanVisuals');
  const { Matrix4, Vector3, InstancedMesh } = await import('three');
  const sync = new StateSynchronizer();
  const data = { entities: [{id:'human_01',kind:'human'}], primitives: [
    { id:'human_01/body/leg',kind:'box',dimensions:[.1,.2,.5],colour:[.2,.3,.4,1],category:'human',
      transforms:[[3,0,0,0,0,Math.PI/2],[0,0,0,0,0,0],[1,0,0,0,0,0]] }
  ] } as unknown as import('../src/core/types/warehouse-scene').WarehouseScene;
  const humans = buildHumans(data,sync);
  expect(humans.humanCount).toBe(1); expect(humans.visualCount).toBe(1);
  const mesh=humans.group.children.find(o=>o instanceof InstancedMesh) as InstanceType<typeof InstancedMesh>;
  const matrix=new Matrix4(); mesh.getMatrixAt(0,matrix);
  expect(new Vector3().setFromMatrixPosition(matrix).toArray()).toEqual([3,1,0]);
  sync.objects.get('human_01/body/leg')!.position.x=2;
  humans.update();mesh.getMatrixAt(0,matrix);
  expect(new Vector3().setFromMatrixPosition(matrix).toArray()).toEqual([3,2,0]);
  const {disposeTree}=await import('../src/rendering/core/resources');
  disposeTree(humans.group);expect(humans.group.children).toHaveLength(0);
});
