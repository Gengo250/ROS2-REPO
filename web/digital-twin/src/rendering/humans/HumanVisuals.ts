import { Group, InstancedMesh, Matrix4, Object3D, Vector3 } from 'three';
import type { WarehouseScene } from '../../core/types/warehouse-scene';
import { buildWarehouse } from '../warehouse/WarehouseBuilder';
import { poseMatrix } from '../coordinates/CoordinateSystem';
import type { StateSynchronizer } from '../synchronization/StateSynchronizer';

/** Original SDF geometry, batched as before; actual Gazebo visual transforms. */
export function buildHumans(data: WarehouseScene, synchronizer: StateSynchronizer) {
  const primitives = data.primitives.filter(p => p.category === 'human');
  const group = buildWarehouse({ ...data, primitives });
  group.name = 'dynamic-humans';
  const objects = new Map<string, Object3D>();
  for (const primitive of primitives) {
    const names = primitive.id.split('/');
    if (names.length !== primitive.transforms.length) throw Error('Unsupported human SDF hierarchy');
    let parent: Object3D = group;
    for (let i = 0; i < names.length; i++) {
      const id = names.slice(0, i+1).join('/');
      let object = objects.get(id);
      if (!object) {
        object = new Group(); object.name = id;
        object.applyMatrix4(poseMatrix(primitive.transforms[i]!));
        parent.add(object); objects.set(id, object); synchronizer.register(id, object);
      }
      parent = object;
    }
  }
  const byId = new Map(primitives.map(p => [p.id, p]));
  const slots: { mesh: InstancedMesh; index: number; object: Object3D; scale: Vector3 }[] = [];
  const meshes = group.children.filter((o): o is InstancedMesh => o instanceof InstancedMesh);
  for (const mesh of meshes) {
    // Static shadow map excludes moving humans, avoiding baked ghost silhouettes.
    // Geometry and lighting still update every frame; no second animation clock.
    mesh.castShadow = false;
    mesh.frustumCulled = false; // Instance bounds change along the physical routes.
    (mesh.userData.ids as string[]).forEach((id, index) => {
      const p = byId.get(id)!; const [a, b, c] = p.dimensions;
      const scale = p.kind === 'sphere' ? new Vector3(a,a,a)
        : p.kind === 'cylinder' ? new Vector3(a,a,b) : new Vector3(a,b,c);
      slots.push({mesh,index,object:objects.get(id)!,scale});
    });
  }
  const matrix = new Matrix4();
  const update = () => {
    group.updateMatrixWorld(true);
    for (const slot of slots) slot.mesh.setMatrixAt(slot.index, matrix.copy(slot.object.matrixWorld).scale(slot.scale));
    for (const mesh of meshes) mesh.instanceMatrix.needsUpdate = true;
  };
  update();
  return { group, update, visualCount: slots.length, humanCount: data.entities.filter(e => e.kind === 'human').length };
}
