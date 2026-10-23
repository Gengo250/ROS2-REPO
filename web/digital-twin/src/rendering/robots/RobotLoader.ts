import { Box3, Group, Vector3 } from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { gltfToWorld, poseMatrix } from '../coordinates/CoordinateSystem';
import { disposeTree } from '../core/resources';

export async function loadRobot(pose: number[], signal: AbortSignal, expectedConfigHash: string): Promise<Group> {
  const [asset, metadata] = await Promise.all([
    fetch('/models/warehouse_amr.glb', { signal }).then(r => { if (!r.ok) throw Error('AMR GLB unavailable'); return r.arrayBuffer(); }),
    fetch('/models/warehouse_amr.json', { signal }).then(r => { if (!r.ok) throw Error('AMR scale metadata unavailable'); return r.json(); }),
  ]);
  if (metadata.config_sha256 !== expectedConfigHash) throw Error('AMR asset configuration is stale; regenerate GLB');
  const gltf = await new GLTFLoader().parseAsync(asset, '/models/');
  const model = gltf.scene;
  model.applyMatrix4(gltfToWorld());
  model.updateMatrixWorld(true);
  const bounds = new Box3().setFromObject(model);
  const expected = metadata.bounds_world as number[][];
  const actual = [bounds.min.toArray(), bounds.max.toArray()];
  if (signal.aborted || !expected || actual.some((v, i) => v.some((x, j) => Math.abs(x - expected[i]![j]!) > .002))) {
    disposeTree(model);
    throw Error(signal.aborted ? 'AMR loading cancelled' : 'AMR metric scale/bounds mismatch');
  }
  const robot = new Group(); robot.name = 'amr_01'; robot.add(model);
  robot.applyMatrix4(poseMatrix(pose));
  robot.userData.size = bounds.getSize(new Vector3()).toArray();
  model.traverse(obj => { obj.castShadow = false; obj.receiveShadow = true; });
  return robot;
}
