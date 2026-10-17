import { describe, it, expect, vi } from 'vitest';
import { Group, Mesh, BoxGeometry, MeshStandardMaterial, Vector3, Quaternion } from 'three';
import { poseMatrix, worldMatrix, gltfToWorld, rosQuaternion } from '../src/rendering/coordinates/CoordinateSystem';
import { disposeTree } from '../src/rendering/core/resources';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { readFileSync } from 'node:fs';

describe('world coordinates and original AMR', () => {
  it('keeps right handed metric poses and composes fixed-axis rotations', () => {
    expect(new Vector3(1,0,0).applyMatrix4(poseMatrix([0,0,0,0,0,Math.PI/2])).distanceTo(new Vector3(0,1,0))).toBeLessThan(1e-10);
    expect(new Vector3().applyMatrix4(worldMatrix([[1,2,3,0,0,Math.PI/2],[1,0,0,0,0,0]])).distanceTo(new Vector3(1,3,3))).toBeLessThan(1e-10);
    const q = rosQuaternion({x:0,y:0,z:Math.sin(.3),w:Math.cos(.3)});
    expect(q.angleTo(new Quaternion().setFromAxisAngle(new Vector3(0,0,1),.6))).toBeLessThan(1e-7);
    expect(new Vector3(0,1,0).applyMatrix4(gltfToWorld()).distanceTo(new Vector3(0,0,1))).toBeLessThan(1e-10);
  });
  it('loads the actual GLB and releases shared resources once', async () => {
    const raw = readFileSync('public/models/warehouse_amr.glb');
    const gltf = await new GLTFLoader().parseAsync(raw.buffer.slice(raw.byteOffset,raw.byteOffset+raw.byteLength), '');
    expect(gltf.scene.children.length).toBeGreaterThan(5);
    disposeTree(gltf.scene);
    const geometry = new BoxGeometry(), material = new MeshStandardMaterial();
    const g = vi.spyOn(geometry,'dispose'), m = vi.spyOn(material,'dispose');
    const group = new Group(); group.add(new Mesh(geometry,material),new Mesh(geometry,material));
    disposeTree(group); expect(g).toHaveBeenCalledTimes(1); expect(m).toHaveBeenCalledTimes(1);
    expect(group.children).toHaveLength(0);
  });
});
