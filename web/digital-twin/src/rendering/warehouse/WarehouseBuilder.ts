import { BoxGeometry, CanvasTexture, Color, CylinderGeometry, Group, InstancedMesh,
  Mesh, MeshStandardMaterial, PlaneGeometry, SphereGeometry, SRGBColorSpace, Vector3 } from 'three';
import type { Primitive, WarehouseScene } from '../../core/types/warehouse-scene';
import { worldMatrix } from '../coordinates/CoordinateSystem';

export function buildWarehouse(data: WarehouseScene): Group {
  const group = new Group();
  const batches = new Map<string, Primitive[]>();
  for (const primitive of data.primitives) {
    const key = JSON.stringify([primitive.kind, primitive.colour, primitive.category, primitive.label]);
    const batch = batches.get(key) ?? [];
    batch.push(primitive); batches.set(key, batch);
  }
  for (const batch of batches.values()) {
    const p = batch[0]!;
    const geometry = p.kind === 'box' ? new BoxGeometry(1, 1, 1)
      : p.kind === 'sphere' ? new SphereGeometry(1, 16, 12)
      : p.kind === 'cylinder' ? new CylinderGeometry(1, 1, 1, 20).rotateX(Math.PI / 2)
      : new PlaneGeometry(1, 1);
    const material = new MeshStandardMaterial({ color: new Color().setRGB(...p.colour.slice(0, 3) as [number, number, number]),
      roughness: .78, metalness: p.category === 'equipment' ? .15 : .04 });
    if (p.kind === 'label') {
      const canvas = document.createElement('canvas'); canvas.width = 512; canvas.height = 128;
      const ctx = canvas.getContext('2d')!;
      ctx.fillStyle = '#16242f'; ctx.fillRect(0, 0, 512, 128);
      ctx.fillStyle = '#eff1eb'; ctx.font = 'bold 46px sans-serif'; ctx.textAlign = 'center';
      ctx.textBaseline = 'middle'; ctx.fillText(p.label!, 256, 64);
      material.map = new CanvasTexture(canvas); material.map.colorSpace = SRGBColorSpace;
    }
    const mesh = new InstancedMesh(geometry, material, batch.length);
    mesh.name = p.category; mesh.userData.ids = batch.map(item => item.id);
    batch.forEach((item, index) => {
      const [a, b, c] = item.dimensions;
      const scale = item.kind === 'sphere' ? [a, a, a]
        : item.kind === 'cylinder' ? [a, a, b]
        : item.kind === 'label' ? [a, b, 1] : [a, b, c];
      const matrix = worldMatrix(item.transforms);
      matrix.scale(new Vector3(scale[0], scale[1], scale[2]));
      mesh.setMatrixAt(index, matrix);
    });
    mesh.instanceMatrix.needsUpdate = true;
    mesh.castShadow = !['floor', 'marking', 'wall'].includes(p.category);
    mesh.receiveShadow = true;
    group.add(mesh);
  }
  group.userData.primitiveCount = data.primitives.length;
  return group;
}

export function setStructureVisible(group: Group, visible: boolean) {
  group.children.forEach(mesh => {
    if (mesh instanceof Mesh && ['wall', 'beam'].includes(mesh.name)) mesh.visible = visible;
  });
}
