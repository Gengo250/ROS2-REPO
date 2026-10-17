import { Material, Mesh, Object3D, Texture, InstancedMesh, BufferGeometry } from 'three';

export function disposeTree(root: Object3D) {
  const geometries = new Set<BufferGeometry>();
  const materials = new Set<Material>();
  const textures = new Set<Texture>();
  root.traverse(object => {
    if (!(object instanceof Mesh)) return;
    geometries.add(object.geometry);
    (Array.isArray(object.material) ? object.material : [object.material]).forEach(m => materials.add(m));
    if (object instanceof InstancedMesh) object.dispose();
  });
  materials.forEach(material => {
    Object.values(material).forEach(value => { if (value instanceof Texture) textures.add(value); });
    material.dispose();
  });
  textures.forEach(texture => {
    if (typeof ImageBitmap !== 'undefined' && texture.image instanceof ImageBitmap) texture.image.close();
    texture.dispose();
  });
  geometries.forEach(geometry => geometry.dispose());
  root.clear();
}
