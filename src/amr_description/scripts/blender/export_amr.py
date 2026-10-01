#!/usr/bin/env python3
"""Export evaluated copies in ROS link-local coordinates; leave the scene editable."""
from pathlib import Path
import sys
import bpy
from mathutils import Matrix

ROOT = Path(__file__).resolve().parents[2]


def export_meshes(scene, root=ROOT):
    output = Path(root)/'meshes/visual'
    output.mkdir(parents=True,exist_ok=True)
    bpy.context.window.scene = scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for obj in list(scene.objects):
        stem = obj.get('export_stem')
        if not stem:
            continue
        if any(abs(v-1)>1e-6 for v in obj.scale) or obj.rotation_euler.to_matrix() != Matrix.Identity(3):
            raise ValueError(f'{obj.name}: unexpected transform; correct the generator')
        # new_from_object bakes modifiers. Translation is deliberately NOT baked:
        # mesh coordinates are relative to the matching link's origin.
        mesh = bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph),depsgraph=depsgraph)
        temp = bpy.data.objects.new('AMR_Export_Temporary',mesh)
        scene.collection.objects.link(temp)
        bpy.ops.object.select_all(action='DESELECT')
        temp.select_set(True)
        bpy.context.view_layer.objects.active = temp
        path = str(output/(stem+'.stl'))
        try:
            if bpy.app.version >= (4,0,0):
                bpy.ops.wm.stl_export(filepath=path,export_selected_objects=True,
                                      apply_modifiers=True,global_scale=1.0,
                                      use_scene_unit=False,forward_axis='Y',up_axis='Z',ascii_format=False)
            else:
                # Blender <=3.6 bundled io_mesh_stl; enabling it requires no download.
                bpy.ops.preferences.addon_enable(module='io_mesh_stl')
                bpy.ops.export_mesh.stl(filepath=path,use_selection=True,
                                        use_mesh_modifiers=True,global_scale=1.0,
                                        use_scene_unit=False,axis_forward='Y',axis_up='Z',ascii=False)
            # Blender's binary STL header can contain the current .blend filename.
            # Normalize metadata so clean and already-open rebuilds are byte reproducible.
            with open(path,'r+b') as stream:
                stream.write(('warehouse_amr '+stem+' | metres | X forward Y left Z up').encode('ascii').ljust(80,b' '))
            print('EXPORTED',stem,'local origin',tuple(obj.location),'metres; identity axes')
        finally:
            bpy.data.objects.remove(temp,do_unlink=True)
            bpy.data.meshes.remove(mesh)


if __name__ == '__main__':
    scene = bpy.data.scenes.get('AMR_Generation')
    if scene is None:
        raise RuntimeError('Open blender/warehouse_amr.blend first')
    sys.path.insert(0,str(ROOT/'scripts'))
    from amr_parameters import load_parameters
    if scene.get('config_sha256') != load_parameters()['config_sha256']:
        raise RuntimeError('Configuration changed: regenerate before exporting')
    export_meshes(scene)
