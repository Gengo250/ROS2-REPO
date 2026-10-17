#!/usr/bin/env python3
"""Export the original generated scene to glTF Y-up, retaining metres/materials."""
import argparse
import json
from pathlib import Path
import sys
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from amr_parameters import load_parameters


def export_glb(scene, output):
    params = load_parameters()
    if scene is None or scene.get('config_sha256') != params['config_sha256']:
        raise RuntimeError('Regenerate the Blender scene from current AMR configuration first')
    bpy.context.window.scene = scene
    bpy.ops.object.select_all(action='DESELECT')
    objects = [obj for obj in scene.objects if obj.type == 'MESH' and not obj.hide_render
               and not any(c.name.endswith('DEBUG') for c in obj.users_collection)]
    if not objects:
        raise RuntimeError('No visible robot geometry')
    depsgraph = bpy.context.evaluated_depsgraph_get()
    points = [obj.matrix_world @ Vector(corner) for obj in objects
              for corner in obj.evaluated_get(depsgraph).bound_box]
    bounds = [[min(p[i] for p in points) for i in range(3)],
              [max(p[i] for p in points) for i in range(3)]]
    for obj in objects: obj.select_set(True)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(filepath=str(output), export_format='GLB', use_selection=True,
        export_yup=True, export_apply=True, export_materials='EXPORT', export_cameras=False,
        export_lights=False, export_animations=False, export_extras=False)
    metadata = dict(schema_version='1.0', config_sha256=params['config_sha256'],
        units='metres', source_axes='X forward Y left Z up', asset_axes='X forward Y up Z right',
        world_conversion='Rotate +pi/2 about X once at the asset root',
        bounds_world=bounds, mesh_count=len(objects), blender_version=bpy.app.version_string)
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT.parents[1]/'web/digital-twin/public/models/warehouse_amr.glb')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
    export_glb(bpy.data.scenes.get('AMR_Generation'), args.output)
