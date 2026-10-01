#!/usr/bin/env python3
"""Rebuild the owned AMR scene. Run with Blender or import through a local MCP."""
import argparse
import math
from pathlib import Path
import sys

import bpy
import bmesh
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from amr_parameters import load_parameters

SCENE_NAME = 'AMR_Generation'
COLLECTIONS = ('CHASSIS', 'PLATFORM', 'DRIVE', 'CASTERS', 'SENSORS', 'DEBUG')


def clear_scene():
    """Only remove data belonging exclusively to our generated scene."""
    scene = bpy.data.scenes.get(SCENE_NAME)
    if scene and not scene.get('amr_generated'):
        raise RuntimeError(f'Refusing to overwrite unowned scene {SCENE_NAME}')
    if scene:
        for obj in list(scene.objects):
            if len(obj.users_scene) > 1:
                raise RuntimeError(f'{obj.name} is shared with another scene')
        for obj in list(scene.objects):
            data = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if isinstance(data, bpy.types.Mesh) and data.users == 0:
                bpy.data.meshes.remove(data)
        for collection in list(scene.collection.children):
            for child in list(collection.children):
                bpy.data.collections.remove(child)
            bpy.data.collections.remove(collection)
    else:
        scene = bpy.data.scenes.new(SCENE_NAME)
    scene['amr_generated'] = True
    bpy.context.window.scene = scene
    root = bpy.data.collections.new('AMR')
    scene.collection.children.link(root)
    groups = {}
    for name in COLLECTIONS:
        child = bpy.data.collections.new(name)
        root.children.link(child)
        groups[name] = child
    return scene, groups


def configure_units(scene):
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.length_unit = 'METERS'
    scene.unit_settings.scale_length = 1.0
    scene['axes'] = '+X forward; +Y left; +Z up; metres'


def create_materials(p):
    result = {}
    for role in ('chassis', 'platform', 'rubber', 'sensor', 'accent'):
        name = 'AMR_' + role
        material = bpy.data.materials.get(name) or bpy.data.materials.new(name)
        material.diffuse_color = p[role + '_rgba']
        if not material.use_nodes:
            material.use_nodes = True
        shader = material.node_tree.nodes.get('Principled BSDF')
        shader.inputs['Base Color'].default_value = p[role + '_rgba']
        shader.inputs['Metallic'].default_value = 0.55 if role == 'platform' else 0.15
        shader.inputs['Roughness'].default_value = 0.6 if role == 'rubber' else 0.36
        result[role] = material
    return result


def mesh_object(name, vertices, faces, location, group, material):
    mesh = bpy.data.meshes.new(name + '_Mesh')
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    group.objects.link(obj)
    obj.location = location
    obj.data.materials.append(material)
    obj['amr_part'] = True
    return obj


def box(name, size, location, group, material):
    x, y, z = (v/2 for v in size)
    vertices = [(a*x, b*y, c*z) for a, b, c in
                [(-1,-1,-1), (1,-1,-1), (1,1,-1), (-1,1,-1),
                 (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1)]]
    return mesh_object(name, vertices,
                       [(0,3,2,1), (4,5,6,7), (0,1,5,4), (1,2,6,5), (2,3,7,6), (3,0,4,7)],
                       location, group, material)


def rounded_prism(name, length, width, height, radius, taper, location, group, material, p):
    """Round rectangular XY footprint, with a continuous narrowing at both ends."""
    outline = []
    for cx, cy, start in [(length/2-radius, width/2-radius, 0),
                          (-length/2+radius, width/2-radius, 90),
                          (-length/2+radius, -width/2+radius, 180),
                          (length/2-radius, -width/2+radius, 270)]:
        for step in range(p['corner_segments']+1):
            angle = math.radians(start + step * 90/p['corner_segments'])
            x, y = cx+radius*math.cos(angle), cy+radius*math.sin(angle)
            outline.append((x, y))
    # Include the centre of straight sides, retaining the nominal maximum width.
    subdivided = []
    for i, point in enumerate(outline):
        nxt = outline[(i+1) % len(outline)]
        subdivided.extend([point, ((point[0]+nxt[0])/2, (point[1]+nxt[1])/2)])
    outline = [(x,y*(1-(2*taper/width)*(abs(x)/(length/2))**4)) for x,y in subdivided]
    n = len(outline)
    vertices = [(x, y, z) for z in (-height/2, height/2) for x, y in outline]
    faces = [tuple(reversed(range(n))), tuple(range(n, 2*n))]
    faces += [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)]
    return mesh_object(name, vertices, faces, location, group, material)


def bevel(obj, amount, p):
    mod = obj.modifiers.new('Manufactured edges', 'BEVEL')
    mod.width = amount
    mod.segments = p['bevel_segments']
    mod.limit_method = 'ANGLE'
    mod.harden_normals = True


def subtract(obj, cutter):
    mod = obj.modifiers.new('Functional clearance', 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.solver = 'EXACT'
    mod.object = cutter
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)
    mesh = cutter.data
    bpy.data.objects.remove(cutter, do_unlink=True)
    bpy.data.meshes.remove(mesh)


def cylinder(name, radius, width, axis, location, group, material, p):
    vertices = []
    n = p['mesh_segments']
    for axial in (-width/2, width/2):
        for i in range(n):
            a = 2*math.pi*i/n
            ring = (radius*math.cos(a), radius*math.sin(a))
            vertices.append((ring[0], axial, ring[1]) if axis == 'Y' else (*ring, axial))
    faces = [tuple(range(n-1,-1,-1)), tuple(range(n,2*n))]
    faces += [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)]
    obj = mesh_object(name, vertices, faces, location, group, material)
    for polygon in obj.data.polygons[2:]:
        polygon.use_smooth = True
    return obj


def create_chassis(p, g, m):
    obj = rounded_prism('AMR_Chassis', p['chassis_length'], p['chassis_width'],
                        p['chassis_height'], p['chassis_corner_radius'], p['chassis_end_taper'],
                        (0,0,p['chassis_z']), g['CHASSIS'], m['chassis'], p)
    # Side pockets are open underneath and outboard. Their roof hides the wheel tops.
    for sign in (-1, 1):
        depth = p['chassis_width']/2-p['core_half_width'] + p['wheel_arch_gap']
        pocket = box('AMR_Cutter', (2*p['arch_half_length'], depth,
                     p['arch_top'] + p['wheel_arch_gap']),
                     (0, sign*(p['core_half_width']+depth/2),
                      (p['arch_top']-p['wheel_arch_gap'])/2), g['DEBUG'], m['chassis'])
        subtract(obj, pocket)
    bay = box('AMR_Cutter', (2*p['sensor_bay_depth'], 2*p['chassis_width'], p['sensor_bay_height']),
              (p['chassis_length']/2, 0, p['bay_z']), g['DEBUG'], m['chassis'])
    subtract(obj, bay)
    # Flush-mounted panels get real recesses, avoiding coplanar surfaces in RViz/Gazebo.
    for sign in (-1,1):
        pocket = box('AMR_Cutter', tuple(p[key]+2*p['panel_gap'] for key in
                     ('bumper_length','bumper_width','bumper_height')),
                     (sign*p['bumper_x'],0,p['bumper_z']),g['DEBUG'],m['chassis'])
        subtract(obj,pocket)
    pocket = box('AMR_Cutter',tuple(p[key]+2*p['panel_gap'] for key in
                 ('camera_length','camera_width','camera_height')),
                 (p['camera_x'],0,p['camera_z']),g['DEBUG'],m['chassis'])
    subtract(obj,pocket)
    bevel(obj, p['chassis_bevel'], p)
    obj['export_stem'] = 'chassis'
    obj['ros_link'] = 'chassis_link'
    return obj


def create_platform(p, g, m):
    obj = rounded_prism('AMR_Platform', p['platform_length'], p['platform_width'],
                        p['platform_thickness'], p['platform_corner_radius'], 0,
                        (0,0,p['platform_z']), g['PLATFORM'], m['platform'], p)
    bevel(obj, p['chassis_bevel']/2, p)
    obj['export_stem'] = 'platform'
    obj['ros_link'] = 'platform_link'


def create_bumpers(p, g, m):
    for label, sign in (('Front', 1), ('Rear', -1)):
        obj = box(f'AMR_Bumper_{label}', (p['bumper_length'],p['bumper_width'],p['bumper_height']),
                  (sign*p['bumper_x'],0,p['bumper_z']), g['CHASSIS'], m['accent'])
        bevel(obj, p['chassis_bevel'], p)
        obj['export_stem'] = 'bumper_' + label.lower()
        obj['ros_link'] = 'bumper_' + label.lower() + '_link'


def create_drive_wheel(p, g, m):
    obj = cylinder('AMR_Drive_Wheel',p['wheel_radius'],p['wheel_width'],'Y',
                   (0,p['wheel_y'],p['wheel_z']),g['DRIVE'],m['rubber'],p)
    bevel(obj,p['wheel_bevel'],p)
    obj['export_stem'] = 'wheel'
    obj['ros_link'] = 'left_wheel_link'
    right = obj.copy()
    right.name = 'AMR_Drive_Wheel_Right'
    g['DRIVE'].objects.link(right)
    right.location.y = -p['wheel_y']
    del right['export_stem']
    right['ros_link'] = 'right_wheel_link'


def create_casters(p, g, m):
    # Ball-transfer supports: visually and physically a simple sphere, no fake swivel mechanism.
    # Explicit indexed rings avoid bmesh's allocation-dependent sphere face ordering.
    segments, stacks, radius = p['mesh_segments']//2, p['mesh_segments']//4, p['caster_radius']
    vertices = [(0,0,radius),(0,0,-radius)]
    for j in range(1,stacks):
        theta = math.pi*j/stacks
        for i in range(segments):
            angle = 2*math.pi*i/segments
            vertices.append((radius*math.sin(theta)*math.cos(angle),
                             radius*math.sin(theta)*math.sin(angle),radius*math.cos(theta)))
    faces = [(0,2+i,2+(i+1)%segments) for i in range(segments)]
    for j in range(stacks-2):
        start = 2+j*segments
        faces.extend((start+i,start+(i+1)%segments,start+segments+(i+1)%segments,start+segments+i)
                     for i in range(segments))
    start = 2+(stacks-2)*segments
    faces.extend((1,start+(i+1)%segments,start+i) for i in range(segments))
    obj = mesh_object('AMR_Caster',vertices,faces,(p['caster_x'],0,radius),g['CASTERS'],m['rubber'])
    obj['ros_link'] = 'front_caster_link'
    obj['export_stem'] = 'caster'
    rear = obj.copy()
    rear.name = 'AMR_Caster_Rear'
    rear.location.x = -p['caster_x']
    rear['ros_link'] = 'rear_caster_link'
    del rear['export_stem']
    g['CASTERS'].objects.link(rear)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True


def create_lidar_housing(p, g, m):
    obj = cylinder('AMR_Lidar_Housing',p['lidar_radius'],p['lidar_height'],'Z',
                   (p['lidar_x'],0,p['lidar_z']),g['SENSORS'],m['sensor'],p)
    bevel(obj,p['wheel_bevel']/2,p)
    obj['export_stem'] = 'lidar_housing'
    obj['ros_link'] = 'lidar_link'


def create_camera_housing(p, g, m):
    obj = box('AMR_Camera_Housing',(p['camera_length'],p['camera_width'],p['camera_height']),
              (p['camera_x'],0,p['camera_z']),g['SENSORS'],m['sensor'])
    bevel(obj,p['wheel_bevel'],p)
    obj['export_stem'] = 'camera_housing'
    obj['ros_link'] = 'camera_link'
    # Lens disks are merged into the same housing mesh, thus no extra TF or export is needed.
    for sign in (-1,1):
        radius = p['camera_height']/4
        lens = cylinder('AMR_Lens', radius, p['camera_lens_gap'], 'Z', (0,0,0),
                        g['SENSORS'],m['platform'],p)
        # Bake local Z-to-X rotation and the housing-relative lens location into vertices.
        from mathutils import Matrix
        lens.data.transform(Matrix.Rotation(math.pi/2,4,'Y'))
        lens.location = (p['chassis_length']/2, sign*p['camera_width']/4, p['camera_z'])
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        lens.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.join()


def create_debug(p, g, m):
    obj = box('AMR_IMU_Debug', (p['imu_size'],)*3, (0,0,p['chassis_com_height']),g['DEBUG'],m['accent'])
    obj.hide_render = True
    obj.hide_set(True)
    obj['ros_link'] = 'imu_link'


def create_inspection_renders(scene, p, root):
    """Temporary studio objects exist only during rendering and never enter exports."""
    group = bpy.data.collections.new('AMR_Render_Only')
    scene.collection.children.link(group)
    world = bpy.data.worlds.get('AMR_Studio') or bpy.data.worlds.new('AMR_Studio')
    world.use_nodes = True
    world.node_tree.nodes['Background'].inputs[0].default_value = (0.15,0.18,0.22,1)
    world.node_tree.nodes['Background'].inputs[1].default_value = 0.5
    scene.world = world
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = p['render_samples']
    scene.cycles.use_denoising = True
    scene.render.resolution_x = p['render_resolution']
    scene.render.resolution_y = p['render_resolution']
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.film_transparent = False
    target = Vector((0,0,p['overall_height']/2))
    extent = max(p['chassis_length'],p['chassis_width'])
    floor_mat = bpy.data.materials.get('AMR_Studio_Floor') or bpy.data.materials.new('AMR_Studio_Floor')
    floor_mat.diffuse_color = (0.20,0.23,0.27,1)
    if not floor_mat.use_nodes:
        floor_mat.use_nodes = True
    floor_shader = floor_mat.node_tree.nodes.get('Principled BSDF')
    floor_shader.inputs['Base Color'].default_value = floor_mat.diffuse_color
    floor_shader.inputs['Roughness'].default_value = 0.8
    box('AMR_Render_Floor',(extent*200,extent*200,0.01),(0,0,-0.006),group,floor_mat)
    for label, pos, energy, size in [('Key',(2,-2,3),450,2.5),('Fill',(0,2,2),250,2),('Rim',(-2,-1,2.5),350,1.5)]:
        light = bpy.data.lights.new('AMR_'+label,'AREA')
        light.energy = energy
        light.shape = 'DISK'
        light.size = size*extent
        obj = bpy.data.objects.new(light.name,light)
        group.objects.link(obj)
        obj.location = Vector(pos)*extent
        obj.rotation_euler = (target-obj.location).to_track_quat('-Z','Y').to_euler()
    camera = bpy.data.cameras.new('AMR_Inspection_Camera')
    camera.type = 'ORTHO'
    camera.ortho_scale = extent*1.45
    obj = bpy.data.objects.new(camera.name,camera)
    group.objects.link(obj)
    scene.camera = obj
    views = {'front':(3,0,0.45),'rear':(-3,0,0.45),'side':(0,-3,0.45),
             'top':(0,0,3),'perspective':(2.4,-2.3,1.8)}
    output = Path(root)/'renders'
    output.mkdir(parents=True,exist_ok=True)
    try:
        for name, relative in views.items():
            # Keep the full orthographic ray origin plane above the studio floor.
            obj.location = target+Vector(relative)*extent*2
            obj.rotation_euler = (target-obj.location).to_track_quat('-Z','Y').to_euler()
            scene.render.filepath = str(output/(name+'.png'))
            bpy.ops.render.render(write_still=True)
    finally:
        scene.camera = None
        for item in list(group.objects):
            data = item.data
            bpy.data.objects.remove(item,do_unlink=True)
            if data.users == 0:
                if isinstance(data,bpy.types.Mesh):
                    bpy.data.meshes.remove(data)
                elif isinstance(data,bpy.types.Camera):
                    bpy.data.cameras.remove(data)
                elif isinstance(data,bpy.types.Light):
                    bpy.data.lights.remove(data)
        bpy.data.collections.remove(group)


def save_blend(scene, root):
    output = Path(root)/'blender/warehouse_amr.blend'
    output.parent.mkdir(parents=True,exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    # The generated scene is the active scene when the result is opened.
    bpy.context.window.scene = scene
    bpy.ops.wm.save_as_mainfile(filepath=str(output))


def main(root=ROOT, renders=True):
    root = Path(root)
    p = load_parameters(root)
    scene, groups = clear_scene()
    configure_units(scene)
    scene['config_sha256'] = p['config_sha256']
    materials = create_materials(p)
    create_chassis(p,groups,materials)
    create_platform(p,groups,materials)
    create_bumpers(p,groups,materials)
    create_drive_wheel(p,groups,materials)
    create_casters(p,groups,materials)
    create_lidar_housing(p,groups,materials)
    create_camera_housing(p,groups,materials)
    create_debug(p,groups,materials)
    bpy.context.view_layer.update()
    from validate_amr import validate_scene, validate_exports
    from export_amr import export_meshes
    validate_scene(scene,p)
    export_meshes(scene,root)
    validate_exports(root,p)
    save_blend(scene,root)
    if renders:
        create_inspection_renders(scene,p,root)
        save_blend(scene,root)
    print('AMR BUILD OK:',bpy.app.version_string,sys.version,root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-renders',action='store_true')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
    main(renders=not args.no_renders)
