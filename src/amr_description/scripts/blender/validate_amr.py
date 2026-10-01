#!/usr/bin/env python3
"""Fail loudly on geometric, origin, contact, FOV or binary STL inconsistencies."""
import json
import math
from pathlib import Path
import struct
import sys
import bpy
import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from amr_parameters import load_parameters


def bounds(obj):
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    points = [obj.matrix_world @ Vector(v) for v in evaluated.bound_box]
    return ([min(v[i] for v in points) for i in range(3)],
            [max(v[i] for v in points) for i in range(3)])


def require(condition, message):
    if not condition:
        raise AssertionError(message)
    print('PASS:',message)


def validate_scene(scene, p):
    bpy.context.window.scene = scene
    bpy.context.view_layer.update()
    expected = {
        'AMR_Chassis': (0,0,p['chassis_z']), 'AMR_Platform': (0,0,p['platform_z']),
        'AMR_Drive_Wheel': (0,p['wheel_y'],p['wheel_z']),
        'AMR_Drive_Wheel_Right': (0,-p['wheel_y'],p['wheel_z']),
        'AMR_Caster': (p['caster_x'],0,p['caster_radius']),
        'AMR_Caster_Rear': (-p['caster_x'],0,p['caster_radius']),
        'AMR_Bumper_Front': (p['bumper_x'],0,p['bumper_z']),
        'AMR_Bumper_Rear': (-p['bumper_x'],0,p['bumper_z']),
        'AMR_Lidar_Housing': (p['lidar_x'],0,p['lidar_z']),
        'AMR_Camera_Housing': (p['camera_x'],0,p['camera_z'])}
    require(scene.unit_settings.scale_length == 1 and scene.unit_settings.system == 'METRIC','metric scene, unit scale 1')
    require(scene.get('config_sha256') == p['config_sha256'],'scene matches parameter file')
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for name, location in expected.items():
        obj = scene.objects.get(name)
        require(obj is not None,name+' exists')
        require(obj.type == 'MESH' and len(obj.data.polygons)>0,name+' nonempty mesh')
        require(all(abs(v-1)<1e-6 for v in obj.scale),name+' unit scale')
        require(obj.rotation_euler.to_matrix().is_identity,name+' ROS axes baked in local mesh')
        require((obj.location-Vector(location)).length<1e-6,name+' calculated link origin')
        bm = bmesh.new()
        bm.from_object(obj,depsgraph)
        require(all(e.is_manifold for e in bm.edges),name+' manifold evaluated surface')
        bm.free()
    chassis = scene.objects['AMR_Chassis']
    lo,hi = bounds(chassis)
    require(abs(hi[0]-lo[0]-p['chassis_length'])<0.002,'chassis length matches configuration')
    require(abs(hi[1]-lo[1]-p['chassis_width'])<0.005,'chassis width matches configuration')
    require(abs(lo[2]-p['ground_clearance'])<1e-5,'chassis ground clearance')
    require(bounds(scene.objects['AMR_Platform'])[0][2] > hi[2],'platform sits above chassis')
    for name in ('AMR_Drive_Wheel','AMR_Drive_Wheel_Right','AMR_Caster','AMR_Caster_Rear'):
        lo,hi = bounds(scene.objects[name])
        require(abs(lo[2])<0.0005,name+' touches z=0')
    left,right = (scene.objects[n] for n in ('AMR_Drive_Wheel','AMR_Drive_Wheel_Right'))
    require(left.data == right.data and abs(left.location.y+right.location.y)<1e-8,'drive wheels share symmetric geometry')
    # Sample complete wheel envelopes against the evaluated chassis; no hidden intrusion.
    tree = BVHTree.FromObject(chassis,depsgraph)
    for wheel in (left,right):
        for sign in (-1,1):
            for i in range(p['mesh_segments']):
                a = 2*math.pi*i/p['mesh_segments']
                world = wheel.location+Vector((p['wheel_radius']*math.cos(a),sign*p['wheel_width']/2,p['wheel_radius']*math.sin(a)))
                nearest = tree.find_nearest(chassis.matrix_world.inverted() @ world)
                require_distance = nearest[3]
                if require_distance < p['wheel_arch_gap']*0.8:
                    raise AssertionError('wheel-chassis clearance too small')
    require(True,'wheel envelopes clear chassis pockets')
    origin = Vector((p['lidar_x'],0,p['laser_z']))
    for obj in scene.objects:
        if obj.type != 'MESH' or obj.hide_render:
            continue
        tree = BVHTree.FromObject(obj,depsgraph)
        for i in range(p['lidar_samples']):
            a = -p['lidar_half_fov']+2*p['lidar_half_fov']*i/(p['lidar_samples']-1)
            hit = tree.ray_cast(obj.matrix_world.inverted() @ origin,Vector((math.cos(a),math.sin(a),0)),2)
            if hit[0] is not None:
                raise AssertionError(f'LiDAR beam {i} obstructed by {obj.name}')
    require(True,'LiDAR entire configured FOV clear of robot geometry')
    print('BLENDER GEOMETRY VALIDATED')


def validate_exports(root, p):
    expected = {'chassis':(p['chassis_length'],p['chassis_width'],p['chassis_height']),
                'platform':(p['platform_length'],p['platform_width'],p['platform_thickness']),
                'wheel':(2*p['wheel_radius'],p['wheel_width'],2*p['wheel_radius']),
                'caster':(2*p['caster_radius'],)*3,
                'lidar_housing':(2*p['lidar_radius'],2*p['lidar_radius'],p['lidar_height']),
                'camera_housing':(p['camera_length']+p['camera_lens_gap']/2,p['camera_width'],p['camera_height']),
                'bumper_front':(p['bumper_length'],p['bumper_width'],p['bumper_height']),
                'bumper_rear':(p['bumper_length'],p['bumper_width'],p['bumper_height'])}
    report = {}
    for stem, dimensions in expected.items():
        path = Path(root)/'meshes/visual'/(stem+'.stl')
        raw = path.read_bytes()
        count, = struct.unpack_from('<I',raw,80)
        require(count>0 and len(raw)==84+50*count,stem+' valid nonempty binary STL')
        vertices = [struct.unpack_from('<3f',raw,84+50*i+12+12*j) for i in range(count) for j in range(3)]
        lo = [min(v[a] for v in vertices) for a in range(3)]
        hi = [max(v[a] for v in vertices) for a in range(3)]
        actual = [hi[a]-lo[a] for a in range(3)]
        require(all(abs(actual[a]-dimensions[a])<0.005 for a in range(3)),stem+' STL dimensions/orientation in metres')
        require(all(abs((lo[a]+hi[a])/2)<0.002 for a in range(3)),stem+' STL origin at link centre')
        report[stem] = {'triangles':count,'dimensions':actual,'min':lo,'max':hi}
    target = Path(root)/'validation'
    target.mkdir(exist_ok=True)
    (target/'meshes.json').write_text(json.dumps({'config_sha256':p['config_sha256'],'meshes':report},indent=2)+'\n')
    return report


if __name__ == '__main__':
    parameters = load_parameters()
    validate_scene(bpy.data.scenes['AMR_Generation'],parameters)
    validate_exports(ROOT,parameters)
