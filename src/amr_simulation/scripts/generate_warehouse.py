#!/usr/bin/env python3
"""Deterministic, offline warehouse compiler: JSON -> reusable SDF + scenarios.

All distances are SI. The building dimensions describe the clear inside faces.
Generated files are replaced individually; no directory is ever deleted.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ('normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked')


def fmt(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return ' '.join(fmt(v) for v in value)
    if isinstance(value, (int, float)):
        return f'{value:.12g}'
    return str(value)


def tag(parent, element_name, value=None, **attrs):
    element = ET.SubElement(parent, element_name, {k: str(v) for k, v in attrs.items()})
    if value is not None:
        element.text = fmt(value)
    return element


def save_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    path.write_bytes(b'<?xml version="1.0"?>\n' + ET.tostring(root, encoding='utf-8') + b'\n')


def model(name, static=True, pose=None):
    m = ET.Element('model', name=name)
    tag(m, 'static', static)
    if pose is not None:
        tag(m, 'pose', pose)
    return m


def geometry(parent, kind, dimensions):
    g = tag(tag(parent, 'geometry'), kind)
    if kind == 'box':
        tag(g, 'size', dimensions)
    elif kind == 'cylinder':
        tag(g, 'radius', dimensions[0]); tag(g, 'length', dimensions[1])
    elif kind == 'sphere':
        tag(g, 'radius', dimensions[0])
    elif kind == 'mesh':
        tag(g, 'uri', dimensions)


def shape(link, name, size, xyz, colour, collision=True, kind='box', rpy=(0, 0, 0)):
    v = tag(link, 'visual', name=name+'_visual')
    tag(v, 'pose', [*xyz, *rpy]); geometry(v, kind, size)
    if colour is not None:
        material = tag(v, 'material')
        tag(material, 'ambient', colour); tag(material, 'diffuse', colour)
        tag(material, 'specular', [0.06, 0.06, 0.06, 1])
    if collision:
        c = tag(link, 'collision', name=name+'_collision')
        tag(c, 'pose', [*xyz, *rpy]); geometry(c, kind, size)
        return c
    return v


def include(parent, kind, name, pose):
    inc = tag(parent, 'include')
    tag(inc, 'uri', 'model://wh_'+kind)
    tag(inc, 'name', name); tag(inc, 'pose', pose)
    return inc


def write_model(output, kind, m):
    root = ET.Element('sdf', version='1.9'); root.append(m)
    directory = output/'models'/('wh_'+kind)
    save_xml(directory/'model.sdf', root)
    metadata = ET.Element('model')
    tag(metadata, 'name', 'wh_'+kind); tag(metadata, 'version', '1.0')
    tag(metadata, 'sdf', 'model.sdf', version='1.9')
    tag(metadata, 'description', 'Original procedural warehouse asset; Apache-2.0.')
    save_xml(directory/'model.config', metadata)


def load_config(path=ROOT/'config/warehouse_layout.json', amr_path=None):
    config = json.loads(Path(path).read_text())
    if amr_path is None:
        candidate = ROOT.parent/'amr_description/config/amr_dimensions.json'
        if not candidate.is_file():
            from ament_index_python.packages import get_package_share_directory
            candidate = Path(get_package_share_directory('amr_description'))/'config/amr_dimensions.json'
        amr_path = candidate
    amr = json.loads(Path(amr_path).read_text())
    validate(config, amr)
    return config, amr


def validate(c, amr):
    """Reject unsafe dimensions, duplicate identities and blocked default spawns."""
    def require(condition, message):
        if not condition:
            raise ValueError(message)
    def finite(value):
        if isinstance(value, dict):
            for item in value.values(): finite(item)
        elif isinstance(value, list):
            for item in value: finite(item)
        elif isinstance(value, (int, float)):
            require(math.isfinite(value), 'All numbers must be finite')
    finite(c)
    require(c['schema_version'] == 1, 'Unsupported schema version')
    b, r = c['building'], c['racks']
    for key in ('length', 'width', 'height', 'wall_thickness', 'floor_thickness'):
        require(b[key] > 0, 'Invalid building '+key)
    for dims in (r['size'], c['pallet']['size'], c['box']['size']):
        require(len(dims) == 3 and min(dims) > 0, 'Dimensions must be positive XYZ')
    require(c['simulation']['max_step_size'] > 0, 'Invalid physics timestep')
    require(len(r['row_x']) > 1 and len(r['bay_y']) > 1, 'Need rack rows and bays')
    require(len(set(r['row_x'])) == len(r['row_x']), 'Duplicate rack row')
    require(len(set(r['bay_y'])) == len(r['bay_y']), 'Duplicate rack bay')
    require(r['post'] > 0 and 2*r['post'] < min(r['size'][:2]), 'Invalid rack post')
    require(all(0 < z < r['size'][2] for z in r['shelf_levels']), 'Shelf outside rack')
    require(all(0 <= level < len(r['shelf_levels']) for level in r['loaded_levels']), 'Unknown shelf')
    require(c['racks']['pallets_per_level'] == 1, 'One pallet per shelf currently supported')
    load = c['load']; box = c['box']['size']; pallet = c['pallet']['size']
    require(all(isinstance(load[k], int) and load[k] > 0 for k in ('columns', 'rows', 'layers')), 'Invalid load grid')
    require(load['columns']*box[0]+(load['columns']-1)*load['gap'] < pallet[0], 'Load exceeds pallet X')
    require(load['rows']*box[1]+(load['rows']-1)*load['gap'] < pallet[1], 'Load exceeds pallet Y')
    require(pallet[0] < r['size'][0]-2*r['post'] and pallet[1] < r['size'][1]-2*r['post'], 'Pallet does not fit rack')
    load_height = pallet[2]+box[2]*load['layers']
    for level in r['loaded_levels']:
        top = r['shelf_levels'][level]+r['shelf_thickness']/2+load_height
        ceiling = r['shelf_levels'][level+1] if level+1 < len(r['shelf_levels']) else r['size'][2]
        require(top < ceiling-r['shelf_thickness']/2, 'Load intersects next shelf')
    require(b['height'] > r['size'][2], 'Racks exceed building height')
    obstacles = rack_bounds(c)
    for item in obstacles:
        x,y,sx,sy = item
        require(abs(x)+sx/2 < b['length']/2 and abs(y)+sy/2 < b['width']/2, 'Rack outside building')
    for i, (x,y,sx,sy) in enumerate(obstacles):
        for X,Y,SX,SY in obstacles[i+1:]:
            require(abs(x-X) >= (sx+SX)/2-1e-6 or abs(y-Y) >= (sy+SY)/2-1e-6, 'Racks overlap')
    ids = [s['id'] for key in ('stations', 'spawns', 'humans', 'floor_loads') for s in c[key]]
    require(len(ids) == len(set(ids)), 'Duplicate identity')
    require(all(re.fullmatch('[a-z][a-z0-9_]*', name) for name in ids+[c['world_name']]), 'Invalid identity')
    nav = c['navigation']
    turning = math.hypot(amr['chassis_length'], amr['chassis_width'])+2*nav['lateral_clearance']
    for aisle in nav['aisles']:
        n = aisle['capacity']
        needed = n*amr['chassis_width']+2*nav['lateral_clearance']+(n-1)*nav['inter_robot_clearance']
        require(min(aisle['size']) >= max(needed, turning), 'Aisle clearance insufficient: '+aisle['id'])
        ax,ay = aisle['centre']; sx,sy = aisle['size']
        for x,y,rx,ry in obstacles:
            require(abs(x-ax) >= (rx+sx)/2-1e-6 or abs(y-ay) >= (ry+sy)/2-1e-6, 'Rack blocks aisle '+aisle['id'])
    require(sum(s['active'] for s in c['spawns']) == 1, 'Exactly one active AMR supported')
    for key in ('spawns', 'stations', 'floor_loads', 'humans'):
        for item in c[key]:
            require(len(item['pose']) == 6, 'Pose must contain XYZ RPY')
            x,y = item.get('goal_pose', item['pose'])[:2]
            require(abs(x) < b['length']/2 and abs(y) < b['width']/2, 'Pose outside building')
            if key in ('spawns', 'stations'):
                radius = math.hypot(amr['chassis_length'], amr['chassis_width'])/2+nav['lateral_clearance']
                for X,Y,SX,SY in obstacles:
                    require(abs(x-X) >= SX/2+radius or abs(y-Y) >= SY/2+radius, 'Goal/spawn too close to rack: '+item['id'])
    for human in c['humans']:
        require(not human['moving'] or len(human['route']) >= 2, 'Moving human needs a route')
        validate_route(c, human)
    for door in b['doors']:
        require(door['wall'] in ('west','east','north','south'), 'Unknown door wall')
        span = b['width'] if door['wall'] in ('west','east') else b['length']
        require(0 < door['width'] < span and 0 < door['height'] < b['height'], 'Invalid door opening')
        require(abs(door['centre'])+door['width']/2 < span/2, 'Door exceeds wall')
    cam = c['camera']
    require(0 < cam['horizontal_fov'] < math.pi and 0 < cam['near'] < cam['far'], 'Invalid camera optics')
    require(cam['rate'] > 0 and all(isinstance(cam[k], int) and cam[k] > 0 for k in ('width','height')), 'Invalid camera acquisition')
    h = c['human_model']
    require(h['mass'] > 0 and h['radius'] > 0 and h['height'] > 0 and min(h['inertia']) > 0, 'Invalid human physics')
    require(h['force'] > 0 and h['torque'] > 0 and h['friction'] >= 0, 'Invalid human drive')
    # Check every docking/spawn envelope against all permanent obstructions.
    radius = math.hypot(amr['chassis_length'], amr['chassis_width'])/2
    for item in c['stations']+c['spawns']:
        x,y = item.get('goal_pose',item['pose'])[:2]
        require(abs(x)+radius < b['length']/2 and abs(y)+radius < b['width']/2,
                'Goal/spawn intersects wall: '+item['id'])
        for X,Y,SX,SY in static_bounds(c):
            dx,dy = max(abs(x-X)-SX/2,0),max(abs(y-Y)-SY/2,0)
            require(math.hypot(dx,dy) > radius, 'Goal/spawn intersects obstruction: '+item['id'])
        for human in c['humans']:
            require(math.dist((x,y),human['pose'][:2]) > radius+h['radius'],
                    'Goal/spawn intersects human: '+item['id'])


def rack_bounds(c):
    return [(x,y,*c['racks']['size'][:2]) for x in c['racks']['row_x'] for y in c['racks']['bay_y']]


def static_bounds(c):
    """Conservative XY bounds for route/pose validation, including local yaw."""
    def bound(pose,size):
        cs,sn = abs(math.cos(pose[5])),abs(math.sin(pose[5]))
        return (*pose[:2],cs*size[0]+sn*size[1],sn*size[0]+cs*size[1])
    return (rack_bounds(c)
            + [bound(load['pose'],c['pallet']['size']) for load in c['floor_loads']]
            + [(*xy,*c['building']['pillars']['size']) for xy in c['building']['pillars']['positions']]
            + [bound(c['charger']['pose'],c['charger']['size'])]
            + [(*c['camera']['pose'][:2],2*c['camera']['mast_radius'],2*c['camera']['mast_radius'])])


def validate_route(c, human):
    points = [human['pose'][:2], *human['route']]
    if len(points) == 1:
        points.append(points[0])
    elif human['loop']:
        points.append(human['route'][0])
    radius = c['human_model']['radius']
    for a,b in zip(points, points[1:]):
        # Conservative sampled segment validation at <= radius/2 spacing.
        steps = max(1, math.ceil(math.dist(a,b)/(radius/2)))
        for step in range(steps+1):
            x,y = [a[k]+(b[k]-a[k])*step/steps for k in (0,1)]
            if abs(x)+radius >= c['building']['length']/2 or abs(y)+radius >= c['building']['width']/2:
                raise ValueError('Human route outside walls: '+human['id'])
            for X,Y,SX,SY in static_bounds(c):
                if abs(x-X) < SX/2+radius and abs(y-Y) < SY/2+radius:
                    raise ValueError('Human route intersects obstruction: '+human['id'])


def make_assets(c, output):
    a = c['appearance']; ps = c['pallet']['size']; bs = c['box']['size']
    p = model('wh_pallet'); link = tag(p, 'link', name='body')
    # One simple collision, slats/runners only in the visual representation.
    col = tag(link, 'collision', name='pallet_collision')
    tag(col, 'pose', [0,0,ps[2]/2,0,0,0]); geometry(col, 'box', ps)
    slats = c['pallet']['slat_count']; t = c['pallet']['slat_thickness']
    for i in range(slats):
        y = -ps[1]/2+(i+0.5)*ps[1]/slats
        shape(link, f'slat_{i}', [ps[0],ps[1]/slats*0.8,t], [0,y,ps[2]-t/2], a['wood'], False)
    for i in range(3):
        x = (i-1)*(ps[0]-c['pallet']['runner_width'])/2
        shape(link, f'runner_{i}', [c['pallet']['runner_width'],ps[1],ps[2]-t], [x,0,(ps[2]-t)/2], a['wood'], False)
    write_model(output, 'pallet', p)
    box = model('wh_box'); link = tag(box,'link',name='body')
    shape(link,'carton',bs,[0,0,bs[2]/2],a['cardboard'])
    shape(link,'tape',[c['box']['tape_width'],bs[1],t/10],[0,0,bs[2]+t/20],a['tape'],False)
    write_model(output,'box',box)
    load = model('wh_loaded_pallet'); include(load,'pallet','pallet',[0]*6)
    grid = c['load']
    for z in range(grid['layers']):
        for x in range(grid['columns']):
            for y in range(grid['rows']):
                include(load,'box',f'box_{z}_{x}_{y}',[(x-(grid['columns']-1)/2)*(bs[0]+grid['gap']),
                    (y-(grid['rows']-1)/2)*(bs[1]+grid['gap']),ps[2]+z*bs[2],0,0,0])
    write_model(output,'loaded_pallet',load)
    r = c['racks']; sx,sy,sz = r['size']; post = r['post']
    rack = model('wh_rack'); link = tag(rack,'link',name='structure')
    for i,x in enumerate((-1,1)):
        for j,y in enumerate((-1,1)):
            shape(link,f'post_{i}_{j}',[post,post,sz],[x*(sx-post)/2,y*(sy-post)/2,sz/2],a['rack_post'])
        # Low kick rails are visible at the actual AMR laser plane, z=0.213 m.
        shape(link,f'kickrail_{i}',[post,sy,r['collision_rail_height']],
              [x*(sx-post)/2,0,r['collision_rail_height']/2],a['rack_beam'])
    for n,z in enumerate(r['shelf_levels']):
        shape(link,f'shelf_{n}',[sx,sy,r['shelf_thickness']],[0,0,z],a['steel'])
        for i,x in enumerate((-1,1)):
            shape(link,f'beam_{n}_{i}',[post,sy,r['beam_height']],[x*(sx-post)/2,0,z],a['rack_beam'],False)
    for level in r['loaded_levels']:
        include(rack,'loaded_pallet',f'load_{level}',[0,0,r['shelf_levels'][level]+r['shelf_thickness']/2,0,0,0])
    write_model(output,'rack',rack)
    obstacle = c['obstacles']['barrier']; barrier = model('wh_barrier')
    link = tag(barrier,'link',name='body'); sx,sy,sz = obstacle['size']
    shape(link,'solid',[sx,sy,sz],[0,0,0],a['yellow'])
    for i in range(obstacle['stripe_count']):
        if i%2 == 0:
            x = -sx/2+(i+0.5)*sx/obstacle['stripe_count']
            for side in (-1,1):
                shape(link,f'stripe_{i}_{side}',[sx/obstacle['stripe_count'],0.002,sz],
                      [x,side*(sy/2+0.001),0],a['steel'],False)
    add_contact(link,'barrier','solid_collision',c['human_model']['contact_rate'],'/warehouse/contacts/barrier')
    write_model(output,'barrier',barrier)


def add_contact(link, name, collision, rate, topic):
    sensor = tag(link, 'sensor', name=name+'_contact', type='contact')
    tag(sensor,'always_on',True); tag(sensor,'update_rate',rate); tag(sensor,'topic',topic)
    contact = tag(sensor,'contact')
    tag(contact,'collision',collision)
    # Sim 8 Contact reads contact/topic; sensor/topic alone is ignored.
    tag(contact,'topic',topic)


def human_model(c, human):
    h = c['human_model']; a = c['appearance']; height = h['height']; radius = h['radius']
    m = model(human['id'], not human['moving'], human['pose']); link = tag(m,'link',name='body')
    inertia = tag(link,'inertial'); tag(inertia,'pose',[0,0,h['com_height'],0,0,0]); tag(inertia,'mass',h['mass'])
    tensor = tag(inertia,'inertia')
    for key,value in zip(('ixx','iyy','izz'),h['inertia']): tag(tensor,key,value)
    for key in ('ixy','ixz','iyz'): tag(tensor,key,0)
    collision = tag(link,'collision',name='body_collision')
    tag(collision,'pose',[0,0,height/2,0,0,0]); geometry(collision,'cylinder',[radius,height])
    friction = tag(tag(tag(collision,'surface'),'friction'),'ode')
    tag(friction,'mu',h['friction']); tag(friction,'mu2',h['friction'])
    # Two legs meet the laser plane; solid conservative body collision spans the gap.
    leg_h = height*h['leg_height_fraction']
    for side in (-1,1):
        shape(link,f'leg_{side}',[radius*0.8,radius*0.65,leg_h],[0,side*radius*0.48,leg_h/2],a['trousers'],False)
        shape(link,f'shoe_{side}',[radius*1.1,radius*0.7,height*0.045],[radius*0.1,side*radius*0.48,height*0.0225],a['steel'],False)
        shape(link,f'arm_{side}',[radius*0.48,radius*0.45,height*0.31],[0,side*radius*0.83,height*0.62],a['yellow'],False)
    shape(link,'torso',[radius*1.15,radius*1.4,height*0.32],[0,0,height*0.62],a['yellow'],False)
    shape(link,'reflective_band',[radius*1.16,radius*1.41,height*0.028],[0,0,height*0.58],a['white'],False)
    head_r = height*h['head_radius_fraction']
    shape(link,'head',[head_r],[0,0,height-head_r],a['skin'],False,'sphere')
    shape(link,'helmet',[head_r*1.06,head_r*0.3],[0,0,height-head_r*0.24],a['yellow'],False,'cylinder')
    add_contact(link,human['id'],'body_collision',h['contact_rate'],'/warehouse/contacts/'+human['id'])
    if human['moving']:
        plugin = tag(m,'plugin',filename='gz-sim-trajectory-follower-system',name='gz::sim::systems::TrajectoryFollower')
        tag(plugin,'link_name','body'); tag(plugin,'loop',human['loop'])
        for key in ('force','torque','range_tolerance'): tag(plugin,key,h[key])
        tag(plugin,'bearing_tolerance',h['bearing_tolerance_degrees'])
        tag(plugin,'zero_vel_on_bearing_reached',True)
        waypoints = tag(plugin,'waypoints')
        for point in human['route']: tag(waypoints,'waypoint',point)
    return m


def label_asset(output, name, text, c):
    """Create our own tiny textured quad, no Fuel/Blender assets or network."""
    from PIL import Image, ImageDraw, ImageFont
    a = c['appearance']; directory = output/'models/wh_signs/meshes'; directory.mkdir(parents=True,exist_ok=True)
    im = Image.new('RGB',tuple(a['sign_pixels']),(22,36,47)); draw = ImageDraw.Draw(im)
    font = ImageFont.truetype(a['font'],a['sign_font_size'])
    draw.text((im.width/2,im.height/2),text,font=font,anchor='mm',fill=(239,241,235))
    im.save(directory/(name+'.png'), optimize=False)
    w,h = a['sign_size']; w/=2; h/=2
    # Standard Collada material/UVs, supported by both Ogre renderers.
    dae = f'''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="img"><init_from>{name}.png</init_from></image></library_images>
<library_effects><effect id="fx"><profile_COMMON><newparam sid="surface"><surface type="2D"><init_from>img</init_from></surface></newparam><newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam><technique sid="common"><phong><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse></phong></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>
<library_geometries><geometry id="quad"><mesh>
<source id="pos"><float_array id="positions" count="12">{-w} {-h} 0 {w} {-h} 0 {w} {h} 0 {-w} {h} 0</float_array><technique_common><accessor source="#positions" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uvs" count="8">0 0 1 0 1 1 0 1</float_array><technique_common><accessor source="#uvs" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<source id="normal"><float_array id="normals" count="3">0 0 1</float_array><technique_common><accessor source="#normals" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#pos"/></vertices>
<triangles count="2" material="mat"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><input semantic="NORMAL" source="#normal" offset="2"/><p>0 0 0 1 1 0 2 2 0 0 0 0 2 2 0 3 3 0</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#quad"><bind_material><technique_common><instance_material symbol="mat" target="#mat"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''
    (directory/(name+'.dae')).write_text(dae+'\n')


def label(link, name, xyz, c, rpy=(0,0,0)):
    shape(link,name,'model://wh_signs/meshes/'+name+'.dae',xyz,None,False,'mesh',rpy)


def building_model(c, output):
    b = c['building']; a = c['appearance']; L,W,H,t = [b[k] for k in ('length','width','height','wall_thickness')]
    m = model('wh_building'); link = tag(m,'link',name='structure')
    shape(link,'floor',[L+2*b['apron'],W+2*b['apron'],b['floor_thickness']],[0,0,-b['floor_thickness']/2],a['floor'])
    for wall in ('west','east','north','south'):
        vertical = wall in ('west','east'); span = W if vertical else L
        sign = -1 if wall in ('west','south') else 1
        fixed = sign*((L if vertical else W)+t)/2
        doors = sorted((d for d in b['doors'] if d['wall']==wall),key=lambda d:d['centre'])
        start = -span/2
        for i,door in enumerate([*doors,{'centre':span/2,'width':0,'height':0}]):
            end = door['centre']-door['width']/2
            if end < start: raise ValueError('Overlapping doors')
            if end > start:
                size = [t,end-start,H] if vertical else [end-start,t,H]
                xyz = [fixed,(start+end)/2,H/2] if vertical else [(start+end)/2,fixed,H/2]
                shape(link,f'{wall}_wall_{i}',size,xyz,a['wall'])
            if door['height']:
                size = [t,door['width'],H-door['height']] if vertical else [door['width'],t,H-door['height']]
                xyz = [fixed,door['centre'],(H+door['height'])/2] if vertical else [door['centre'],fixed,(H+door['height'])/2]
                shape(link,door['id']+'_lintel',size,xyz,a['steel'])
            start = door['centre']+door['width']/2
    for i,xy in enumerate(b['pillars']['positions']):
        shape(link,f'pillar_{i:02d}',[*b['pillars']['size'],H],[*xy,H/2],a['steel'])
        shape(link,f'pillar_guard_{i:02d}',[*(v+0.015 for v in b['pillars']['size']),0.55],[*xy,0.275],a['yellow'],False)
    for i,x in enumerate(b['roof_beams']['x']):
        shape(link,f'roof_beam_{i}',[b['roof_beams']['width'],W,b['roof_beams']['height']],
              [x,0,H-b['roof_beams']['height']/2],a['steel'])
    # Roof intentionally open for overview and affordable lighting, beams mark structure.
    for i,aisle in enumerate(c['navigation']['aisles']):
        sx,sy = aisle['size']; x,y = aisle['centre']; z = a['marking_height']/2
        for side in (-1,1):
            if sx > sy:
                shape(link,f'aisle_{i}_{side}',[sx,a['marking_width'],a['marking_height']],
                      [x,y+side*(sy/2-a['marking_width']/2),z],a['yellow'],False)
            else:
                shape(link,f'aisle_{i}_{side}',[a['marking_width'],sy,a['marking_height']],
                      [x+side*(sx/2-a['marking_width']/2),y,z],a['yellow'],False)
    # Contact sensors provide positive collision evidence in the acceptance tests.
    add_contact(link,'south_wall','south_wall_0_collision',c['human_model']['contact_rate'],'/warehouse/contacts/wall')
    charger = c['charger']; shape(link,'charger',charger['size'],charger['pose'][:3],a['charging'],rpy=charger['pose'][3:])
    for i,pos in enumerate(c['lighting']['overhead_positions']):
        shape(link,f'luminaire_{i}',c['lighting']['fixture_size'],pos,a['white'],False)
    for row,x in enumerate(c['racks']['row_x']):
        for bay,y in enumerate(c['racks']['bay_y']):
            ident = f'rack_{row+1:02d}_{bay+1:02d}'
            include(m,'rack',ident,[x,y,0,0,0,0])
    for load in c['floor_loads']: include(m,'loaded_pallet',load['id'],load['pose'])
    for station in c['stations']:
        station_model = model(station['id'],pose=station['pose']); sl = tag(station_model,'link',name='markings')
        sx,sy = station['size']; z = a['marking_height']/2; colour = a[station['colour']]
        for side in (-1,1):
            shape(sl,'edge_x_'+str(side),[a['marking_width'],sy,a['marking_height']],[side*sx/2,0,z],colour,False)
            shape(sl,'edge_y_'+str(side),[sx,a['marking_width'],a['marking_height']],[0,side*sy/2,z],colour,False)
        # Dock centre cross; no collision geometry in the docking footprint.
        shape(sl,'target_x',[sx/4,a['marking_width'],a['marking_height']],[0,0,z],colour,False)
        shape(sl,'target_y',[a['marking_width'],sy/4,a['marking_height']],[0,0,z],colour,False)
        label_asset(output,station['id'],station['label'],c)
        label(sl,station['id'],[0,-sy/2+a['sign_size'][1]/2,z*2],c)
        m.append(station_model)
    write_model(output,'building',m)


def camera_model(c):
    cam = c['camera']; a = c['appearance']; x,y,z,roll,pitch,yaw = cam['pose']
    m = model(cam['id'],pose=[x,y,0,0,0,0]); link = tag(m,'link',name='mount')
    shape(link,'mast',[cam['mast_radius'],z],[0,0,z/2],a['steel'],True,'cylinder')
    # Housing sits behind the optical origin, avoiding self-occlusion.
    size = cam['housing_size']; back = size[0]/2+cam['near']
    offset = [-back*math.cos(pitch)*math.cos(yaw),-back*math.cos(pitch)*math.sin(yaw),z+back*math.sin(pitch)]
    shape(link,'housing',size,offset,a['steel'],True,rpy=[roll,pitch,yaw])
    sensor = tag(link,'sensor',name='camera',type='camera')
    tag(sensor,'pose',[0,0,z,roll,pitch,yaw]); tag(sensor,'topic',cam['topic']+'/image')
    tag(sensor,'always_on',True); tag(sensor,'update_rate',cam['rate'])
    cc = tag(sensor,'camera'); tag(cc,'horizontal_fov',cam['horizontal_fov'])
    img = tag(cc,'image'); tag(img,'width',cam['width']); tag(img,'height',cam['height']); tag(img,'format','R8G8B8')
    clip = tag(cc,'clip'); tag(clip,'near',cam['near']); tag(clip,'far',cam['far'])
    tag(cc,'optical_frame_id',cam['optical_frame'])
    return m


def make_world(c, scenario):
    sdf = ET.Element('sdf',version='1.9'); w = tag(sdf,'world',name=c['world_name'])
    tag(w,'gravity',c['simulation']['gravity'])
    physics = tag(w,'physics',name='physics',type='ignored')
    for key in ('max_step_size','real_time_factor'): tag(physics,key,c['simulation'][key])
    for filename,system in (('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),
                            ('sensors','Sensors'),('imu','Imu'),('contact','Contact')):
        p = tag(w,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+system)
        if system=='Sensors': tag(p,'render_engine','ogre2')
    lit = c['lighting']; scene = tag(w,'scene')
    for key in ('ambient','background','shadows'): tag(scene,key,lit[key])
    sun = tag(w,'light',name='daylight',type='directional')
    tag(sun,'pose',[0,0,c['building']['height']*2,0,0,0]); tag(sun,'direction',lit['sun_direction'])
    tag(sun,'diffuse',lit['sun_diffuse']); tag(sun,'specular',[0.1,0.1,0.1,1]); tag(sun,'cast_shadows',lit['shadows'])
    for i,xyz in enumerate(lit['overhead_positions']):
        lamp = tag(w,'light',name=f'overhead_{i}',type='point'); tag(lamp,'pose',[*xyz,0,0,0])
        tag(lamp,'diffuse',lit['overhead_diffuse']); tag(lamp,'cast_shadows',False)
        at = tag(lamp,'attenuation'); tag(at,'range',lit['range'])
        for key,value in zip(('constant','linear','quadratic'),lit['attenuation']): tag(at,key,value)
    include(w,'building','warehouse_structure',[0]*6)
    w.append(camera_model(c))
    for human in c['humans']:
        hh = {**human,**scenario['human_overrides'].get(human['id'],{})}
        validate_route(c,hh); w.append(human_model(c,hh))
    if scenario['aisle_obstacle']:
        o = c['obstacles']['aisle_load']; include(w,'loaded_pallet',o['id'],o['pose'])
    if scenario['corridor_blocked']:
        o = c['obstacles']['barrier']; include(w,'barrier',o['id'],o['pose'])
    return sdf


def generate(config_path=ROOT/'config/warehouse_layout.json', output=ROOT, amr_path=None):
    output = Path(output); c,amr = load_config(config_path,amr_path)
    scenarios = {}
    for name in SCENARIOS:
        path = Path(config_path).parent/'scenarios'/(name+'.json')
        s = json.loads(path.read_text())
        if s['name'] != name or not isinstance(s['aisle_obstacle'],bool) or not isinstance(s['corridor_blocked'],bool):
            raise ValueError('Invalid scenario '+name)
        if set(s['human_overrides'])-set(h['id'] for h in c['humans']): raise ValueError('Unknown human override')
        scenarios[name] = s
    make_assets(c,output); building_model(c,output)
    for name,s in scenarios.items(): save_xml(output/'worlds'/('warehouse_'+name+'.sdf'),make_world(c,s))
    cam = c['camera']
    bridge = [{'ros_topic_name':cam['topic']+'/image_raw','gz_topic_name':cam['topic']+'/image',
               'ros_type_name':'sensor_msgs/msg/Image','gz_type_name':'gz.msgs.Image','direction':'GZ_TO_ROS','qos_profile':'SENSOR_DATA','frame_id':cam['optical_frame']},
              {'ros_topic_name':cam['topic']+'/camera_info','gz_topic_name':cam['topic']+'/camera_info',
               'ros_type_name':'sensor_msgs/msg/CameraInfo','gz_type_name':'gz.msgs.CameraInfo','direction':'GZ_TO_ROS','qos_profile':'SENSOR_DATA','frame_id':cam['optical_frame']}]
    (output/'config').mkdir(parents=True,exist_ok=True)
    (output/'config/warehouse_bridge.yaml').write_text(json.dumps(bridge,indent=2)+'\n')
    goals = {'frame_id':c['coordinates']['frame'],'note':'World ground truth. Register future SLAM map to world before using these poses as map goals.',
             'stations':{s['id']:{'xyz_rpy':s.get('goal_pose',s['pose']), 'footprint':s['size']} for s in c['stations']}, 'spawns':c['spawns']}
    (output/'config/station_poses.json').write_text(json.dumps(goals,indent=2)+'\n')
    # Reuse the validated GUI layout; only the overview pose/title/window size change.
    desc = ROOT.parent/'amr_description'
    if not (desc/'config/gazebo_gui.config').is_file():
        from ament_index_python.packages import get_package_share_directory
        desc = Path(get_package_share_directory('amr_description'))
    gui = (desc/'config/gazebo_gui.config').read_text()
    gui = re.sub(r'<camera_pose>.*?</camera_pose>','<camera_pose>'+fmt(c['gui']['camera_pose'])+'</camera_pose>',gui)
    gui = gui.replace('<title>Warehouse AMR</title>','<title>Industrial warehouse</title>')
    gui = gui.replace('<width>1280</width>',f'<width>{c["gui"]["width"]}</width>').replace('<height>800</height>',f'<height>{c["gui"]["height"]}</height>')
    (output/'config/warehouse_gui.config').write_text(gui)
    for name in SCENARIOS:
        identified = gui.replace('<title>Industrial warehouse</title>',
            '<title>Warehouse — '+name+'</title>')
        identified = identified.replace('key="showTitleBar">false',
                                        'key="showTitleBar">true', 1)
        (output/'config'/('warehouse_gui_'+name+'.config')).write_text(identified)
    from render_layout import render
    render(c,output/'config/warehouse_layout.svg',amr)
    n_racks = len(rack_bounds(c)); n_pallets = n_racks*len(c['racks']['loaded_levels'])+len(c['floor_loads'])
    manifest = {'schema_version':1,'config_sha256':hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
                'source_sha256':{'config/warehouse_layout.json':hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
                    **{'config/scenarios/'+name+'.json':hashlib.sha256((Path(config_path).parent/'scenarios'/(name+'.json')).read_bytes()).hexdigest()
                       for name in SCENARIOS}},
                'amr_config_sha256':hashlib.sha256(json.dumps(amr,sort_keys=True).encode()).hexdigest(),
                'amr_dimensions':{k:amr[k] for k in ('chassis_length','chassis_width','chassis_height')},
                'turning_diameter_with_clearance':math.hypot(amr['chassis_length'],amr['chassis_width'])+2*c['navigation']['lateral_clearance'],
                'racks':n_racks,'pallets_normal':n_pallets,'boxes_normal':n_pallets*c['load']['columns']*c['load']['rows']*c['load']['layers'],
                'humans':len(c['humans']),'files':{}}
    # Inventory only compiler-owned paths; unrelated files must be preserved.
    owned = [p for directory in (output/'models').glob('wh_*') for p in directory.rglob('*') if p.is_file()]
    owned += [output/'worlds'/('warehouse_'+name+'.sdf') for name in SCENARIOS]
    for path in sorted(owned):
        manifest['files'][str(path.relative_to(output))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name in ('warehouse_bridge.yaml','station_poses.json','warehouse_gui.config','warehouse_layout.svg'):
        path = output/'config'/name; manifest['files']['config/'+name] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name in SCENARIOS:
        relative = 'config/warehouse_gui_'+name+'.config'
        manifest['files'][relative] = hashlib.sha256((output/relative).read_bytes()).hexdigest()
    (output/'config/generated_manifest.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/warehouse_layout.json')
    parser.add_argument('--output',type=Path,default=ROOT)
    parser.add_argument('--amr-config',type=Path)
    args = parser.parse_args()
    result = generate(args.config,args.output,args.amr_config)
    print(json.dumps({k:v for k,v in result.items() if k!='files'},indent=2))
