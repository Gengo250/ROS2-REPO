"""Read validated compiler outputs, preserving nested SDF transformations and IDs.

Only the warehouse compiler's local primitive/sign vocabulary is supported. Unknown
geometry or relative frames fail explicitly; never silently approximate a new asset.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from .models import WarehouseScene

SCENARIOS = ('normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked')

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def load_scene(scenario, configured_scenario=None, root=None, amr=None):
    if scenario not in SCENARIOS:
        raise ValueError('Unknown warehouse scenario')
    if root is None or amr is None:
        from ament_index_python.packages import get_package_share_directory
        root = root or Path(get_package_share_directory('amr_simulation'))
        amr = amr or Path(get_package_share_directory('amr_description'))/'config/amr_dimensions.json'
    root, amr = Path(root), Path(amr)
    config_path = root/'config/warehouse_layout.json'
    spec = importlib.util.spec_from_file_location('warehouse_compiler', root/'scripts/generate_warehouse.py')
    compiler = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compiler)
    c, dims = compiler.load_config(config_path, amr)
    if c['coordinates'] != {**c['coordinates'], 'frame': 'world', 'x': 'east', 'y': 'north', 'z': 'up', 'units': 'metres_radians'}:
        raise ValueError('Unsupported source coordinate convention')
    manifest = json.loads((root/'config/generated_manifest.json').read_text())
    for name, expected in {**manifest['source_sha256'], **manifest['files']}.items():
        if digest(root/name) != expected:
            raise ValueError('Stale warehouse generation: '+name)
    if hashlib.sha256(json.dumps(dims, sort_keys=True).encode()).hexdigest() != manifest['amr_config_sha256']:
        raise ValueError('Stale AMR dimensions: regenerate warehouse')
    goals = json.loads((root/'config/station_poses.json').read_text())
    if goals['frame_id'] != 'world' or goals['spawns'] != c['spawns']:
        raise ValueError('Station/spawn configuration mismatch')
    entities, primitives = [], []
    station_labels = {s['id']: s['label'] for s in c['stations']}

    def pose(node):
        p = node.find('pose')
        if p is not None and p.attrib:
            raise ValueError('Unsupported SDF pose attributes')
        values = [float(v) for v in (p.text if p is not None else '0 0 0 0 0 0').split()]
        if len(values) != 6:
            raise ValueError('Expected XYZ RPY')
        return values

    def walk(node, prefix='', transforms=(), kind='structure'):
        for child in node:
            if child.tag == 'include':
                uri = child.findtext('uri')
                if not uri.startswith('model://wh_') or '/' in uri[8:]:
                    raise ValueError('Unsupported model URI: '+uri)
                path = root/'models'/uri[8:]/'model.sdf'
                model = ET.parse(path).getroot().find('model')
                ident = prefix+child.findtext('name')
                entities.append(dict(id=ident, pose=pose(child), kind=uri[11:]))
                walk(model, ident+'/', (*transforms, pose(child), pose(model)), uri[11:])
            elif child.tag in ('model', 'link'):
                ident = prefix+child.attrib['name']
                if child.tag == 'model':
                    entities.append(dict(id=ident, pose=pose(child), kind='human' if ident.startswith('human_') else 'station'))
                walk(child, ident+'/', (*transforms, pose(child)), 'human' if ident.startswith('human_') else kind)
            elif child.tag == 'visual':
                geom = list(child.find('geometry'))[0]
                shape = geom.tag
                label = None
                if shape == 'box': sizes = [float(v) for v in geom.findtext('size').split()]
                elif shape == 'sphere': sizes = [float(geom.findtext('radius'))]
                elif shape == 'cylinder': sizes = [float(geom.findtext('radius')), float(geom.findtext('length'))]
                elif shape == 'mesh':
                    uri = geom.findtext('uri')
                    name = Path(uri).stem
                    if uri != f'model://wh_signs/meshes/{name}.dae' or name not in station_labels:
                        raise ValueError('Unsupported mesh: '+uri)
                    shape, sizes, label = 'label', c['appearance']['sign_size'], station_labels[name]
                else: raise ValueError('Unsupported geometry: '+shape)
                name = child.attrib['name']
                category = 'human' if kind == 'human' else 'equipment'
                if kind == 'building':
                    category = ('wall' if '_wall_' in name or '_lintel' in name else
                                'pillar' if name.startswith('pillar') else
                                'beam' if name.startswith('roof_beam') else
                                'floor' if name == 'floor_visual' else
                                'marking' if name.startswith(('aisle_', 'edge_', 'target_')) or label else 'structure')
                primitives.append(dict(id=prefix+name, kind=shape, dimensions=sizes,
                    transforms=[*transforms, pose(child)],
                    colour=[float(v) for v in child.findtext('material/diffuse', '1 1 1 1').split()],
                    label=label, category=category))
    walk(ET.parse(root/f'worlds/warehouse_{scenario}.sdf').getroot().find('world'))
    if len({p['id'] for p in primitives}) != len(primitives):
        raise ValueError('Duplicate visual ID')
    b = c['building']
    sources = {**manifest['source_sha256'], 'amr_dimensions.json': digest(amr),
               'station_poses.json': digest(root/'config/station_poses.json')}
    zones = [{k: v for k, v in a.items() if k in ('id', 'centre', 'size', 'enabled')}
             for a in c['navigation']['open_areas']+c['navigation']['restricted_zones']+
             [a for a in c['navigation']['temporary_blocked_zones'] if a['scenario'] == scenario]]
    return WarehouseScene(metadata=dict(world_id=c['world_name'], scenario=scenario,
        configured_scenario=configured_scenario, source_version=c['schema_version'], source_sha256=sources,
        geometry_sha256=hashlib.sha256(json.dumps(primitives, sort_keys=True).encode()).hexdigest()),
        building=dict(length=b['length'], width=b['width'], height=b['height'],
                      floor_size=[b['length']+2*b['apron'], b['width']+2*b['apron'], b['floor_thickness']]),
        entities=entities, primitives=primitives,
        aisles=[{k: a[k] for k in ('id', 'centre', 'size')} for a in c['navigation']['aisles']], zones=zones,
        stations=[dict(id=s['id'], label=s['label'], pose=s['pose'], goal_pose=goals['stations'][s['id']]['xyz_rpy'], footprint=s['size']) for s in c['stations']],
        robot_spawns=[dict(**s, robot_id=s['id'].removesuffix('_spawn')) for s in c['spawns']])
