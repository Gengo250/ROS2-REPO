"""Regression tests for geometry contracts and the compiler, independent of GUI."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from generate_warehouse import generate, load_config, validate, make_world
from validate_warehouse import validate_tree


def test_reproducible_offline_and_preserves_external_file(tmp_path):
    sentinel = tmp_path/'do_not_touch.txt'
    sentinel.write_text('user data')
    first = generate(output=tmp_path)
    second = generate(output=tmp_path)
    assert first == second
    assert sentinel.read_text() == 'user data'
    for name, digest in first['files'].items():
        assert hashlib.sha256((tmp_path/name).read_bytes()).hexdigest() == digest
    for path in tmp_path.rglob('*.sdf'):
        validate_tree(path)


@pytest.mark.parametrize('change', ['narrow_aisle', 'duplicate', 'overlap', 'bad_route', 'bad_camera', 'bad_load', 'nan',
                                   'spawn_in_load', 'spawn_in_wall', 'human_in_pillar'])
def test_rejects_invalid_configuration(change):
    c,amr = load_config()
    if change == 'narrow_aisle': c['navigation']['aisles'][0]['size'][1] = 1
    if change == 'duplicate': c['spawns'][1]['id'] = c['spawns'][0]['id']
    if change == 'overlap': c['racks']['row_x'][1] = c['racks']['row_x'][0]+0.2
    if change == 'bad_route': c['humans'][0]['route'] = [[-6.6,-5.75],[-8.7,3]]
    if change == 'bad_camera': c['camera']['near'] = c['camera']['far']
    if change == 'bad_load': c['load']['columns'] = 10
    if change == 'nan': c['building']['height'] = float('nan')
    if change == 'spawn_in_load': c['spawns'][0]['pose'] = c['floor_loads'][0]['pose'][:]
    if change == 'spawn_in_wall': c['spawns'][0]['pose'][0] = c['building']['length']/2-.1
    if change == 'human_in_pillar': c['humans'][2]['pose'][:2] = c['building']['pillars']['positions'][0]
    with pytest.raises(ValueError): validate(c,amr)


def test_rejects_larger_amr_without_widening_aisles():
    c,amr = load_config()
    amr['chassis_width'] = 1.5
    with pytest.raises(ValueError,match='clearance'): validate(c,amr)


def test_scenario_composition_and_physical_people():
    c,_ = load_config()
    for path in sorted((ROOT/'config/scenarios').glob('*.json')):
        s = json.loads(path.read_text()); root = make_world(c,s)
        world = root.find('world')
        assert len(world.findall('actor')) == 0
        for h in c['humans']:
            person = world.find(f"model[@name='{h['id']}']")
            assert person.find('link/collision/geometry/cylinder') is not None
            assert person.find('link/visual') is not None
            assert person.find('link/sensor[@type="contact"]') is not None
        names = [inc.findtext('name') for inc in world.findall('include')]
        assert ('corridor_barrier' in names) == s['corridor_blocked']
        assert ('aisle_pallet' in names) == s['aisle_obstacle']


@pytest.mark.parametrize('changed', ['scenario', 'world', 'amr'])
def test_launch_rejects_stale_inputs(tmp_path, changed):
    share = tmp_path/'amr_simulation'
    shutil.copytree(ROOT/'config',share/'config')
    amr = tmp_path/'amr_description'
    (amr/'config').mkdir(parents=True)
    amr_file = amr/'config/amr_dimensions.json'
    shutil.copy2(ROOT.parent/'amr_description/config/amr_dimensions.json',amr_file)
    generate(share/'config/warehouse_layout.json',share,amr_file)
    spec = importlib.util.spec_from_file_location('warehouse_launch',ROOT/'launch/warehouse.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.verify_generated(share,amr)
    if changed == 'scenario':
        path = share/'config/scenarios/person_crossing.json'
        data = json.loads(path.read_text()); data['human_overrides']['human_01']['pose'][1] = -3
        path.write_text(json.dumps(data))
    elif changed == 'world':
        path = share/'worlds/warehouse_normal.sdf'; path.write_text(path.read_text()+'\n')
    else:
        data = json.loads(amr_file.read_text()); data['chassis_width'] = .9
        amr_file.write_text(json.dumps(data))
    with pytest.raises(RuntimeError,match='changed'):
        module.verify_generated(share,amr)
