"""Execute launch composition, installation and scenario isolation contracts."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.events.process import ProcessExited

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from generate_warehouse import SCENARIOS, generate, load_config, make_world, validate


def module(path):
    spec = importlib.util.spec_from_file_location('subject', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(**values):
    c = LaunchContext()
    c.launch_configurations.update(values)
    return c


def warehouse(scenario):
    return module(ROOT/'launch/warehouse.launch.py').setup(context(
        scenario=scenario, gui='false', gui_renderer='ogre', rviz='false'))


def test_scenario_argument_and_invalid_value():
    ld = module(ROOT/'launch/warehouse.launch.py').generate_launch_description()
    assert 'scenario' in [a.name for a in ld.entities if isinstance(a, DeclareLaunchArgument)]
    with pytest.raises(ValueError, match='Unknown'):
        warehouse('typo')


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_actual_selected_installed_world_and_amr_include(scenario):
    include = next(a for a in warehouse(scenario) if isinstance(a, IncludeLaunchDescription))
    args = dict(include.launch_arguments)
    share = Path(get_package_share_directory('amr_simulation'))
    assert args['world'] == str(share/'worlds'/f'warehouse_{scenario}.sdf')
    assert args['gui_config'] == str(share/'config'/f'warehouse_gui_{scenario}.config')
    assert args['world_name'] == ET.parse(args['world']).find('world').get('name')
    include.launch_description_source.get_launch_description(context())
    assert include.launch_description_source.location.endswith('amr_description/launch/sim.launch.py')
    c, amr = load_config()
    validate(c, amr)
    assert [float(args[k]) for k in ('spawn_x','spawn_y','spawn_z','spawn_yaw')] == [-11,0,.015,0]
    assert abs(float(args['spawn_x']))+amr['chassis_length']/2 < c['building']['length']/2
    assert abs(float(args['spawn_y']))+amr['chassis_width']/2 < c['building']['width']/2


def test_spawn_uses_existing_description_and_managed_server():
    root = Path(get_package_share_directory('amr_description'))
    m = module(root/'launch/sim.launch.py')
    c = context(gui='true', gui_renderer='ogre', rviz='false', world='/tmp/selected.sdf',
                world_name='warehouse', spawn_x='-11', spawn_y='0', spawn_z='.015',
                spawn_yaw='0', resource_path='', gui_config='/tmp/view.config')
    actions = m.setup(c)
    commands = [[perform_substitutions(c, s) for s in a.cmd] for a in actions if isinstance(a, ExecuteProcess)]
    server = next(cmd for cmd in commands if '--headless-rendering' in cmd)
    assert server[:2] == ['gz','sim'] and server[-1] == '/tmp/selected.sdf'
    spawn = next(cmd for cmd in commands if any(x.endswith('/create') for x in cmd))
    assert spawn[spawn.index('-world')+1] == 'warehouse'
    assert spawn[spawn.index('-name')+1] == 'warehouse_amr'
    assert spawn[spawn.index('-topic')+1] == 'robot_description'
    assert any(any(x.endswith('/robot_state_publisher') for x in cmd) for cmd in commands)
    # GUI must not race create. Exercise both event handlers, including failure.
    assert not any('-g' in cmd for cmd in commands)
    handlers = [a.event_handler for a in actions if isinstance(a,RegisterEventHandler)]
    spawn_action = next(a for a in actions if isinstance(a,ExecuteProcess) and
                        any(x.endswith('/create') for x in [perform_substitutions(c,s) for s in a.cmd]))
    def event(action, code=0):
        return ProcessExited(action=action,returncode=code,name='test',cmd=[],cwd=None,env=None,pid=123)
    spawn_event = event(spawn_action)
    handler = next(h for h in handlers if h.matches(spawn_event))
    controllers = handler.handle(spawn_event,c)[0]
    with pytest.raises(RuntimeError,match='spawn failed'):
        handler.handle(event(spawn_action,1),c)
    handler = next(h for h in handlers if h.matches(event(controllers)))
    gui = handler.handle(event(controllers),c)[0]
    assert '-g' in [perform_substitutions(c,s) for s in gui.cmd]
    with pytest.raises(RuntimeError,match='controllers failed'):
        handler.handle(event(controllers,1),c)


def test_structural_scenarios_and_full_width_barrier():
    c, _ = load_config()
    worlds = {s:ET.parse(ROOT/'worlds'/f'warehouse_{s}.sdf').find('world') for s in SCENARIOS}
    baseline = ET.tostring(worlds['normal'])
    for s in SCENARIOS[1:]:
        assert ET.tostring(worlds[s]) != baseline
    assert worlds['normal'].find("include[name='corridor_barrier']") is None
    assert worlds['corridor_blocked'].find("include[name='corridor_barrier']") is not None
    assert worlds['obstacle_in_aisle'].find("include[name='aisle_pallet']") is not None
    human = worlds['person_crossing'].find("model[@name='human_01']")
    assert [w.text for w in human.findall('.//waypoint')] == ['-8.7 2.5','-8.7 -2.5']
    assert human.findtext('static') == 'false'
    assert human.find('link/collision') is not None
    barrier = ET.parse(ROOT/'models/wh_barrier/model.sdf')
    assert barrier.findtext('.//collision/geometry/box/size') == '3 0.6 1.2'
    assert barrier.findtext('.//visual/geometry/box/size') == '3 0.6 1.2'
    assert c['obstacles']['barrier']['size'][0] == next(a['size'][0] for a in c['navigation']['aisles'] if a['id']=='aisle_01')


def test_installed_files_match_current_sources():
    for package in ('amr_simulation','amr_description'):
        source = ROOT.parent/package
        installed = Path(get_package_share_directory(package))
        for directory in ('launch','config','models','worlds','urdf','scripts'):
            for p in (source/directory).rglob('*'):
                if p.is_file() and '__pycache__' not in p.parts:
                    assert (installed/p.relative_to(source)).read_bytes() == p.read_bytes(), str(p)


def test_independent_order_and_default_partitions(monkeypatch, tmp_path):
    c, _ = load_config()
    expected = {}
    for order in (SCENARIOS, tuple(reversed(SCENARIOS))):
        for s in order:
            scenario = json.loads((ROOT/'config/scenarios'/f'{s}.json').read_text())
            digest = hashlib.sha256(ET.tostring(make_world(c, scenario))).hexdigest()
            assert expected.setdefault(s,digest) == digest
    monkeypatch.delenv('GZ_PARTITION', raising=False)
    partitions = []
    for _ in range(2):
        monkeypatch.delenv('GZ_PARTITION', raising=False)
        a = next(a for a in warehouse('normal') if isinstance(a,SetEnvironmentVariable))
        ctx = context(); a.execute(ctx)
        partitions.append(ctx.environment['GZ_PARTITION'])
    assert partitions[0] != partitions[1]
    generated = generate(output=tmp_path)
    for name in SCENARIOS:
        assert (tmp_path/'worlds'/f'warehouse_{name}.sdf').read_bytes() == (ROOT/'worlds'/f'warehouse_{name}.sdf').read_bytes()
