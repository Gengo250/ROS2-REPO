"""Actual warehouse/compiler parity, not a second fixture warehouse."""
import json
from pathlib import Path
import shutil
import pytest
from fastapi.testclient import TestClient
from jsonschema import Draft202012Validator
from digital_twin_gateway.scene.adapter import load_scene
from digital_twin_gateway.api.app import create_app
from digital_twin_gateway.config import Settings
from conftest import Runtime

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT/'src/amr_simulation'
AMR = ROOT/'src/amr_description/config/amr_dimensions.json'

@pytest.mark.parametrize('scenario', ['normal','person_crossing','obstacle_in_aisle','corridor_blocked'])
def test_real_geometry(scenario):
    data = load_scene(scenario, root=SIM, amr=AMR)
    assert data == load_scene(scenario, root=SIM, amr=AMR)
    Draft202012Validator(json.loads((ROOT/'shared/schemas/warehouse-scene.schema.json').read_text())).validate(data.model_dump(mode='json'))
    c = json.loads((SIM/'config/warehouse_layout.json').read_text())
    assert (data.building.length, data.building.width, data.building.height) == (30,20,5)
    floor = next(p for p in data.primitives if p.category == 'floor')
    assert floor.dimensions == [34,24,.2]
    assert floor.transforms[-1] == (0,0,-.1,0,0,0)
    assert sum(p.category == 'wall' for p in data.primitives) == 8
    pillars = [p for p in data.primitives if p.category == 'pillar' and 'guard' not in p.id]
    assert len(pillars) == 10
    assert [list(p.transforms[-1][:2]) for p in pillars] == c['building']['pillars']['positions']
    racks = [e for e in data.entities if e.kind == 'rack']
    assert len(racks) == 16
    assert {(e.pose[0], e.pose[1]) for e in racks} == {(x,y) for x in c['racks']['row_x'] for y in c['racks']['bay_y']}
    assert data.robot_spawns[0].pose == (-11,0,.015,0,0,0)
    assert len(data.stations) == 6
    assert data.stations[-1].pose != data.stations[-1].goal_pose
    ids = {e.id for e in data.entities}
    assert ('aisle_pallet' in ids) == (scenario == 'obstacle_in_aisle')
    assert ('aisle_warning_cone' in ids) == (scenario == 'obstacle_in_aisle')
    assert ('corridor_barrier' in ids) == (scenario == 'corridor_blocked')
    human = next(e for e in data.entities if e.id == 'human_01')
    assert human.pose[1] == (-2.5 if scenario == 'person_crossing' else 3)


def test_stale_outputs_rejected(tmp_path):
    shutil.copytree(SIM, tmp_path/'sim')
    path = tmp_path/'sim/models/wh_rack/model.sdf'
    path.write_text(path.read_text().replace('3.2', '9.2'))
    with pytest.raises(ValueError, match='Stale warehouse generation'):
        load_scene('normal', root=tmp_path/'sim', amr=AMR)


def test_scene_api_is_separate_from_telemetry():
    with TestClient(create_app(Settings(scenario='corridor_blocked'), runtime_factory=Runtime)) as client:
        assert client.get('/api/v1/scene').json()['metadata']['scenario'] == 'corridor_blocked'
        assert client.get('/api/v1/scene?scenario=normal').json()['metadata']['configured_scenario'] == 'corridor_blocked'
        assert client.get('/api/v1/state').json()['world']['scenario'] == 'corridor_blocked'
        assert client.get('/api/v1/scene?scenario=invalid').status_code == 422
