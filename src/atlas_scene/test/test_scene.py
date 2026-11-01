import json
from pathlib import Path
import numpy as np
import pytest
from atlas_scene.scene import Scene, rasterize, rectangle

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT/'config/atlas_campus.scene.json'
WEB_COPY = ROOT.parents[1]/'web/digital-twin/public/scene/atlas_campus.scene.json'


@pytest.fixture(scope='module')
def scene():
    return Scene.load(SCENE)


def test_prototype_geometry_is_preserved(scene):
    layers = scene.raw['layers']
    count = {layer: sum(len(v)//13 for v in shapes.values()) for layer, shapes in layers.items()}
    # The ATLAS prototype logs "instances: 10659" at start-up; the enclosure is the LIVE addition.
    assert sum(v for k, v in count.items() if k != 'enclosure') == 10659
    assert count['enclosure'] == 12
    assert len(scene.raw['agents']['workers']) == 16 and len(scene.raw['agents']['forklifts']) == 3


def test_tampering_is_detected():
    data = json.loads(SCENE.read_text())
    data['layers']['racks']['cube'][0] += 0.5
    with pytest.raises(ValueError):
        Scene(data)


def test_browser_copy_is_identical():
    assert WEB_COPY.read_bytes() == SCENE.read_bytes()


def test_axes_follow_ros_world(scene):
    # "Armazenagem Norte" sits at ATLAS z = -40, i.e. north (y > 0) in the world frame.
    north = next(z for z in scene.raw['zones'] if z['name'] == 'ARMAZENAGEM NORTE')
    assert north['pos'][1] == 40
    # Dock wall (ATLAS z = 54) is the southern wall.
    dock_wall = next(s for s in scene.solids if s['tag'] == 'Parede das docas')
    assert dock_wall['y'] == pytest.approx(-53.75)


def test_floor_scanner_sees_a_closed_building(scene):
    polygons = scene.slice_polygons(0.213)
    res = 0.25
    grid = rasterize(polygons, (-75, -57), res, (int(114/res), int(150/res)))
    # Scan along each wall from inside: the enclosure must block every ray.
    for y in np.arange(-50, 50, 2.0):
        row = grid[int((y+57)/res)]
        assert row[:int((-72.3+75)/res)].any() and row[int((72.3+75)/res):].any()
    for x in np.arange(-70, 70, 2.0):
        col = grid[:, int((x+75)/res)]
        assert col[:int((-53.4+57)/res)].any() and col[int((53.4+57)/res):].any()


def test_stations_and_spawn_have_free_footprints(scene):
    body = scene.band_polygons(0.035, 0.47)
    res = 0.05
    for item in scene.raw['stations']+scene.raw['spawns']:
        x, y, yaw = item['pose'][0], item['pose'][1], item['pose'][5]
        footprint = rectangle(x+0.01, y, 0.98+0.6, 0.70+0.6, yaw)  # Nav2 footprint + 0.3 m clearance
        origin = (x-2, y-2)
        occupied = rasterize([p for p in body if np.abs(p.mean(0)-[x, y]).max() < 6], origin, res, (80, 80))
        needed = rasterize([footprint], origin, res, (80, 80))
        assert not (occupied & needed).any(), item['id']


def test_rack_envelopes_block_scanners_and_bodies(scene):
    # Mid-bay point of a rack: no upright is drawn there, but the rack envelope (prototype
    # solid) is what bodies hit and floor scanners see, as in the ATLAS prototype.
    point = (-52, 36.0)
    probe = rectangle(*point, .2, .2, 0)
    origin, shape = (point[0]-1, point[1]-1), (40, 40)
    for polygons in (scene.slice_polygons(0.213), scene.slice_polygons(0.48), scene.band_polygons(0.035, 0.47)):
        near = [p for p in polygons if np.abs(p.mean(0)-point).max() < 12]
        assert (rasterize(near, origin, .05, shape) & rasterize([probe], origin, .05, shape)).any()
    # Aisles between rack rows stay free for scanners.
    aisle = rectangle(-55, 36.0, .2, .2, 0)
    near = [p for p in scene.slice_polygons(0.213) if np.abs(p.mean(0)-(-55, 36.)).max() < 12]
    assert not (rasterize(near, (-56, 35), .05, shape) & rasterize([aisle], (-56, 35), .05, shape)).any()
