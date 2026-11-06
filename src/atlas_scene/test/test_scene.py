import json
from pathlib import Path
import numpy as np
import pytest
from atlas_scene.scene import Scene, rasterize, rectangle

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT/'config/atlas_campus.scene.json'
WEB_COPY = ROOT.parents[1]/'web/digital-twin/public/scene/atlas_campus.scene.json'


SIDES = ('west', 'east', 'north', 'south')
SHELL_LAYERS = ('enclosure',)+tuple(f'walls_{s}' for s in SIDES)


@pytest.fixture(scope='module')
def scene():
    return Scene.load(SCENE)


def test_prototype_geometry_is_preserved(scene):
    layers = scene.raw['layers']
    count = {layer: sum(len(v)//13 for v in shapes.values()) for layer, shapes in layers.items()}
    # The ATLAS prototype logs "instances: 10659" at start-up; the building shell (enclosure and
    # upper walls) is the LIVE addition.
    assert sum(v for k, v in count.items() if k not in SHELL_LAYERS) == 10659
    assert count['enclosure'] == 12
    assert [count[f'walls_{s}'] for s in SIDES] == [1, 1, 1, 9+8]   # south: 9 wall spans + 8 door lintels
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


def test_industrial_camera_lenses_are_free_of_structure(scene):
    # The first-person view renders from the lens: it must not sit inside a wall, column or rack,
    # and the first metre along its optical axis must be clear.
    i = scene.section('building')['interior']
    for cam in scene.section('cameras'):
        x, y, z = cam['position']
        assert i['x_min'] < x < i['x_max'] and i['y_min'] < y < i['y_max'], cam['id']
        c, s, cp, sp = np.cos(cam['yaw']), np.sin(cam['yaw']), np.cos(cam['pitch']), np.sin(cam['pitch'])
        for t in np.arange(0., 1.01, .1):
            px, py, pz = x+cp*c*t, y+cp*s*t, z-sp*t
            hit = [p for p in scene.slice_polygons(pz) if contains(p, px, py)]
            assert not hit, (cam['id'], round(t, 1))


def contains(polygon, x, y):
    """Even-odd point-in-polygon test."""
    inside, j = False, len(polygon)-1
    for k in range(len(polygon)):
        (xi, yi), (xj, yj) = polygon[k], polygon[j]
        if (yi > y) != (yj > y) and x < (xj-xi)*(y-yi)/(yj-yi)+xi:
            inside = not inside
        j = k
    return inside


def shell_boxes(scene, layers=SHELL_LAYERS):
    rows = []
    for layer in layers:
        values = np.asarray(scene.raw['layers'][layer]['cube'], dtype=float).reshape(-1, 13)
        assert not values[:, 10].any()            # the shell is axis-aligned
        rows.append(values[:, :6])
    return np.vstack(rows)


def covered(boxes, x, y, z):
    x0, y0, z0, length, width, height = boxes.T
    return bool(np.any((abs(x-x0) <= length/2) & (abs(y-y0) <= width/2) & (abs(z-z0) <= height/2)))


def test_building_shell_is_closed_except_the_dock_doors(scene):
    shell, walls = scene.raw['building']['shell'], shell_boxes(scene)
    doors = shell['dock_doors']
    for z in (.1, 1., 2.9, 3.1, 5., 8., 13.9):
        for y in np.arange(-53.97, 54., .25):                       # west and east, full length
            assert covered(walls, -73, y, z) and covered(walls, 73, y, z), (y, z)
        for x in np.arange(-73.27, 73.3, .25):
            assert covered(walls, x, 54, z), (x, z)                   # north
            door = z < doors['height'] and any(abs(x-c) < doors['width']/2 for c in doors['centres_x'])
            assert covered(walls, x, -53.875, z) != door, (x, z)      # dock wall: open exactly at the doors
    o = shell['outer']
    for x, y in ((o['west']+.05, o['north']-.05), (o['east']-.05, o['north']-.05),
                 (o['west']+.05, o['south']+.05), (o['east']-.05, o['south']+.05)):
        assert covered(walls, x, y, 1.) and covered(walls, x, y, 10.), (x, y)   # closed corners


def test_dock_openings_are_the_prototype_doors(scene):
    doors = scene.raw['building']['shell']['dock_doors']
    panels = np.asarray(scene.raw['layers']['architecture']['cube'], dtype=float).reshape(-1, 13)
    for c in doors['centres_x']:
        # Roll-up door panel (8 m x 4.2 m) of the prototype inside every opening of the enclosure.
        door = panels[(abs(panels[:, 0]-c) < 1e-6) & (abs(panels[:, 1]+54) < 1e-6) & (abs(panels[:, 3]-8) < 1e-6)]
        assert len(door) == 1 and door[0, 5] <= doors['height'] and door[0, 3] <= doors['width']


def test_upper_walls_change_no_simulated_band(scene):
    # Highest simulated band: forklift traffic 0.05-2.0 m; scanners at 0.21 and 0.48 m.
    upper = shell_boxes(scene, tuple(f'walls_{s}' for s in SIDES))
    assert (upper[:, 2]-upper[:, 5]/2).min() >= scene.raw['building']['shell']['plinth_height'] > 2.0
    enclosure = shell_boxes(scene, ('enclosure',))
    assert np.allclose(enclosure[:, 2]-enclosure[:, 5]/2, 0) and np.allclose(enclosure[:, 5], 3)
