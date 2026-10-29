"""Registration, occupancy semantics and footprint traversability."""
import copy
import math
import numpy as np
import pytest
from PIL import Image
from amr_route_planner import coordinate_registration as reg
from amr_route_planner import map_validation as mv

FOOTPRINT = mv.Footprint(front=.54, back=.52, half_width=.39)


def test_se2_forward_inverse_rotation_and_pose():
    t = reg.SE2(11.0375, .065, .0145)
    for p in [(0., 0.), (-11., 0.), (8.1, 8.5), (-8.7, -8.5)]:
        assert math.dist(t.inverse().apply(*t.apply(*p)), p) < 1e-12
    # Hand-computed: R(0.0145)·(-11, 0) + t.
    x, y = t.apply(-11., 0.)
    assert x == pytest.approx(-11*math.cos(.0145)+11.0375) and y == pytest.approx(-11*math.sin(.0145)+.065)
    assert t.apply_pose(0, 0, math.pi)[2] == pytest.approx(reg.wrap(math.pi+.0145))
    composed = t.compose(t.inverse())
    assert abs(composed.x) < 1e-12 and abs(composed.y) < 1e-12 and abs(composed.yaw) < 1e-12


def record(**changes):
    base = {'schema_version': 1, 'parent_frame': 'world', 'child_frame': 'map', 'map_version': 'm', 'layout_sha256': 'l',
            'transform': {'x': 11., 'y': 0., 'yaw': .01}, 'residuals': {'cells': 900, 'median_m': .05, 'inlier_fraction_0_20': .8},
            'prior': {'transform': {'x': 11., 'y': 0., 'yaw': 0.}}}
    base.update(changes)
    return reg.accept(base, {'min_cells': 500, 'max_median_m': .1, 'min_inlier_fraction_0_20': .7,
                             'max_prior_translation_m': .5, 'max_prior_yaw_rad': .05})


def test_registration_is_bound_to_map_layout_and_acceptance():
    good = record()
    assert reg.evaluate(good, 'm', 'l') == reg.SE2(11., 0., .01)
    for args in [('other-map', 'l'), ('m', 'other-layout')]:
        with pytest.raises(reg.RegistrationError):
            reg.evaluate(good, *args)
    for bad in [record(residuals={'cells': 900, 'median_m': .3, 'inlier_fraction_0_20': .8}),
                record(transform={'x': 12., 'y': 0., 'yaw': .01}), record(transform={'x': 11., 'y': 0., 'yaw': .2})]:
        assert not bad['valid']
        with pytest.raises(reg.RegistrationError):
            reg.evaluate(bad, 'm', 'l')
    tampered = copy.deepcopy(good)
    tampered['transform']['x'] = 11.3
    with pytest.raises(reg.RegistrationError, match='identifier'):
        reg.evaluate(tampered, 'm', 'l')
    with pytest.raises(reg.RegistrationError):
        reg.evaluate({**good, 'child_frame': 'odom'}, 'm', 'l')
    with pytest.raises(reg.RegistrationError):
        reg.evaluate(None, 'm', 'l')


def test_fit_recovers_a_known_rotation_and_translation():
    rects = [(-5, 2, 1.2, 2.5), (0, 2, 1.2, 2.5), (5, -2, 1.2, 2.5), (0, 9.9, 30, .25), (-14.9, 0, .25, 20)]
    rng = np.random.default_rng(3)
    world = []
    for cx, cy, sx, sy in rects:
        for _ in range(120):
            side = rng.integers(4)
            u = rng.uniform(-.5, .5)
            world.append((cx+(u*sx if side < 2 else (-.5 if side == 2 else .5)*sx), cy+(u*sy if side >= 2 else (-.5 if side == 0 else .5)*sy)))
    truth = reg.SE2(10.9, .12, .02)
    points = np.array([truth.apply(*p) for p in world])+rng.normal(0, .01, (len(world), 2))
    fitted, _ = reg.fit(points, rects, reg.SE2(11., 0., 0.))
    assert math.hypot(fitted.x-truth.x, fitted.y-truth.y) < .03 and abs(fitted.yaw-truth.yaw) < .003
    assert np.median(reg.residuals(fitted, points, rects)) < .02


def grid(values, resolution=.1):
    return mv.OccupancyMap(np.array(values, dtype=np.int8), resolution, (0., 0., 0.))


def test_trinary_loading_matches_map_server_and_version_tracks_content(tmp_path):
    pixels = np.full((4, 5), 205, dtype=np.uint8)
    pixels[0, 0], pixels[3, 4] = 0, 254
    Image.fromarray(pixels).save(tmp_path/'m.pgm')
    (tmp_path/'m.yaml').write_text('image: m.pgm\nmode: trinary\nresolution: 0.1\norigin: [1, 2, 0]\n'
                                   'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    m = mv.OccupancyMap.from_yaml(tmp_path/'m.yaml')
    # Image row 0 is the top; OccupancyGrid row 0 is the bottom. 205 stays unknown.
    assert m.grid[3, 0] == mv.OCCUPIED and m.grid[0, 4] == mv.FREE and m.grid[1, 1] == mv.UNKNOWN
    version = m.version()
    m.grid[1, 1] = mv.FREE
    assert m.version() != version
    (tmp_path/'r.yaml').write_text('image: m.pgm\nmode: scale\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\n'
                                   'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    with pytest.raises(mv.MapError):
        mv.OccupancyMap.from_yaml(tmp_path/'r.yaml')


def test_unknown_occupied_clearance_and_outside_are_rejected():
    free = np.zeros((60, 100), dtype=np.int8)
    m = grid(free)
    start, end = (2., 3.), (8., 3.)
    assert mv.classify(m, *mv.edge_shapes(start, end, FOOTPRINT, .3))[0] == 'validated'
    unknown = free.copy()
    unknown[30, 50] = mv.UNKNOWN
    assert mv.classify(grid(unknown), *mv.edge_shapes(start, end, FOOTPRINT, .3))[0] == 'unmapped'
    blocked = free.copy()
    blocked[30, 50] = mv.OCCUPIED
    assert mv.classify(grid(blocked), *mv.edge_shapes(start, end, FOOTPRINT, .3))[:2] == \
        ('blocked', 'occupied map cells under the robot footprint')
    # 0.55 m beside the centreline: outside the 0.39 m padded half-width, inside the 0.30 m clearance.
    near = free.copy()
    near[35, 50] = mv.OCCUPIED
    assert mv.classify(grid(near), *mv.edge_shapes(start, end, FOOTPRINT, .3))[1] == \
        'occupied map cells inside the required clearance'
    far = free.copy()
    far[45, 50] = mv.OCCUPIED
    assert mv.classify(grid(far), *mv.edge_shapes(start, end, FOOTPRINT, .3))[0] == 'validated'
    assert mv.classify(m, *mv.edge_shapes((.3, 3.), end, FOOTPRINT, .3))[0] == 'unmapped'
    # A point robot would wrongly pass a 0.6 m gap; the real footprint does not.
    gap = free.copy()
    gap[:27, 50] = gap[33:, 50] = mv.OCCUPIED
    assert mv.classify(grid(gap), *mv.edge_shapes(start, end, FOOTPRINT, 0.))[0] == 'blocked'


def test_footprint_is_read_from_nav2_with_padding():
    params = {'global_costmap': {'global_costmap': {'ros__parameters': {
        'footprint': '[[0.50,0.35],[0.50,-0.35],[-0.48,-0.35],[-0.48,0.35]]', 'footprint_padding': .04}}}}
    f = mv.Footprint.from_nav2(params)
    assert (f.front, f.back, f.half_width) == pytest.approx((.54, .52, .39))
    assert f.circumscribed == pytest.approx(math.hypot(.54, .39))
    box = mv.sweep((0, 0), (0, 4), f, .3)
    assert box.contains(np.array([0.]), np.array([4.8]))[0] and not box.contains(np.array([0.]), np.array([4.9]))[0]
    assert mv.shape_hits_rect(mv.Disc(0, 0, 1.05), (1.5, 0, 1., 1.)) and not mv.shape_hits_rect(mv.Disc(0, 0, .95), (1.5, 0, 1., 1.))


def test_exclusion_removes_only_the_robot_footprint_cells():
    values = np.zeros((40, 40), dtype=np.int8)
    values[20, 18] = mv.UNKNOWN   # under the robot
    values[20, 30] = mv.UNKNOWN   # ahead of the robot
    m = grid(values)
    robot = mv.Box(1.85, 2.05, 0., .4, .3)
    assert m.region_counts([robot])['unknown'] == 1
    assert m.region_counts([robot], exclude=[robot])['unknown'] == 0
    ahead = mv.sweep((1.85, 2.05), (3.05, 2.05), FOOTPRINT)
    assert m.region_counts([ahead], exclude=[robot])['unknown'] == 1


def test_live_occupancy_grid_and_yaml_have_the_same_version():
    """map_server publishes float32 resolution; the content version must not depend on it."""
    import array
    from nav_msgs.msg import OccupancyGrid
    grid = np.array([[0, 100], [-1, 0]], dtype=np.int8)
    m = OccupancyGrid()
    m.info.resolution, m.info.width, m.info.height = .05, 2, 2
    m.info.origin.position.x, m.info.origin.position.y = -10.196, -10.052
    m.info.origin.orientation.w = 1.
    m.data = array.array('b', grid.flatten().tolist())
    from rclpy.serialization import deserialize_message, serialize_message
    m = deserialize_message(serialize_message(m), OccupancyGrid)  # the wire is float32
    assert m.info.resolution != .05
    assert mv.OccupancyMap.from_message(m).version() == mv.OccupancyMap(grid, .05, (-10.196, -10.052, 0)).version()
    assert mv.OccupancyMap(grid, .05, (-10.196, -10.052, -0.)).version() == mv.OccupancyMap(grid, .05, (-10.196, -10.052, 0.)).version()
