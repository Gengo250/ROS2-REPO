import math
from pathlib import Path
import numpy as np
import pytest
from atlas_scene.scene import Scene
from amr_lite_sim import robot_model
from amr_lite_sim.geometry import polygon_segments
from amr_lite_sim.lidar import SegmentSet, scan
from amr_lite_sim.agents import Worker
from amr_lite_sim.world import World, footprint

SCENE = Path(__file__).resolve().parents[2]/'atlas_scene/config/atlas_campus.scene.json'


@pytest.fixture(scope='module')
def scene():
    return Scene.load(SCENE)


@pytest.fixture(scope='module')
def model():
    return robot_model.load()


def run(world, seconds, v=0., w=0., robot='amr_01'):
    for _ in range(int(round(seconds/0.01))):
        world.command(robot, v, w)
        world.step(0.01)


def test_drive_limits_and_command_timeout(scene, model):
    world = World(scene, model, 'empty')
    r = world.robots['amr_01']
    run(world, 0.5, v=5., w=0.)            # far above the 0.7 m/s limit
    assert r.drive.v == pytest.approx(0.25, abs=1e-6)   # 0.5 m/s² for 0.5 s
    run(world, 3., v=5.)
    assert r.drive.v == pytest.approx(0.7)
    for _ in range(100):                   # no command for 1 s: 0.5 s timeout, then braking
        world.step(0.01)
    assert r.drive.v < 0.7-0.2 and r.drive.command == (5., 0.)
    run(world, 2., w=9.)
    assert r.drive.w == pytest.approx(1.5)


def test_odometry_starts_at_spawn_and_integrates_exactly(scene, model):
    world = World(scene, model, 'empty')
    sx, sy, syaw = world.robots['amr_01'].spawn
    run(world, 4., v=.5)                   # spawn faces north (+y) along the open hall
    x, y, yaw = world.odometry('amr_01')
    r = world.robots['amr_01'].drive
    assert abs(y) < 1e-9 and abs(yaw) < 1e-9 and x == pytest.approx(math.hypot(r.x-sx, r.y-sy))
    assert r.left_angle == pytest.approx(r.right_angle)


def test_collision_blocks_motion_and_emits_one_event(scene, model):
    world = World(scene, model, 'empty')
    robot = world.robots['amr_01']
    obstacle, reason = world.add_obstacle('pallet', robot.drive.x, robot.drive.y+2.0)
    assert obstacle and not reason
    run(world, 8., v=.4)
    assert robot.state == 'blocked' and robot.contact_with == obstacle.id
    poly = footprint(model, robot.drive.x, robot.drive.y, robot.drive.yaw)
    assert poly[:, 1].max() < obstacle.polygon[:, 1].min()+1e-6
    kinds = [e['kind'] for e in world.drain_events()]
    assert kinds.count('contact') == 1 and 'protective_field' in kinds
    run(world, 1., v=-.3)                 # reversing away clears the contact
    assert robot.state == 'moving' and 'released' in [e['kind'] for e in world.drain_events()]


def test_obstacle_placement_is_validated(scene, model):
    world = World(scene, model, 'empty')
    assert world.add_obstacle('piano', 0, 0)[0] is None
    assert world.add_obstacle('box', 500, 0)[1] == 'outside the building'
    assert world.add_obstacle('box', -52, 36)[1] == 'occupied by a permanent structure'
    rx, ry = world.robots['amr_01'].drive.x, world.robots['amr_01'].drive.y
    assert 'robot' in world.add_obstacle('cone', rx, ry)[1]
    first, _ = world.add_obstacle('cone', 2, 8)
    assert world.add_obstacle('box', 2.1, 8)[1] == 'another object is already there'
    assert world.remove_obstacle(first.id) == 1 and world.remove_obstacle('missing') == 0


def test_lidar_matches_brute_force_with_dynamic_objects(scene, model):
    world = World(scene, model, 'operations')
    world.add_obstacle('pallet', 2, 3.5)
    spec = model.front
    ranges = world.scan('amr_01', spec)
    x, y, yaw = world.sensor_pose(world.robots['amr_01'], spec)
    a, b = polygon_segments(scene.slice_polygons(spec.mount_z))
    da, db = world.dynamic_segments(world.robots['amr_01'], spec)
    a, b = np.vstack([a, da]), np.vstack([b, db])
    e = b-a
    for k in range(0, spec.samples, 7):
        th = yaw+spec.angle_min+k*spec.increment
        d = np.array([math.cos(th), math.sin(th)])
        q = a-[x, y]
        den = d[0]*e[:, 1]-d[1]*e[:, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (q[:, 0]*e[:, 1]-q[:, 1]*e[:, 0])/den
            u = (q[:, 0]*d[1]-q[:, 1]*d[0])/den
        hits = t[(t > 0) & (u >= -1e-9) & (u <= 1+1e-9)]
        expected = hits.min() if len(hits) and hits.min() <= spec.range_max else math.inf
        assert (math.isinf(expected) and math.isinf(ranges[k])) or abs(ranges[k]-expected) < 1e-4
    # The pallet 3.5 m ahead (1.02 m deep, front face 2.99 m away) is seen by the centre beam.
    assert ranges[spec.samples//2] == pytest.approx(3.5-.51-spec.mount_x, abs=.02)


def test_workers_never_enter_structure_and_world_is_deterministic(scene, model):
    def trajectory():
        world = World(scene, model, 'operations', seed=3)
        out = []
        for i in range(1500):
            world.step(0.02)
            if i % 50 == 0:
                for w in world.workers:
                    if w.on_floor:
                        assert not world.human_static.circle_collides(w.x, w.y, .2), (w.id, w.x, w.y)
                out.append([(round(w.x, 6), round(w.y, 6)) for w in world.workers]+[round(f.d, 6) for f in world.forklifts])
        return out
    first = trajectory()
    assert first == trajectory()
    moved = sum(1 for a, b in zip(first[0][:16], first[-1][:16]) if math.dist(a, b) > 1)
    assert moved >= 8


def test_in_place_rotation_refused_with_person_in_swept_area(scene, model):
    world = World(scene, model, 'operations')
    robot = world.robots['amr_01']
    worker = world.workers[0]
    worker.x, worker.y = robot.drive.x+.75, robot.drive.y      # beside the chassis, inside the swept disc
    worker.vmax = 0.
    run(world, 1., w=1.)
    assert abs(robot.drive.yaw-math.pi/2) < 1e-9 and robot.state == 'stopped' and robot.reason.startswith('rotation_stop')
    assert [e['kind'] for e in world.drain_events()].count('protective_stop') == 1
    worker.x, worker.y = robot.drive.x+3., robot.drive.y+3.     # person leaves: the turn proceeds
    run(world, .5, w=1.)
    assert robot.drive.yaw > math.pi/2+.01


def meet_parked_robot(scene, model, robot_x):
    """COL-01 walks the 2.3 m aisle x = -55 (26 <-> 40); an AMR is parked in it at y = 31."""
    world = World(scene, model, 'operations')
    worker = next(w for w in world.workers if w.id == 'COL-01')
    robot = world.robots['amr_01']
    robot.drive.x, robot.drive.y, robot.drive.yaw = robot_x, 31., -math.pi/2
    events, waited, longest = [], 0., 0.
    for _ in range(2400):                                         # 24 s
        world.step(.01)
        events += world.drain_events()
        waited = waited+.01 if worker.waiting else 0.
        longest = max(longest, waited)
    return world, worker, robot, events, longest


def test_worker_steps_aside_and_passes_a_parked_robot(scene, model):
    _, worker, _, events, longest = meet_parked_robot(scene, model, -55.)
    assert not any(e['kind'] == 'contact' for e in events)
    assert longest < 1. and worker.y > 33.                        # stepped aside and walked past


def test_worker_turns_back_when_a_parked_robot_blocks_its_side(scene, model):
    _, worker, robot, events, longest = meet_parked_robot(scene, model, -54.3)   # leaves no room on the right
    assert any(e['kind'] == 'turned_back' and e['entity'] == 'COL-01' for e in events)
    assert not any(e['kind'] == 'contact' for e in events)
    assert longest < worker.STUCK_LIMIT+.5                       # bounded wait (+ turning round): no deadlock
    assert math.dist((worker.x, worker.y), (robot.drive.x, robot.drive.y)) > 2.


def test_safe_stop_for_a_person_in_the_protective_field(scene, model):
    world = World(scene, model, 'operations')
    robot = world.robots['amr_01']
    worker = world.workers[0]
    worker.x, worker.y = robot.drive.x, robot.drive.y+2.5            # standing on the path ahead
    worker.vmax, worker.act = 0., None
    run(world, 6., v=.7)
    kinds = [e['kind'] for e in world.drain_events()]
    assert 'contact' not in kinds and kinds.count('protective_stop') == 1
    assert robot.drive.v == 0. and robot.reason == f'protective_stop:{worker.id}'
    gap = worker.y-Worker.RADIUS-footprint(model, robot.drive.x, robot.drive.y, robot.drive.yaw)[:, 1].max()
    assert gap > .05
    run(world, 1., v=-.2)                                            # reversing stays possible
    assert robot.drive.v < 0.
    worker.x = robot.drive.x+6.                                       # person leaves: restart
    run(world, 2., v=.5)
    assert robot.drive.v > .3 and robot.reason == ''


def test_person_pinned_against_a_robot_can_step_away(scene, model):
    world = World(scene, model, 'operations')
    worker = next(w for w in world.workers if w.id == 'COL-01')     # heads south from (-55, 27.8)
    robot = world.robots['amr_01']
    robot.drive.x, robot.drive.yaw = worker.x, -math.pi/2
    front = footprint(model, 0., 0., -math.pi/2)[:, 1].min()          # nose offset when facing south
    robot.drive.y = worker.y+Worker.RADIUS+.02-front                  # 2 cm gap, inside the worker's margin
    start = worker.y
    for _ in range(300):
        world.step(.01)
    assert start-worker.y > .3
