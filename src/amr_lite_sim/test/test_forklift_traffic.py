"""Forklift traffic with Nav2 AMRs (engine only): holds with hysteresis, smooth following, right of
way, keep-clear lane intervals at narrow corridor mouths, stationary and turning robots.

Only EMP-1 and amr_01 take part (workers and the other forklifts are removed); the robot is driven by
scripted commands, which stand in for Nav2.
"""
import math
from types import SimpleNamespace
from pathlib import Path
import numpy as np
import pytest
from atlas_scene.scene import Scene
from amr_lite_sim import robot_model
from amr_lite_sim.agents import Traffic, VEHICLE_MARGIN
from amr_lite_sim.geometry import overlap
from amr_lite_sim.world import World, footprint, predicted_samples, rotation_radius

SCENE = Path(__file__).resolve().parents[2]/'atlas_scene/config/atlas_campus.scene.json'
UTURN = 127.          # EMP-1 lane distance where the north loop turns at the charger aisle mouth


@pytest.fixture(scope='module')
def scene():
    return Scene.load(SCENE)


@pytest.fixture(scope='module')
def model():
    return robot_model.load()


def setup(scene, model, pose, emp_d):
    w = World(scene, model, 'operations', robots=['amr_01'])
    w.workers = []
    w.forklifts = [f for f in w.forklifts if f.id == 'EMP-1']
    w.traffic.forklifts = w.forklifts
    f = w.forklifts[0]
    f.d = emp_d
    f.pose = f.pose_at(f.d)
    f.obb = f.footprint(f.pose)
    d = w.robots['amr_01'].drive
    d.x, d.y, d.yaw = pose
    return w, f, d


def gap(a, b):
    if overlap(a, b):
        return 0.
    def seg(p, q0, q1):
        e = q1-q0
        t = np.clip(np.dot(p-q0, e)/np.dot(e, e), 0., 1.)
        return float(np.hypot(*(p-q0-t*e)))
    return min(min(seg(p, q[i], q[(i+1) % len(q)]) for p in pp for i in range(len(q))) for pp, q in ((a, b), (b, a)))


def drive(w, f, d, model, seconds, profile, dt=.02):
    """Run with the robot commanded by ``profile(t)``; returns per-step (t, forklift v, gap, state)."""
    out = []
    for i in range(int(round(seconds/dt))):
        t = i*dt
        w.command('amr_01', *profile(t, d))
        w.step(dt)
        out.append((t, f.v, gap(f.obb, footprint(model, d.x, d.y, d.yaw)), Traffic.state(f)))
    return out


def restarts(trace, near=6.):
    n, moving = 0, trace[0][1] > .06
    for _, v, g, _ in trace:
        if v > .06 and not moving and g < near:
            n += 1
        moving = v > .06
    return n


def hesitating(t, d):
    """Nav2-like hesitation of a waiting robot: short turns and a short creep every 6 s."""
    k = t % 6.
    return (0., .35) if k < 1. else (0., -.35) if 3. < k < 4. else (.08, 0.) if 4.5 < k < 5. else (0., 0.)


def test_stationary_robot_keeps_its_body_and_no_invented_motion(model):
    d = SimpleNamespace(x=1., y=2., yaw=.3, v=0., w=0.)
    samples = predicted_samples(model, d)
    assert len(samples) == 1 and samples[0][0] == 0.
    assert np.allclose(samples[0][2], footprint(model, 1., 2., .3))


def test_robot_turning_in_place_occupies_its_swept_disc(scene, model):
    w, f, d = setup(scene, model, (30., 21.3, 0.), 10.)
    d.w = .35
    body = w.robots['amr_01'].view(model).obb
    r = np.hypot(*(body-[30., 21.3]).T)
    assert np.allclose(r, rotation_radius(model), atol=1e-6) and len(body) == 16


def test_no_stop_and_go_behind_a_hesitating_robot(scene, model):
    """Before: 12 restarts within 60 s (the forklift crept forward on every small robot motion)."""
    w, f, d = setup(scene, model, (30., 21.3, 0.), 63.5+30.-20.)
    trace = drive(w, f, d, model, 60., hesitating)
    assert restarts(trace) == 0
    held = [s for t, v, g, s in trace if t > 15.]
    assert all(s == ('stopped', 'veh') for s in held)                      # one stable state while held
    assert min(g for _, _, g, _ in trace) >= VEHICLE_MARGIN


def test_forklift_resumes_once_the_robot_has_left(scene, model):
    w, f, d = setup(scene, model, (30., 21.3, 0.), 63.5+30.-20.)
    trace = drive(w, f, d, model, 20., lambda t, d: (0., 0.))
    assert trace[-1][3] == ('stopped', 'veh') and f.hold is not None
    # The robot turns south and drives off the lane (into the rack aisle at x=27 is not on its way:
    # it simply leaves the lane southwards in the cross aisle).
    def leave(t, d):
        err = math.atan2(math.sin(-math.pi/2-d.yaw), math.cos(-math.pi/2-d.yaw))
        return (0., max(-.5, min(.5, 2*err))) if abs(err) > .2 else (.5, 2*err)
    trace = drive(w, f, d, model, 25., leave)
    moving = [t for t, v, g, s in trace if v > .06]
    assert moving and f.hold is None and restarts(trace, near=99.) == 1
    assert min(g for _, _, g, _ in trace) >= VEHICLE_MARGIN


def test_following_a_robot_that_slows_down_is_smooth(scene, model):
    """Before: 6 stops and restarts and 408 emergency-braking steps in 60 s."""
    w, f, d = setup(scene, model, (10., 21., 0.), 63.5+10.-12.)
    trace = drive(w, f, d, model, 60., lambda t, d: ((.15 if 6. < t % 10. < 8. else .6), 0.))
    assert restarts(trace) == 0
    decel = [(a[1]-b[1])/.02 for a, b in zip(trace, trace[1:])]           # agent period 0.02 s
    assert max(decel) < .95*f.emerg                                       # never emergency braking
    gaps = [g for t, _, g, _ in trace if t > 15.]
    assert min(gaps) >= VEHICLE_MARGIN and contacts(w) == []


def contacts(w):
    return [e for e in w.drain_events() if e['kind'] == 'contact']


def test_keep_clear_interval_covers_the_charger_aisle_mouth(scene, model):
    w, f, d = setup(scene, model, (0., 0., 0.), 0.)
    assert len(f.zones) == 1
    d_in, d_out = f.zones[0]['d_in'], f.zones[0]['d_out']
    assert 125. < d_in < 127. and 132. < d_out < 134.


def test_parked_robot_at_charger_01_no_longer_traps_the_forklift_in_the_aisle_mouth(scene, model):
    """Reported case. Before: EMP-1 stopped mid-turn (lane 128.8, (63.5, 22.8), 'veh') with its body
    across the charger aisle mouth, so the parked robot could not leave. Now it waits before the turn,
    reports the persistent wait, the robot leaves through the free mouth, and the forklift resumes."""
    w, f, d = setup(scene, model, (63.4, 26.9, 0.), UTURN-14.)
    trace = drive(w, f, d, model, 45., lambda t, d: (0., 0.))
    assert f.d < f.zones[0]['d_in']+.05 and trace[-1][3] == ('stopped', 'yield')
    assert any(e['kind'] == 'vehicle_wait' and e['other'] == 'amr_01' for e in w.events)
    mouth = footprint(model, 63.4, 23.6, -math.pi/2)                       # the robot's way out
    assert not overlap(mouth, f.obb, .3)
    # Leave: south out of the aisle, then west along the cross aisle.
    state = {}

    def leave(t, d):
        if state.get('west') or d.y < 24.6:
            state['west'] = True
            err = math.atan2(math.sin(math.pi-d.yaw), math.cos(math.pi-d.yaw))
        else:
            err = math.atan2(math.sin(-math.pi/2-d.yaw), math.cos(-math.pi/2-d.yaw))
        return (0., max(-.35, min(.35, 2*err))) if abs(err) > .3 else (.6, 2*err)
    trace = drive(w, f, d, model, 40., leave)
    assert d.x < 57. and f.d > f.zones[0]['d_out']                               # both got through
    events = w.drain_events()
    assert any(e['kind'] == 'vehicle_resumed' for e in events) and not any(e['kind'] == 'contact' for e in events)
    assert restarts(trace, near=99.) == 1
    assert min(g for _, _, g, _ in trace) >= VEHICLE_MARGIN


def test_forklift_yields_to_a_crossing_robot_even_after_a_long_wait(scene, model):
    """Right of way does not depend on the forklift's waiting time (the inter-forklift boost)."""
    w, f, d = setup(scene, model, (27., 28., -math.pi/2), 63.5+27.-16.)
    f.boost_t, f.wait_t = 5., 20.
    def cross(t, d):
        probe = footprint(model, d.x+math.cos(d.yaw)*1.2, d.y+math.sin(d.yaw)*1.2, d.yaw)
        return (0., 0.) if overlap(probe, f.obb, .1) else (.6, 0.)
    trace = drive(w, f, d, model, 25., cross)
    assert d.y < 17. and contacts(w) == []                                 # the robot crossed both lanes
    assert any(s == ('stopped', 'yield') for _, _, _, s in trace)
    assert min(g for _, _, g, _ in trace) >= VEHICLE_MARGIN
