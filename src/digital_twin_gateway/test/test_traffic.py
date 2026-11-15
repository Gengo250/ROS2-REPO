"""Fleet coordinator (world/traffic.py): station reservations, narrow corridors, persistent conflicts.

Pure, deterministic decision logic: poses, missions, plans and simulation time are given explicitly.
"""
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace
from digital_twin_gateway.world.models import Mission
from digital_twin_gateway.world.traffic import Corridor, Traffic, footprint_gap

T0 = datetime(2026, 11, 15, tzinfo=timezone.utc)
STATIONS = {'charger_01': (63.4, 27., 0.), 'dock_03': (-24., -49.5, -1.57), 'pick_north_e1': (39., 36., 1.57)}
AISLE = Corridor('charger_aisle', 61.8, 65.0, 22., 46., 1)
WIDE = Corridor('north_cross_aisle', -69., 69., 19., 26., 3)


def amr(robot, x, y, yaw=0., reason=None):
    return SimpleNamespace(id=robot, type='amr', x=x, y=y, yaw=yaw, reason=reason)


def mission(robot, destination, status='active', order=0, remaining=None):
    return Mission(id=f'm_{robot}_{destination}', robot_id=robot, destination=destination, strategy='direct', status=status,
                   detail=None, goal=(0., 0., 0.), distance_remaining_m=remaining, recoveries=0,
                   created_at=T0+timedelta(seconds=order), updated_at=T0)


def amrs(*robots):
    return {r.id: r for r in robots}


def test_shared_destination_one_owner_fifo_queue():
    t = Traffic(STATIONS)
    world = amrs(amr('amr_01', 2, 0), amr('amr_02', -5, 0), amr('amr_03', 9, 0))
    assert t.request('amr_01', 'charger_01', world) == ('dispatch', None)
    d2, w2 = t.request('amr_02', 'charger_01', world)
    d3, w3 = t.request('amr_03', 'charger_01', world)
    assert (d2, w2.kind, w2.on, w2.position, w2.blocker) == ('queue', 'station', 'charger_01', 1, 'amr_01')
    assert (d3, w3.position) == ('queue', 2)
    stations, _ = t.describe(list(world.values()))
    assert stations == [dict(id='charger_01', owner='amr_01', occupants=(), queue=('amr_02', 'amr_03'))]


def test_station_occupied_until_the_robot_physically_leaves():
    t = Traffic(STATIONS)
    t.request('amr_01', 'charger_01', {})
    missions = {'amr_01': mission('amr_01', 'charger_01', order=0), 'amr_02': mission('amr_02', 'charger_01', 'waiting', 1)}
    t.request('amr_02', 'charger_01', {})
    # amr_01 arrived and its mission succeeded: ownership ends, but it is still parked on the station.
    missions['amr_01'] = mission('amr_01', 'charger_01', 'succeeded')
    parked = [amr('amr_01', 63.4, 27.), amr('amr_02', 40, 22)]
    for sim in (1., 2., 30.):
        actions, waiting = t.step(sim, parked, missions, {})
        assert actions == [] and waiting['amr_02'].blocker == 'amr_01'
    # Still inside the occupancy disc (1.4 m from the pose): occupied.
    assert t.step(31., [amr('amr_01', 63.4, 28.4), amr('amr_02', 40, 22)], missions, {})[0] == []
    # Outside: the queue head goes, and becomes the owner.
    actions, _ = t.step(32., [amr('amr_01', 63.4, 28.6), amr('amr_02', 40, 22)], missions, {})
    assert actions == [('dispatch', 'amr_02')] and t.owners == {'charger_01': 'amr_02'}


def test_parked_robot_without_mission_blocks_the_station():
    t = Traffic(STATIONS)
    world = amrs(amr('amr_01', 63.5, 27.2), amr('amr_02', 40, 22))
    decision, info = t.request('amr_02', 'charger_01', world)
    assert decision == 'queue' and info.blocker == 'amr_01'


def test_cancel_and_replace_update_the_queue():
    t = Traffic(STATIONS)
    t.request('amr_01', 'charger_01', {})
    t.request('amr_02', 'charger_01', {})
    t.request('amr_03', 'charger_01', {})
    t.withdraw('amr_02')                                  # queued mission cancelled
    assert t.queues['charger_01'] == ['amr_03']
    assert t.request('amr_03', 'dock_03', {}) == ('dispatch', None)   # replaced by another destination
    assert t.queues['charger_01'] == [] and t.owners == {'charger_01': 'amr_01', 'dock_03': 'amr_03'}
    t.mission_ended('amr_01')
    assert 'charger_01' not in t.owners


def test_queue_is_bounded():
    t = Traffic(STATIONS, queue_limit=1)
    t.request('amr_01', 'charger_01', {})
    assert t.request('amr_02', 'charger_01', {})[0] == 'queue'
    decision, reason = t.request('amr_03', 'charger_01', {})
    assert decision == 'reject' and 'queue full' in reason


def test_station_swap_cycle_is_broken_by_priority():
    """amr_01 parked on charger_01 wants pick_north_e1, where amr_03 is parked and wants charger_01."""
    t = Traffic(STATIONS)
    world = [amr('amr_01', 63.4, 27.), amr('amr_03', 39., 36.)]
    assert t.request('amr_01', 'pick_north_e1', amrs(*world))[0] == 'queue'
    assert t.request('amr_03', 'charger_01', amrs(*world))[0] == 'queue'
    missions = {'amr_01': mission('amr_01', 'pick_north_e1', 'waiting', 0), 'amr_03': mission('amr_03', 'charger_01', 'waiting', 1)}
    actions, _ = t.step(1., world, missions, {})
    assert actions == [('dispatch', 'amr_01')]            # older mission first; amr_03 still waits
    missions['amr_01'] = mission('amr_01', 'pick_north_e1', 'active', 0)
    assert t.step(2., world, missions, {})[0] == []      # amr_01 not out of charger_01 yet
    actions, _ = t.step(3., [amr('amr_01', 61.0, 25.0), amr('amr_03', 39., 36.)], missions, {})
    assert actions == [('dispatch', 'amr_03')]


def test_corridor_hold_before_entrance_and_resume_when_clear():
    t = Traffic(STATIONS, [AISLE, WIDE])
    missions = {'amr_01': mission('amr_01', 'pick_north_e1', order=0), 'amr_02': mission('amr_02', 'charger_01', order=1)}
    plan_in = (0.5, [(58., 22.5), (60., 22.5), (62.5, 22.5), (63.4, 24.), (63.4, 27.)])
    # amr_01 drives out of the charger aisle; amr_02's plan enters it 4 m ahead: held before the entrance.
    world = [amr('amr_01', 63.4, 25.), amr('amr_02', 58., 22.5)]
    actions, _ = t.step(1., world, missions, {'amr_01': (0.5, [(63.4, 25.), (63.4, 21.)]), 'amr_02': plan_in})
    assert actions[0][0] == 'hold' and actions[0][1] == 'amr_02' and actions[0][2].on == 'charger_aisle' and actions[0][2].blocker == 'amr_01'
    missions['amr_02'] = mission('amr_02', 'charger_01', 'waiting', 1)
    assert t.step(2., world, missions, {'amr_02': plan_in})[0] == []
    # amr_01 has left the aisle: amr_02 claims it and resumes.
    world = [amr('amr_01', 63.0, 20.5), amr('amr_02', 58.3, 22.5)]
    actions, _ = t.step(4., world, missions, {'amr_02': plan_in})
    assert actions == [('resume', 'amr_02')] and t.claims == {'charger_aisle': 'amr_02'}


def test_parked_robot_in_corridor_holds_nobody():
    t = Traffic(STATIONS, [AISLE])
    missions = {'amr_01': mission('amr_01', 'charger_01', 'succeeded'), 'amr_02': mission('amr_02', 'pick_north_e1', order=1)}
    world = [amr('amr_01', 63.4, 27.), amr('amr_02', 58., 22.5)]
    actions, _ = t.step(1., world, missions, {'amr_02': (0.5, [(58., 22.5), (62.5, 22.5), (63.4, 25.)])})
    assert actions == [] and t.claims == {'charger_aisle': 'amr_02'}


def test_simultaneous_corridor_approach_priority_claims():
    t = Traffic(STATIONS, [AISLE])
    missions = {'amr_01': mission('amr_01', 'x', order=0), 'amr_03': mission('amr_03', 'y', order=1)}
    world = [amr('amr_01', 63.4, 19.), amr('amr_03', 63.4, 49.)]
    plans = {'amr_01': (0.5, [(63.4, 19.), (63.4, 23.)]), 'amr_03': (0.5, [(63.4, 49.), (63.4, 45.)])}
    actions, _ = t.step(1., world, missions, plans)
    assert t.claims == {'charger_aisle': 'amr_01'} and actions == [('hold', 'amr_03', actions[0][2])]
    assert actions[0][2].blocker == 'amr_01'


def test_head_on_stall_lower_priority_yields_after_stall_time_only():
    t = Traffic(STATIONS, stall_seconds=12.)
    missions = {'amr_01': mission('amr_01', 'a', order=0, remaining=20.), 'amr_03': mission('amr_03', 'b', order=1, remaining=20.)}
    world = [amr('amr_01', 0., 22.5, 0.), amr('amr_03', 1.6, 22.5, 3.14159)]
    assert footprint_gap(world[0], world[1]) < 1.
    for sim in (0., 5., 11.):                             # a brief stop is not a deadlock
        assert t.step(sim, world, missions, {})[0] == []
    actions, _ = t.step(12.5, world, missions, {})
    assert [(a[0], a[1], a[2].kind, a[2].on) for a in actions] == [('hold', 'amr_03', 'robot', 'amr_01')]
    missions['amr_03'] = mission('amr_03', 'b', 'waiting', 1)
    # amr_01 passes: once it is more than 2 m away, amr_03 resumes.
    actions, _ = t.step(20., [amr('amr_01', 4.5, 21.0, 0.), amr('amr_03', 1.6, 22.5, 3.14159)], missions, {})
    assert actions == [('resume', 'amr_03')]


def test_stall_detection_runs_on_simulation_time():
    """Same simulation history, any wall-clock pace (RTF): same decision."""
    world = [amr('amr_01', 0., 22.5, 0.), amr('amr_03', 1.6, 22.5, 3.14159)]
    decisions = []
    for samples in ((0., 6., 12.5), (0., 1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 12.5)):
        t = Traffic(STATIONS)
        missions = {'amr_01': mission('amr_01', 'a', order=0), 'amr_03': mission('amr_03', 'b', order=1)}
        decisions.append([a[:2] for sim in samples for a in t.step(sim, world, missions, {})[0]])
    assert decisions[0] == decisions[1] == [('hold', 'amr_03')]


def test_yield_swap_then_block_when_no_safe_resolution():
    t = Traffic(STATIONS, stall_seconds=10., max_yields=2)
    missions = {'amr_01': mission('amr_01', 'a', order=0), 'amr_03': mission('amr_03', 'b', order=1)}
    world = [amr('amr_01', 0., 22.5, 0.), amr('amr_03', 1.2, 22.5, 3.14159)]
    t.step(0., world, missions, {})
    assert [a[:2] for a in t.step(10.5, world, missions, {})[0]] == [('hold', 'amr_03')]
    missions['amr_03'] = mission('amr_03', 'b', 'waiting', 1)
    # amr_01 cannot pass the yielded amr_03: roles swapped once.
    t.step(11., world, missions, {})
    actions = t.step(21., world, missions, {})[0]
    assert [a[:2] for a in actions] == [('resume', 'amr_03'), ('hold', 'amr_01')]
    missions['amr_03'] = mission('amr_03', 'b', 'active', 1)
    missions['amr_01'] = mission('amr_01', 'a', 'waiting', 0)
    t.step(21.5, world, missions, {})
    # Still no way through: both stopped and reported, no further attempts.
    actions = t.step(32., world, missions, {})[0]
    assert sorted(a[:2] for a in actions) == [('block', 'amr_01'), ('block', 'amr_03')]


def test_contact_counts_as_conflict_and_parked_blocker_leads_to_block():
    t = Traffic(STATIONS, stall_seconds=12., block_seconds=30.)
    missions = {'amr_01': mission('amr_01', 'a', order=0), 'amr_02': mission('amr_02', 'b', 'succeeded')}
    world = [amr('amr_01', 0., 0., 0., reason='contact:amr_02'), amr('amr_02', 3.0, 0., 0.)]
    assert t._blocker(world[0], amrs(*world)) == 'amr_02'
    t.step(0., world, missions, {})
    assert t.step(20., world, missions, {})[0] == []      # behind a parked robot: wait for Nav2 first
    actions = t.step(30.5, world, missions, {})[0]
    assert [(a[0], a[1], a[2].on) for a in actions] == [('block', 'amr_01', 'amr_02')]


def test_progress_resets_stall_timer():
    t = Traffic(STATIONS, stall_seconds=12.)
    missions = {'amr_01': mission('amr_01', 'a', order=0, remaining=10.), 'amr_03': mission('amr_03', 'b', order=1, remaining=10.)}
    near = lambda x: [amr('amr_01', x, 22.5), amr('amr_03', x+1.4, 22.5, 3.14)]
    t.step(0., near(0.), missions, {})
    t.step(8., near(0.5), missions, {})                    # moved 0.5 m: progress
    assert t.step(15., near(0.5), missions, {})[0] == []  # only 7 s since the last progress
