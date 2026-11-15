"""Fleet tracking ranges (fleet_scan.py): the other AMRs' boxes as seen from a robot's base_link."""
import math
import numpy as np
from amr_lite_sim.fleet_scan import BEAMS, beam_angles, fleet_ranges

L, W, OFF = 0.975, 0.70, 0.0125


def beam(angle):
    return int(round((angle+math.pi)/(2*math.pi/BEAMS))) % BEAMS


def test_robot_behind_is_seen_where_the_scanners_are_blind():
    """Another AMR 1.5 m behind (outside the front scanner's ±90 degrees, below the 360-degree scanner)."""
    r = fleet_ranges((10., 5., 0.), [(8.5, 5., 0.)], L, W, OFF, margin=0.)
    # Rear face of the observer's view: the other box's front edge at x = 8.5 + 0.0125 + 0.4875 = 9.0.
    assert math.isclose(r[beam(math.pi)], 1.0, abs_tol=1e-6)
    assert np.isinf(r[beam(0.)]) and np.isinf(r[beam(math.pi/2)])


def test_no_self_registration_and_inf_clears_free_beams():
    r = fleet_ranges((0., 0., 0.3), [], L, W, OFF)
    assert np.isinf(r).all() and len(r) == BEAMS


def test_rotation_margin_and_range_limit():
    # Observer yawed 90 degrees: a robot 3 m east is at -90 degrees in its frame; margin enlarges the box.
    r0 = fleet_ranges((0., 0., math.pi/2), [(3., 0., math.pi/2)], L, W, OFF, margin=0.)
    r1 = fleet_ranges((0., 0., math.pi/2), [(3., 0., math.pi/2)], L, W, OFF, margin=.1)
    i = beam(-math.pi/2)
    assert math.isclose(r0[i], 3.-W/2, abs_tol=1e-6) and math.isclose(r1[i], 3.-W/2-.1, abs_tol=1e-6)
    assert np.isinf(fleet_ranges((0., 0., 0.), [(40., 0., 0.)], L, W, OFF, range_max=30.)).all()


def test_nearest_robot_wins_and_angles_cover_the_circle():
    r = fleet_ranges((0., 0., 0.), [(5., 0., 0.), (2.5, 0., 0.)], L, W, OFF, margin=0.)
    assert math.isclose(r[beam(0.)], 2.5+OFF-L/2, abs_tol=1e-6)
    a = beam_angles()
    assert math.isclose(a[0], -math.pi) and a[-1] < math.pi
