import math
import pytest
from amr_lite_sim.node import stamp


@pytest.mark.parametrize('seconds', [0., 0.01, 12.9999999997, 13.0000000002, 99.995, 1234.5])
def test_stamp_is_exact_and_normalised(seconds):
    t = stamp(seconds)
    assert 0 <= t.nanosec < 1_000_000_000
    assert abs(t.sec+t.nanosec/1e9-seconds) < 1e-9


def test_accumulated_steps_never_go_backwards():
    # Float accumulation of 0.01 lands just below integers (e.g. 12.9999999997); the
    # message must still be 13 s, not 12 s, so ROS clocks never jump back.
    elapsed, previous = 0, -1
    for _ in range(100000):
        elapsed += 10_000_000
        t = stamp(elapsed/1e9)
        value = t.sec*1_000_000_000+t.nanosec
        assert value > previous
        previous = value
    assert math.isclose(previous/1e9, 1000.)
