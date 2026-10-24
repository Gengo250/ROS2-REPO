"""Interlock failure cases; these tests do not stand in for physical acceptance."""
from types import SimpleNamespace
import time
from geometry_msgs.msg import TwistStamped
from amr_navigation.command_guard import CommandGuard

def fixture():
    now=time.monotonic();cmd=TwistStamped();cmd.twist.linear.x=.2
    # TF only needs stamps here; the real integration audit verifies connectivity.
    tf=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)))
    guard=SimpleNamespace(last_clock=10.,clock_changed=now,command=cmd,covariance=[0.]*36,
        received={k:now for k in ['scan','odom','command']},stamps={k:10. for k in ['scan','odom','command']},
        buffer=SimpleNamespace(lookup_transform=lambda *a:tf,clear=lambda:None),
        get_publishers_info_by_topic=lambda _:[],get_name=lambda:'navigation_command_guard')
    return guard,now

def test_missing_sensors_fail_closed():
    for key in ['scan','odom','command']:
        g,t=fixture();g.received.pop(key)
        assert CommandGuard.check(g,t,10.)=='Missing or stale '+key

def test_stale_clock_and_clock_reset():
    g,t=fixture();assert CommandGuard.check(g,t+1.1,10.)=='Simulation clock stopped or reset'
    g,t=fixture();CommandGuard.check(g,t,1.);assert g.command is None and g.covariance is None

def test_uncertain_localization_and_drive_limits():
    g,t=fixture();assert CommandGuard.check(g,t,10.)=='ready'
    g.covariance[35]=2.;assert CommandGuard.check(g,t,10.)=='Localization unavailable or uncertain'
    g,t=fixture();g.command.twist.linear.x=.71;assert CommandGuard.check(g,t,10.)=='Command outside drive limits'
    g,t=fixture();g.command.twist.linear.x=-.1;assert CommandGuard.check(g,t,10.)=='Command outside drive limits'

def test_duplicate_command_owner():
    g,t=fixture();g.get_publishers_info_by_topic=lambda _:[SimpleNamespace(node_name='manual')]
    assert CommandGuard.check(g,t,10.).startswith('Conflicting command publishers')
