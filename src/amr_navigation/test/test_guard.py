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
        received={k:now for k in ['scan','top_scan','odom','command']},stamps={k:10. for k in ['scan','top_scan','odom','command']},
        buffer=SimpleNamespace(lookup_transform=lambda *a:tf,clear=lambda:None),
        get_publishers_info_by_topic=lambda _:[],get_name=lambda:'navigation_command_guard')
    return guard,now

def test_missing_sensors_fail_closed():
    for key in ['scan','top_scan','odom','command']:
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


def test_clock_failure_latches_zero_and_requests_simulation_pause(monkeypatch):
    import rclpy
    monkeypatch.setenv("GZ_PARTITION", "navigation_unit_test")
    rclpy.init()
    guard=CommandGuard()
    messages=[];pauses=[]
    try:
        guard.publisher=SimpleNamespace(publish=messages.append)
        guard.emergency_pause=lambda wall:pauses.append(wall)
        guard.command=TwistStamped();guard.command.twist.linear.x=.4
        guard.check=lambda wall,sim:'ready'
        guard.tick()
        assert messages[-1].twist.linear.x==.4
        guard.check=lambda wall,sim:'Simulation clock stopped or reset'
        guard.tick()
        assert guard.emergency_latched and pauses
        assert messages[-1].twist.linear.x==0.
        # Sensor recovery must not silently restart motion after emergency pause.
        guard.check=lambda wall,sim:'ready'
        guard.tick()
        assert messages[-1].twist.linear.x==0.
    finally:
        guard.destroy_node();rclpy.shutdown()


def test_gazebo_timeout_exit_zero_is_not_a_pause_acknowledgement():
    errors=[]
    guard=SimpleNamespace(pause_acknowledged=False,
        pause_process=SimpleNamespace(poll=lambda:0,communicate=lambda:('Service call timed out\n',None)),
        next_pause_retry=100.,get_logger=lambda:SimpleNamespace(error=errors.append))
    CommandGuard.emergency_pause(guard,10.)
    assert not guard.pause_acknowledged and guard.pause_process is None and errors
    guard.pause_process=SimpleNamespace(poll=lambda:0,communicate=lambda:('data: true\n',None))
    CommandGuard.emergency_pause(guard,11.)
    assert guard.pause_acknowledged


def test_wrong_frame_or_empty_360_scan_is_not_fresh():
    from sensor_msgs.msg import LaserScan
    for frame,ranges in [('laser_frame',[1.]),('top_laser_frame',[float('inf')])]:
        g=SimpleNamespace(received={},stamps={})
        m=LaserScan();m.header.frame_id=frame;m.range_min,m.range_max=.12,20.;m.ranges=ranges
        CommandGuard.sensor(g,'top_scan',m)
        assert 'top_scan' not in g.received
    g=SimpleNamespace(received={},stamps={})
    m=LaserScan();m.header.frame_id='top_laser_frame';m.range_min,m.range_max=.12,20.;m.ranges=[3.]
    CommandGuard.sensor(g,'top_scan',m)
    assert 'top_scan' in g.received

def test_audit_detects_blind_sectors():
    from amr_navigation.audit import blind_arc
    # Zink renderer fault: the left 45-90 deg of the front scanner never returns.
    front=[True]*271+[False]*90
    assert blind_arc(front,False)==90
    # A 360 deg scan's blind arc may span the wrap-around (rear door).
    ring=[False]*20+[True]*680+[False]*20
    assert blind_arc(ring,True)==40 and blind_arc(ring,False)==20
    assert blind_arc([False]*8,True)==8
