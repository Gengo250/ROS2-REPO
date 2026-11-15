from pathlib import Path
import math
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
from amr_navigation.command_guard import command_is_finite
from amr_navigation.goal_client import free_goal
ROOT=Path(__file__).resolve().parents[1]

def test_goal_rejects_unknown_occupied_edge_and_nonfinite():
    m=OccupancyGrid();m.info.width=100;m.info.height=100;m.info.resolution=.1;m.data=[0]*10000
    assert free_goal(m,5,5)
    for value in [-1,100]:
        m.data[50*100+55]=value
        assert not free_goal(m,5,5)
    assert not free_goal(m,.1,.1)
    assert not free_goal(m,math.nan,5)
    assert not free_goal(m,-.01,5)

def test_nonfinite_commands_rejected():
    m=TwistStamped();assert command_is_finite(m)
    m.twist.linear.x=math.inf;assert not command_is_finite(m)
    m.twist.linear.x=0.;m.twist.angular.z=math.nan;assert not command_is_finite(m)

def test_footprint_includes_camera_and_chassis_and_limits():
    p=yaml.safe_load((ROOT/'config/nav2_params.yaml').read_text())
    local=p['local_costmap']['local_costmap']['ros__parameters'];global_=p['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint']==global_['footprint']
    footprint=yaml.safe_load(local['footprint'])
    assert max(v[0] for v in footprint)>=.494
    assert min(v[0] for v in footprint)<=-.475
    assert max(v[1] for v in footprint)>=.35 and min(v[1] for v in footprint)<=-.35
    control=p['controller_server']['ros__parameters'];behavior=p['behavior_server']['ros__parameters']
    assert control['enable_stamped_cmd_vel'] and behavior['enable_stamped_cmd_vel']
    assert control['FollowPath']['desired_linear_vel']<=.7
    assert control['FollowPath']['max_angular_accel']<=1.5
    assert not control['FollowPath']['allow_reversing']
    assert behavior['behavior_plugins']==['spin','wait']
    for tree in ['navigate_to_pose.xml','navigate_through_poses.xml']:
        assert '<BackUp ' not in (ROOT/'config'/tree).read_text()

def test_through_poses_navigator_shares_the_bounded_tree():
    p=yaml.safe_load((ROOT/'config/nav2_params.yaml').read_text())['bt_navigator']['ros__parameters']
    assert p['navigators']==['navigate_to_pose','navigate_through_poses']
    assert p['navigate_through_poses']['plugin']=='nav2_bt_navigator::NavigateThroughPosesNavigator'
    tree=(ROOT/'config/navigate_through_poses.xml').read_text()
    assert 'number_of_retries="3" name="NavigateRecovery"' in tree and 'RemovePassedGoals' in tree
    assert "default_nav_through_poses_bt_xml" in (ROOT/'launch/navigation.launch.py').read_text()

def test_each_scanner_feeds_its_own_obstacle_layer():
    p=yaml.safe_load((ROOT/'config/nav2_params.yaml').read_text())
    for name in ['local_costmap','global_costmap']:
        c=p[name][name]['ros__parameters']
        assert c['plugins']==['static_layer','obstacle_layer','top_obstacle_layer','inflation_layer']
        assert c['obstacle_layer']['observation_sources']=='scan' and c['obstacle_layer']['scan']['topic']=='/scan'
        top=c['top_obstacle_layer']
        assert top['observation_sources']=='top_scan' and top['top_scan']['topic']=='/top_scan'
        assert top['top_scan']['observation_persistence']==0.0 and top['top_scan']['marking'] and top['top_scan']['clearing']


def test_robot_namespace_rewrites_only_robot_frames_and_topics():
    """amr_02's stack: its frames and topics, the shared map frame; the root robot unchanged."""
    from amr_navigation.launch_support import rewrite_for_robot, robot_params
    original=ROOT/'config/nav2_params.yaml'
    params=yaml.safe_load(original.read_text())
    assert rewrite_for_robot(params,'','')==params and robot_params(original)==str(original)
    nested=rewrite_for_robot(params,'amr_02','amr_02/')
    assert list(nested)==['amr_02']
    p=nested['amr_02']
    local=p['local_costmap']['local_costmap']['ros__parameters'];global_=p['global_costmap']['global_costmap']['ros__parameters']
    assert local['global_frame']=='amr_02/odom' and local['robot_base_frame']=='amr_02/base_link'
    assert global_['global_frame']=='map' and global_['robot_base_frame']=='amr_02/base_link'
    assert local['obstacle_layer']['scan']['topic']=='/amr_02/scan' and global_['top_obstacle_layer']['top_scan']['topic']=='/amr_02/top_scan'
    assert p['bt_navigator']['ros__parameters']['odom_topic']=='/amr_02/odom'
    b=p['behavior_server']['ros__parameters'];assert (b['local_frame'],b['global_frame'],b['robot_base_frame'])==('amr_02/odom','map','amr_02/base_link')
    # Every other value (limits, plugins, speeds) is the validated file's.
    assert p['controller_server']==params['controller_server']
    amcl=rewrite_for_robot(yaml.safe_load((ROOT/'config/amcl_params.yaml').read_text()),'amr_03','amr_03/')['amr_03']['amcl']['ros__parameters']
    assert (amcl['base_frame_id'],amcl['odom_frame_id'],amcl['global_frame_id'],amcl['scan_topic'])==('amr_03/base_link','amr_03/odom','map','/amr_03/scan')
    written=yaml.safe_load(open(robot_params(original,'amr_02','amr_02/')).read())
    assert written==nested


def test_fleet_layer_is_added_only_on_request_and_follows_the_namespace():
    """Multi-robot stacks get the fleet tracking layer (other AMRs) in both costmaps, on the robot's
    own topic; the validated parameter file is not edited and single-robot stacks are unchanged."""
    from amr_navigation.launch_support import robot_params, with_fleet_layer
    original=ROOT/'config/nav2_params.yaml'
    params=yaml.safe_load(original.read_text())
    assert robot_params(original)==str(original) and 'fleet_layer' not in str(params)
    fleet=with_fleet_layer(params)
    for name,reach in (('local_costmap',8.),('global_costmap',25.)):
        c=fleet[name][name]['ros__parameters']
        assert c['plugins']==['static_layer','obstacle_layer','top_obstacle_layer','fleet_layer','inflation_layer']
        f=c['fleet_layer']['fleet']
        assert f['topic']=='/fleet/robots' and f['marking'] and f['clearing'] and f['inf_is_valid'] and f['observation_persistence']==0.0
        assert f['obstacle_max_range']==reach and f['raytrace_max_range']>reach and f['expected_update_rate']>0
        # The scanner layers are untouched.
        assert c['obstacle_layer']==params[name][name]['ros__parameters']['obstacle_layer']
    assert fleet['controller_server']==params['controller_server']
    root=yaml.safe_load(open(robot_params(original,'','',fleet=True)).read())
    assert root==fleet
    amr_02=yaml.safe_load(open(robot_params(original,'amr_02','amr_02/',fleet=True)).read())['amr_02']
    assert amr_02['local_costmap']['local_costmap']['ros__parameters']['fleet_layer']['fleet']['topic']=='/amr_02/fleet/robots'
