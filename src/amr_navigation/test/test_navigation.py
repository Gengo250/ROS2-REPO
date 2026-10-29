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
