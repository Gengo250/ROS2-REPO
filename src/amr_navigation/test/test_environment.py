"""Installed Jazzy interfaces, executables and plugin identifiers, without mocks."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory

@pytest.mark.parametrize('package,executable',[
 ('slam_toolbox','async_slam_toolbox_node'),('nav2_amcl','amcl'),('nav2_map_server','map_server'),
 ('nav2_map_server','map_saver_cli'),('nav2_controller','controller_server'),('nav2_planner','planner_server'),
 ('nav2_bt_navigator','bt_navigator'),('nav2_behaviors','behavior_server'),('nav2_lifecycle_manager','lifecycle_manager')])
def test_installed_executables(package,executable):
 assert (Path(get_package_prefix(package))/'lib'/package/executable).is_file()

@pytest.mark.parametrize('package,plugin',[
 ('nav2_navfn_planner','nav2_navfn_planner::NavfnPlanner'),
 ('nav2_regulated_pure_pursuit_controller','nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'),
 ('nav2_amcl','nav2_amcl::DifferentialMotionModel')])
def test_installed_plugins(package,plugin):
 types=[]
 for file in Path(get_package_share_directory(package)).glob('*.xml'):
  types.extend(c.get('type') for c in ET.parse(file).getroot().iter('class'))
 assert plugin in types

def test_navigation_action_interface():
 from nav2_msgs.action import NavigateToPose
 goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
 assert 'distance_remaining' in NavigateToPose.Feedback.get_fields_and_field_types()
 assert 'error_code' in NavigateToPose.Result.get_fields_and_field_types()
