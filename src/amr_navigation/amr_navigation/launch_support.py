"""Shared launch composition; no robot, clock or world TF is created here."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def share(): return Path(get_package_share_directory('amr_navigation'))

def localization_actions(map_path):
    if not Path(map_path).is_file():
        raise RuntimeError('Saved occupancy map missing: '+str(map_path)+'. Complete a real SLAM survey first.')
    return [
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
             parameters=[{'use_sim_time':True,'yaml_filename':str(map_path)}]),
        Node(package='nav2_amcl',executable='amcl',name='amcl',output='screen',
             parameters=[str(share()/'config/amcl_params.yaml')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',output='screen',
             parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','amcl'],'bond_timeout':4.0}])]
