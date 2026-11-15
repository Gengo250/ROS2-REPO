"""Shared launch composition; no robot, clock or world TF is created here."""
import os
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def share(): return Path(get_package_share_directory('amr_navigation'))

# Parameters that name a robot frame or a robot topic. In a robot namespace (multi-robot lite twin,
# amr_lite_sim/fleet.py) frames get the robot's prefix and absolute topics move under the namespace;
# 'map' stays the one shared frame. The validated files themselves are never edited (their hash is a
# source of the Phase 5 graph).
FRAME_KEYS = {'base_frame_id', 'odom_frame_id', 'robot_base_frame', 'local_frame', 'global_frame'}
ROBOT_FRAMES = {'base_link', 'odom'}
TOPIC_KEYS = {'scan_topic', 'topic', 'odom_topic'}


def rewrite_for_robot(params, namespace, frame_prefix):
    """Copy of a parameter tree for the robot in ``namespace`` (no change for the root namespace)."""
    ns = namespace.strip('/')
    def walk(value, key=None):
        if isinstance(value, dict):
            return {k: walk(v, k) for k, v in value.items()}
        if isinstance(value, list):
            return [walk(v) for v in value]
        if key in FRAME_KEYS and value in ROBOT_FRAMES:
            return frame_prefix+value
        if key in TOPIC_KEYS and isinstance(value, str) and value.startswith('/') and ns:
            return f'/{ns}{value}'
        return value
    tree = walk(params)
    # Node keys of a parameter file match the node name only in the root namespace: nest them
    # under the robot namespace (the convention of nav2_bringup's RewrittenYaml root_key).
    return {ns: tree} if ns else tree


# Fleet tracking (multi-robot lite twin, amr_lite_sim/fleet_scan.py): the other AMRs' footprints from
# the engine state, as an extra obstacle layer of both costmaps. Its own layer, so its clearing never
# touches what the scanners observed; marking and clearing over 360 degrees, so a robot that drives
# away leaves no ghost; fail-closed freshness (costmap not current when fleet data stops).
FLEET_TOPIC = '/fleet/robots'


def fleet_layer(max_range):
    return {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True, 'footprint_clearing_enabled': True,
            'observation_sources': 'fleet',
            'fleet': {'topic': FLEET_TOPIC, 'data_type': 'LaserScan', 'marking': True, 'clearing': True,
                      'min_obstacle_height': -0.1, 'max_obstacle_height': 2.0, 'obstacle_min_range': 0.0,
                      'obstacle_max_range': max_range, 'raytrace_min_range': 0.0, 'raytrace_max_range': max_range+1.,
                      'inf_is_valid': True, 'observation_persistence': 0.0, 'expected_update_rate': 0.5}}


def with_fleet_layer(params):
    """Copy of the Nav2 parameter tree with ``fleet_layer`` before the inflation layer of both costmaps
    (local: the rolling window's reach; global: the planner sees robots up to 25 m away)."""
    tree = yaml.safe_load(yaml.safe_dump(params))
    for name, reach in (('local_costmap', 8.), ('global_costmap', 25.)):
        costmap = tree[name][name]['ros__parameters']
        plugins = list(costmap['plugins'])
        if 'fleet_layer' not in plugins:
            plugins.insert(plugins.index('inflation_layer'), 'fleet_layer')
        costmap['plugins'] = plugins
        costmap['fleet_layer'] = fleet_layer(reach)
    return tree


def robot_params(path, namespace='', frame_prefix='', fleet=False):
    """Parameter file for a robot: the original file in the root namespace, else a rewritten copy
    (``fleet``: with the fleet tracking layer, Nav2 parameter files only)."""
    if not namespace.strip('/') and not frame_prefix and not fleet:
        return str(path)
    data = yaml.safe_load(Path(path).read_text())
    if fleet:
        data = with_fleet_layer(data)
    data = rewrite_for_robot(data, namespace, frame_prefix)
    handle, name = tempfile.mkstemp(prefix=f'{namespace.strip("/")}_{Path(path).stem}_', suffix='.yaml')
    with os.fdopen(handle, 'w') as out:
        yaml.safe_dump(data, out, sort_keys=False)
    return name


def localization_actions(map_path, scan_topic='', namespace='', frame_prefix=''):
    if not Path(map_path).is_file():
        raise RuntimeError('Saved occupancy map missing: '+str(map_path)+'. Complete a real SLAM survey first.')
    ns = namespace.strip('/')
    if scan_topic and ns and scan_topic.startswith('/'):
        scan_topic = f'/{ns}{scan_topic}'
    return [
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',namespace=ns or None,
             parameters=[{'use_sim_time':True,'yaml_filename':str(map_path)}]),
        # AMCL must use the same scanner that built the map (empty: amcl_params.yaml).
        Node(package='nav2_amcl',executable='amcl',name='amcl',output='screen',namespace=ns or None,
             parameters=[robot_params(share()/'config/amcl_params.yaml',namespace,frame_prefix)]+([{'scan_topic':scan_topic}] if scan_topic else [])),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',output='screen',
             namespace=ns or None,
             parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','amcl'],'bond_timeout':4.0}])]
