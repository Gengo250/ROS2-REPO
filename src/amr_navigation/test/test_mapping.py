from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[1]

def test_mapping_frames_and_sensor_limits():
    p=yaml.safe_load((ROOT/'config/slam_params.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert [p[x] for x in ['map_frame','odom_frame','base_frame','scan_topic']]==['map','odom','base_link','/scan']
    assert p['use_sim_time'] and p['mode']=='mapping'
    assert p['min_laser_range']==.08 and p['max_laser_range']==20.

def test_existing_warehouse_has_only_named_workers_removed():
    world=ET.parse(ROOT.parent/'amr_simulation/worlds/warehouse_normal.sdf').getroot().find('world')
    original=[ET.tostring(e) for e in world if not (e.tag=='model' and e.get('name','').startswith('human_'))]
    assert len(world.findall("model[@name='human_01']"))==1
    for e in list(world.findall('model')):
        if e.get('name') in {'human_01','human_02','human_03'}: world.remove(e)
    assert [ET.tostring(e) for e in world]==original
