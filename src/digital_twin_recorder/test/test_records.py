from pathlib import Path
import math
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from digital_twin_recorder.config import Config
from digital_twin_recorder.models.records import convert, Integrity

def odom():
    msg=Odometry(); msg.header.stamp.sec=123; msg.header.stamp.nanosec=987654321
    msg.header.frame_id='odom'; msg.child_frame_id='base_link'
    msg.pose.pose.position.x=1.25; msg.pose.pose.position.y=-7.5
    msg.pose.pose.orientation.w=.9; msg.pose.pose.orientation.z=.3
    msg.twist.twist.linear.x=.4; msg.twist.twist.angular.z=-.7
    msg.pose.covariance=[float(i)/10 for i in range(36)]
    msg.twist.covariance=[float(i)/20 for i in range(36)]
    return msg

def test_odometry_complete():
    msg=odom(); data=convert(msg,'/odom','amr_01',456)
    assert data['source_timestamp_ns']==123987654321
    assert data['received_wall_ns']==456
    assert data['frame_id']=='odom' and data['child_frame_id']=='base_link'
    assert data['position']==[1.25,-7.5,0]
    assert data['orientation']==[0,0,.3,.9]
    assert data['linear_velocity']==[.4,0,0] and data['angular_velocity']==[0,0,-.7]
    assert data['pose_covariance']==list(msg.pose.covariance)
    assert data['twist_covariance']==list(msg.twist.covariance)

def test_imu_sentinel_and_covariances_preserved():
    msg=Imu(); msg.header.frame_id='imu_link'; msg.header.stamp.nanosec=123
    msg.orientation_covariance=[-1.0]+[0.0]*8
    msg.angular_velocity_covariance=[float(i) for i in range(9)]
    msg.linear_acceleration_covariance=[float(i)/10 for i in range(9)]
    msg.orientation.x=.125; msg.angular_velocity.z=-.75; msg.linear_acceleration.z=9.81
    data=convert(msg,'/imu/data','amr_01',456)
    assert data['source_timestamp_ns']==123 and data['frame_id']=='imu_link'
    assert data['orientation_covariance']==list(msg.orientation_covariance)
    assert data['orientation']==[.125,0,0,1]
    assert data['angular_velocity']==[0,0,-.75]
    assert data['linear_acceleration']==[0,0,9.81]
    assert data['angular_velocity_covariance']==list(msg.angular_velocity_covariance)
    assert data['linear_acceleration_covariance']==list(msg.linear_acceleration_covariance)

def test_integrity_detects_change_and_clock_reset():
    data=convert(odom(),'/odom','amr_01',456)
    a,b=Integrity(),Integrity(); a.add(data); b.add({**data,'frame_id':'map'})
    assert a.report()['sha256']!=b.report()['sha256']
    with pytest.raises(ValueError,match='clock reset'): a.add({**data,'source_timestamp_ns':0})
    msg=odom(); msg.pose.pose.position.x=math.nan
    with pytest.raises(ValueError,match='Nonfinite'): convert(msg,'/odom','amr_01')

@pytest.mark.parametrize('kwargs',[{'queue_size':0},{'duration':-1},{'port':0},{'scenario':'fake'},{'odom_topic':'odom'},{'imu_topic':'/odom'}])
def test_configuration_rejected(kwargs):
    with pytest.raises(ValueError): Config(**kwargs)

def test_custom_ontology_round_trip():
    pytest.importorskip('mosaicolabs')
    from digital_twin_recorder.models.ontology import RosOdometryV1,RosImuV1
    from mosaicolabs import Message
    for model,msg in [(RosOdometryV1,odom()),(RosImuV1,Imu())]:
        data=convert(msg,'/test','amr_01',456)
        assert model.model_validate(data).model_dump()==data
        assert model.__msco_pyarrow_struct__.field('source_timestamp_ns').type.bit_width==64
        with pytest.raises(ValueError): model.model_validate({**data,'orientation':[1,2]})
