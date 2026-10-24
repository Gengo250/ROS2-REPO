"""Real Nav2 lifecycle, command interfaces, maps and TF acceptance probe."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer,TransformListener
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--timeout',type=float,default=90);a=p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init();n=rclpy.create_node('navigation_runtime_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
    b=Buffer(node=n);listener=TransformListener(b,n)
    maps={};clients={};pending={};states={};report={'passed':False}
    for topic in ['/map','/global_costmap/costmap','/local_costmap/costmap']:
        n.create_subscription(OccupancyGrid,topic,lambda m,t=topic:maps.update({t:m}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for name in ['map_server','amcl','planner_server','controller_server','behavior_server','bt_navigator']:
        clients[name]=n.create_client(GetState,'/'+name+'/get_state')
    action=ActionClient(n,NavigateToPose,'/navigate_to_pose');deadline=time.monotonic()+a.timeout;last_check=0
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.1)
            for name,future in list(pending.items()):
                if future.done():states[name]=future.result().current_state.label;del pending[name]
            if time.monotonic()-last_check>1:
                last_check=time.monotonic()
                for name,client in clients.items():
                    if name not in pending and client.service_is_ready():pending[name]=client.call_async(GetState.Request())
            owners=n.get_publishers_info_by_topic('/cmd_vel')
            topology_ready=len(owners)==1 and owners[0].node_name=='navigation_command_guard' and bool(n.get_subscriptions_info_by_topic('/nav_cmd_vel'))
            if all(states.get(k)=='active' for k in clients) and len(maps)==3 and action.server_is_ready() and b.can_transform('map','laser_frame',rclpy.time.Time()) and topology_ready:break
        else:raise TimeoutError('Lifecycle/TF/maps/action readiness failed: '+str(states))
        report['states']=states;report['action']='/navigate_to_pose'
        report['map_to_laser']=message_to_ordereddict(b.lookup_transform('map','laser_frame',rclpy.time.Time()))
        report['tf_frames_yaml']=b.all_frames_as_yaml()
        report['tf_publishers']=[x.node_name for x in n.get_publishers_info_by_topic('/tf')]
        assert sorted(report['tf_publishers'])==['amcl','diff_drive_controller','robot_state_publisher'], 'Unexpected TF publishers'
        report['clock_publishers']=[x.node_name for x in n.get_publishers_info_by_topic('/clock')]
        assert len(report['clock_publishers'])==1, 'Expected one simulation clock publisher'
        report['map_publishers']=[x.node_name for x in n.get_publishers_info_by_topic('/map')]
        assert report['map_publishers']==['map_server']
        report['commands']={}
        for topic in ['/cmd_vel','/nav_cmd_vel']:
            report['commands'][topic]={'publishers':[{'node':x.node_name,'type':x.topic_type} for x in n.get_publishers_info_by_topic(topic)],'subscribers':[{'node':x.node_name,'type':x.topic_type} for x in n.get_subscriptions_info_by_topic(topic)]}
            assert all(x['type']=='geometry_msgs/msg/TwistStamped' for side in report['commands'][topic].values() for x in side)
        assert report['commands']['/cmd_vel']['publishers']==[{'node':'navigation_command_guard','type':'geometry_msgs/msg/TwistStamped'}]
        report['stamped_parameters']={}
        for name in ['controller_server','behavior_server']:
            c=n.create_client(GetParameters,'/'+name+'/get_parameters');assert c.wait_for_service(timeout_sec=5)
            req=GetParameters.Request();req.names=['enable_stamped_cmd_vel','use_sim_time'];f=c.call_async(req)
            end=time.monotonic()+10
            while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
            assert f.done();report['stamped_parameters'][name]=[v.bool_value for v in f.result().values];assert report['stamped_parameters'][name]==[True,True]
        report['maps']={}
        from PIL import Image
        import numpy as np
        a.output.parent.mkdir(parents=True,exist_ok=True)
        for topic,m in maps.items():
            data=np.array(m.data).reshape(m.info.height,m.info.width)
            report['maps'][topic]={'frame':m.header.frame_id,'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'occupied':int((data>=65).sum()),'unknown':int((data<0).sum()),'free':int((data==0).sum())}
            assert m.info.resolution>0 and (data>=65).sum()>0
            pixels=np.where(data<0,128,255-data.clip(0,100)*2.55).astype('uint8')
            Image.fromarray(np.flipud(pixels)).save(a.output.parent/(topic.strip('/').replace('/','-')+'.png'))
        report['passed']=True
    except Exception as exc:report['error']=repr(exc);report['states']=states;raise
    finally:
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');action.destroy();n.destroy_node();rclpy.shutdown()
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
