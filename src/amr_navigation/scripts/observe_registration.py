#!/usr/bin/env python3
"""Read-only comparison. Ground truth is NEVER published to navigation TF."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from amr_route_planner.coordinate_registration import active, wrap
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from tf2_msgs.msg import TFMessage

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=30);a=p.parse_args()
 # Verified registration of the ACTIVE map and layout; refuses rather than assume an offset.
 reg,record=active()
 rclpy.init();n=rclpy.create_node('registration_evaluation',parameter_overrides=[Parameter('use_sim_time',value=True)])
 b=Buffer(node=n);listener=TransformListener(b,n);rows=[]
 def receive(msg):
  for t in msg.transforms:
   if t.child_frame_id not in ['amr_01','robot/amr_01','warehouse_amr'] and 'amr_01' not in t.child_frame_id:continue
   try:
    estimate=b.lookup_transform('map','base_link',rclpy.time.Time.from_msg(t.header.stamp))
   except Exception:continue
   g=t.transform;e=estimate.transform
   expected=reg.apply_pose(g.translation.x,g.translation.y,yaw(g.rotation))
   rows.append({'stamp':t.header.stamp.sec+t.header.stamp.nanosec/1e9,
     'world':[g.translation.x,g.translation.y,yaw(g.rotation)],'map':[e.translation.x,e.translation.y,yaw(e.rotation)],
     'error_m':math.hypot(e.translation.x-expected[0],e.translation.y-expected[1]),
     'yaw_error_rad':wrap(yaw(e.rotation)-expected[2])})
 n.create_subscription(TFMessage,'/digital_twin/simulation/poses',receive,qos_profile_sensor_data)
 end=time.monotonic()+a.duration
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 a.output.write_text(json.dumps({'registration':{'id':record['id'],'transform':record['transform'],'map_version':record['map_version'],
   'usage':'Verified world->map SE(2) of the active map; evaluation only, never TF.'},'samples':rows},indent=2)+'\n')
 print('matched observations',len(rows));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
