#!/usr/bin/env python3
"""One supervised relative displacement for surveying; NOT autonomous navigation.

Requires SLAM and exclusive /cmd_vel ownership. Rotation first, then forward
translation from SLAM pose feedback; stops on a forward LiDAR obstacle, stale input or a wall deadline.
Rotations need the 360-degree /top_scan to show nothing inside the rotation clearance;
without it a rotation is refused unless --allow-unobserved-rotation is given.
No world poses, routes or map cells are synthesized here.
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, Pose
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(x): return math.atan2(math.sin(x),math.cos(x))

def main():
    p=argparse.ArgumentParser(); p.add_argument('--turn',type=float,default=0.,help='Relative radians, at most pi')
    p.add_argument('--heading',type=float,help='Optional absolute heading in SLAM map frame')
    p.add_argument('--distance',type=float,default=0.,help='Forward metres after turning')
    p.add_argument('--rotation-clearance',type=float,default=.80,help='Required free radius around base_link while turning, m')
    p.add_argument('--allow-unobserved-rotation',action='store_true',help='Operator accepts turning without 360-degree observation')
    p.add_argument('--output',type=Path,required=True); args=p.parse_args()
    if not 0 <= args.distance <= 25 or abs(args.turn)>math.pi: p.error('Invalid bounded jog')
    rclpy.init(); n=rclpy.create_node('supervised_mapping_jog',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=n); listener=TransformListener(buffer,n)
    latest={}; received={}
    def get(key,m): latest[key]=m; received[key]=time.monotonic()
    n.create_subscription(Odometry,'/odom',lambda m:get('odom',m),qos_profile_sensor_data)
    n.create_subscription(LaserScan,'/scan',lambda m:get('scan',m),qos_profile_sensor_data)
    top={}
    n.create_subscription(LaserScan,'/top_scan',lambda m:top.update(msg=m,received=time.monotonic()),qos_profile_sensor_data)
    deadline=time.monotonic()+25
    while len(latest)<2:
        if time.monotonic()>deadline: raise TimeoutError('Sensors unavailable')
        rclpy.spin_once(n,timeout_sec=.1)
    discovery_deadline=time.monotonic()+3.
    while time.monotonic()<discovery_deadline: rclpy.spin_once(n,timeout_sec=.1)
    if n.get_publishers_info_by_topic('/cmd_vel') or '/slam_toolbox' not in ['/'+x for x in n.get_node_names()]:
        raise RuntimeError('Requires SLAM and exclusive command ownership')
    tf_deadline=time.monotonic()+20.
    while not buffer.can_transform('map','base_link',rclpy.time.Time()):
        if time.monotonic()>tf_deadline: raise TimeoutError('SLAM TF unavailable')
        rclpy.spin_once(n,timeout_sec=.1)
    pub=n.create_publisher(TwistStamped,'/cmd_vel',10)
    report={'feedback_frame':'map (SLAM estimate)','kind':'supervised_mapping_only','turn_rad':args.turn,'distance_requested_m':args.distance,'samples':[],'passed':False}
    def command(v,w):
        m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=v;m.twist.angular.z=w;pub.publish(m)
    def pose():
        tf=buffer.lookup_transform('map','base_link',rclpy.time.Time())
        state=Pose();state.position.x=tf.transform.translation.x;state.position.y=tf.transform.translation.y;state.orientation=tf.transform.rotation
        return state
    target=wrap(args.heading if args.heading is not None else yaw(pose().orientation)+args.turn); phase='turn'; origin=None
    mount=None
    def rotation_clearance():
        """Smallest distance from base_link to any 360-degree return; None if unobserved."""
        nonlocal mount
        # Freshness by simulation stamp (robust to a slow real-time factor); a wall-clock
        # bound still catches a stalled simulator or bridge.
        if 'msg' not in top or time.monotonic()-top['received']>3.:return None
        m=top['msg']
        if n.get_clock().now().nanoseconds/1e9-(m.header.stamp.sec+m.header.stamp.nanosec/1e9)>.6:return None
        if mount is None:
            try:t=buffer.lookup_transform('base_link',m.header.frame_id,rclpy.time.Time()).transform
            except Exception:return None  # static TF not received yet: treated as unobserved
            mount=(t.translation.x,t.translation.y,yaw(t.rotation))
        return min((math.hypot(mount[0]+r*math.cos(mount[2]+m.angle_min+i*m.angle_increment),
                               mount[1]+r*math.sin(mount[2]+m.angle_min+i*m.angle_increment))
                    for i,r in enumerate(m.ranges) if math.isfinite(r) and m.range_min<=r<=m.range_max),default=math.inf)
    if abs(wrap(target-yaw(pose().orientation)))>=.02:
        settle=time.monotonic()+5.
        # Wait for the 360-degree scan AND its static mount transform before turning.
        while ('msg' not in top or not buffer.can_transform('base_link',top['msg'].header.frame_id,rclpy.time.Time())) and time.monotonic()<settle:
            rclpy.spin_once(n,timeout_sec=.1)
        if 'msg' not in top and not args.allow_unobserved_rotation:
            raise RuntimeError('Rotation refused: no 360-degree observation (/top_scan); pass --allow-unobserved-rotation only after inspecting the surroundings')
    report['rotation_clearance_m']=None
    deadline=time.monotonic()+max(120,args.distance*10+120)
    last_sample=0;last_progress=0
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.02)
            now=time.monotonic()
            if any(now-t>1.5 for t in received.values()): raise TimeoutError('Stale sensor input')
            if len(n.get_publishers_info_by_topic('/cmd_vel'))>1: raise RuntimeError('Competing command publisher')
            state=pose(); error=wrap(target-yaw(state.orientation)); dist=0 if origin is None else math.hypot(state.position.x-origin[0],state.position.y-origin[1])
            if now-last_sample>.5:
                report['samples'].append([n.get_clock().now().nanoseconds/1e9,state.position.x,state.position.y,yaw(state.orientation)]);last_sample=now
            if now-last_progress>5:
                print(json.dumps({'phase':phase,'map_pose':[state.position.x,state.position.y,yaw(state.orientation)],'remaining_m':args.distance-dist}),flush=True);last_progress=now
            if phase=='turn':
                # Already aligned: no rotation is commanded, so no rotation clearance is needed.
                if abs(error)<.02:
                    phase='drive';origin=(state.position.x,state.position.y);continue
                clearance=rotation_clearance()
                if clearance is not None:
                    report['rotation_clearance_m']=clearance if report['rotation_clearance_m'] is None else min(report['rotation_clearance_m'],clearance)
                    if clearance<args.rotation_clearance: raise RuntimeError(f'Rotation stopped: obstacle {clearance:.2f} m from base_link')
                elif not args.allow_unobserved_rotation: raise RuntimeError('Rotation stopped: 360-degree observation stale')
                command(0.,max(-.15,min(.15,error)))
            elif args.distance-dist<.04:
                report['passed']=True;report['distance_measured_m']=dist;break
            else:
                scan=latest['scan']
                # Stop with clearance from the laser at x=0.44; no reverse.
                close=min((r for i,r in enumerate(scan.ranges) if abs(scan.angle_min+i*scan.angle_increment)<.65 and math.isfinite(r)),default=20.)
                if close<.65: raise RuntimeError('Survey stopped: obstacle in forward sector')
                command(min(.55,max(.06,(args.distance-dist)*.6)),max(-.25,min(.25,error)))
        else: raise TimeoutError('Jog wall timeout')
    finally:
        for _ in range(25): command(0.,0.);rclpy.spin_once(n,timeout_sec=.02)
        args.output.write_text(json.dumps(report,indent=2)+'\n');n.destroy_node();rclpy.shutdown()
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}))
if __name__=='__main__': main()
