#!/usr/bin/env python3
"""Read-only sensor/costmap evidence; world poses are used ONLY for evaluation."""
import argparse
import json
import math
import signal
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from diagnostic_msgs.msg import DiagnosticArray
from amr_navigation.goal_client import yaw, grid_coordinates


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=300)
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('obstacle_evaluation',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    latest={};rows=[];running=True
    def poses(msg):
        latest['humans']={t.child_frame_id:[t.transform.translation.x,t.transform.translation.y] for t in msg.transforms if t.header.frame_id=='world' and t.child_frame_id.startswith('human_')}
    node.create_subscription(TFMessage,'/digital_twin/simulation/poses',poses,qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m:latest.update(grid=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(LaserScan,'/scan',lambda m:latest.update(scan=m),qos_profile_sensor_data)
    node.create_subscription(TwistStamped,'/cmd_vel',lambda m:latest.update(command=[m.twist.linear.x,m.twist.angular.z]),10)
    node.create_subscription(DiagnosticArray,'/navigation/diagnostics',lambda m:latest.update(interlock=next((s.message for s in m.status if s.name=='navigation/interlock'),latest.get('interlock'))),10)
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    deadline=time.monotonic()+args.duration;last=0
    try:
        while running and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last<.3 or not all(k in latest for k in ['grid','scan','humans']):continue
            last=time.monotonic();grid=latest['grid'];scan=latest['scan']
            try:
                # These transforms are read solely to compare observations in the grid frame.
                laser=buffer.lookup_transform(grid.header.frame_id,scan.header.frame_id,rclpy.time.Time.from_msg(scan.header.stamp)).transform
                map_tf=buffer.lookup_transform(grid.header.frame_id,'map',rclpy.time.Time()).transform
                robot=buffer.lookup_transform(grid.header.frame_id,'base_link',rclpy.time.Time()).transform
            except Exception:continue
            points=[];angle=yaw(laser.rotation)
            for i,r in enumerate(scan.ranges):
                if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                    theta=angle+scan.angle_min+i*scan.angle_increment
                    points.append((laser.translation.x+r*math.cos(theta),laser.translation.y+r*math.sin(theta)))
            row={'wall':time.time(),'sim':scan.header.stamp.sec+scan.header.stamp.nanosec/1e9,'command':latest.get('command'),'interlock':latest.get('interlock'),'humans':{}}
            for name,(wx,wy) in latest['humans'].items():
                # Explicit independently validated map/world registration, never published.
                mx,my=wx+11.,wy;theta=yaw(map_tf.rotation)
                x=map_tf.translation.x+mx*math.cos(theta)-my*math.sin(theta)
                y=map_tf.translation.y+mx*math.sin(theta)+my*math.cos(theta)
                cx,cy=grid_coordinates(grid,x,y);radius=math.ceil(.4/grid.info.resolution)
                values=[grid.data[j*grid.info.width+i] for j in range(cy-radius,cy+radius+1) for i in range(cx-radius,cx+radius+1) if 0<=i<grid.info.width and 0<=j<grid.info.height]
                row['humans'][name]={'world':[wx,wy],'grid':[x,y],'distance_to_robot_m':math.hypot(x-robot.translation.x,y-robot.translation.y),'nearest_scan_endpoint_m':min((math.hypot(x-px,y-py) for px,py in points),default=None),'local_cost_max_near_worker':max(values) if values else None}
            rows.append(row)
    finally:
        args.output.write_text(json.dumps({'registration_for_evaluation_only':[11.,0.,0.],'samples':rows},indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
