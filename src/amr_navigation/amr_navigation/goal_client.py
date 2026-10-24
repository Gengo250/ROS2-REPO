"""Bounded NavigateToPose client and real telemetry recorder (no velocity writer)."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as NavPath,Odometry,OccupancyGrid
from geometry_msgs.msg import TwistStamped,PoseStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def length(points):return sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))
def grid_coordinates(grid,x,y):
    origin=grid.info.origin;angle=yaw(origin.orientation)
    dx=x-origin.position.x;dy=y-origin.position.y
    return (int(math.floor((math.cos(angle)*dx+math.sin(angle)*dy)/grid.info.resolution)),
            int(math.floor((-math.sin(angle)*dx+math.cos(angle)*dy)/grid.info.resolution)))

def free_goal(grid,x,y,radius=.64):
    if not all(math.isfinite(v) for v in [x,y]):return False
    res=grid.info.resolution
    if res<=0:return False
    cx,cy=grid_coordinates(grid,x,y);cells=math.ceil(radius/res)
    for j in range(cy-cells,cy+cells+1):
        for i in range(cx-cells,cx+cells+1):
            if (i-cx)**2+(j-cy)**2>cells*cells:continue
            if not (0<=i<grid.info.width and 0<=j<grid.info.height):return False
            if grid.data[j*grid.info.width+i]<0 or grid.data[j*grid.info.width+i]>=65:return False
    return True


def main():
    p=argparse.ArgumentParser();p.add_argument('x',type=float);p.add_argument('y',type=float);p.add_argument('yaw',type=float)
    p.add_argument('--timeout',type=float,default=180.,help='Wall-clock deadline, seconds')
    p.add_argument('--server-validation',action='store_true',help='Test Nav2 goal rejection directly; Nav2 collision checks remain enabled')
    p.add_argument('--cancel-after',type=float,help='Intentional cancellation after wall seconds')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--action',default='/navigate_to_pose')
    a=p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if not all(math.isfinite(v) for v in [a.x,a.y,a.yaw,a.timeout]) or not 1<=a.timeout<=900:p.error('Finite pose and timeout of 1–900 seconds required')
    rclpy.init();n=rclpy.create_node('navigation_goal_client',parameter_overrides=[Parameter('use_sim_time',value=True)])
    b=Buffer(node=n);listener=TransformListener(b,n);client=ActionClient(n,NavigateToPose,a.action)
    report={'goal_map':[a.x,a.y,a.yaw],'action':a.action,'paths':[],'trajectory':[],'feedback':[],'accepted':False,'passed':False,'collision_events':None,'server_validation':a.server_validation}
    latest={};handle=None;result_future=None;cancelled=False;start=time.monotonic();last_sample=0.;command_count=0
    def path(m):
        if m.header.frame_id=='map':report['paths'].append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]})
    def ground(m):
        for t in m.transforms:
            if t.child_frame_id=='amr_01' and t.header.frame_id=='world':latest['world']=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)];latest['world_stamp']=t.header.stamp.sec+t.header.stamp.nanosec/1e9
    def command(m):
        nonlocal command_count
        command_count+=1;latest['command']=[m.twist.linear.x,m.twist.angular.z]
    n.create_subscription(NavPath,'/plan',path,10)
    n.create_subscription(Odometry,'/odom',lambda m:latest.update(odom=m),qos_profile_sensor_data)
    n.create_subscription(TFMessage,'/digital_twin/simulation/poses',ground,qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid,'/map',lambda m:latest.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(TwistStamped,'/cmd_vel',command,10)
    diagnostics=n.create_publisher(DiagnosticArray,'/navigation/diagnostics',10)
    goal_view=n.create_publisher(PoseStamped,'/navigation/goal',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def status(state,details=''):
        m=DiagnosticArray();m.header.stamp=n.get_clock().now().to_msg();m.status=[DiagnosticStatus(name='navigation/action',hardware_id='warehouse_amr',level=DiagnosticStatus.ERROR if state in ['failed','timeout'] else DiagnosticStatus.OK,message=state,values=[KeyValue(key='goal_map',value=json.dumps(report['goal_map'])),KeyValue(key='details',value=details)])];diagnostics.publish(m)
    def feedback(m):
        f=m.feedback
        row={'wall_elapsed_s':time.monotonic()-start,'distance_remaining_m':f.distance_remaining,'recoveries':f.number_of_recoveries,'navigation_time_s':f.navigation_time.sec+f.navigation_time.nanosec/1e9}
        if not report['feedback'] or row['wall_elapsed_s']-report['feedback'][-1]['wall_elapsed_s']>.5:report['feedback'].append(row)
        status('active',json.dumps(row))
    def spin():
        nonlocal last_sample
        rclpy.spin_once(n,timeout_sec=.03)
        wall=time.monotonic()
        if wall-last_sample>.2:
            last_sample=wall
            try:
                t=b.lookup_transform('map','base_link',rclpy.time.Time());pose=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]
            except Exception:pose=None;t=None
            odom=latest.get('odom');row={'wall':wall,'sim':n.get_clock().now().nanoseconds/1e9,'map':pose,'map_stamp':t.header.stamp.sec+t.header.stamp.nanosec/1e9 if t else None,'world':latest.get('world'),'world_stamp':latest.get('world_stamp'),'command':latest.get('command')}
            if odom:row['odom']=[odom.pose.pose.position.x,odom.pose.pose.position.y,yaw(odom.pose.pose.orientation)];row['velocity']=[odom.twist.twist.linear.x,odom.twist.twist.angular.z]
            report['trajectory'].append(row)
    def wait(future,seconds):
        deadline=time.monotonic()+seconds
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('Action response deadline exceeded')
            spin()
        return future.result()
    try:
        deadline=time.monotonic()+60
        while 'map' not in latest or 'odom' not in latest or not b.can_transform('map','base_link',rclpy.time.Time()) or not client.server_is_ready() or command_count<2 or not n.get_publishers_info_by_topic('/cmd_vel') or any(p.node_name=='_NODE_NAME_UNKNOWN_' for p in n.get_publishers_info_by_topic('/cmd_vel')):
            if time.monotonic()>deadline:raise TimeoutError('Map, odometry, localization or NavigateToPose unavailable')
            spin()
        if not a.server_validation and not free_goal(latest['map'],a.x,a.y):
            report['rejection']='Goal footprint intersects unknown, occupied or outside-map cells';status('rejected',report['rejection']);raise SystemExit(2)
        publishers=n.get_publishers_info_by_topic('/cmd_vel')
        report['cmd_vel_publishers']=[{'node':p.node_name,'type':p.topic_type} for p in publishers]
        if report['cmd_vel_publishers']!=[{'node':'navigation_command_guard','type':'geometry_msgs/msg/TwistStamped'}]:raise RuntimeError('Unexpected controller ownership or command type')
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg();goal.pose.pose.position.x=a.x;goal.pose.pose.position.y=a.y;goal.pose.pose.orientation.z=math.sin(a.yaw/2);goal.pose.pose.orientation.w=math.cos(a.yaw/2)
        start=time.monotonic();sim_start=n.get_clock().now().nanoseconds/1e9
        handle=wait(client.send_goal_async(goal,feedback_callback=feedback),15)
        report['acceptance_latency_s']=time.monotonic()-start;report['accepted']=handle.accepted
        if not handle.accepted:raise RuntimeError('Nav2 rejected goal')
        goal_view.publish(goal.pose)
        result_future=handle.get_result_async();status('accepted')
        while not result_future.done():
            elapsed=time.monotonic()-start
            if elapsed>a.timeout or (a.cancel_after is not None and elapsed>a.cancel_after):
                report['cancel_response_count']=len(wait(handle.cancel_goal_async(),10).goals_canceling);cancelled=True
                report['timed_out']=elapsed>a.timeout;break
            spin()
        result=wait(result_future,15)
        report.update(status=result.status,error_code=result.result.error_code,wall_time_s=time.monotonic()-start,sim_time_s=n.get_clock().now().nanoseconds/1e9-sim_start)
        # Observe original controller deceleration after Nav2 stop/cancel.
        end=time.monotonic()+4
        while time.monotonic()<end:spin()
        final=report['trajectory'][-1];report['stopped']=max(abs(v) for v in final.get('velocity',[float('inf')]))<.02
        if final['map']:
            report['goal_position_error_m']=math.hypot(final['map'][0]-a.x,final['map'][1]-a.y);report['goal_orientation_error_rad']=abs(wrap(final['map'][2]-a.yaw))
        report['passed']=result.status==4 and report.get('goal_position_error_m',999)<.30 and report.get('goal_orientation_error_rad',999)<.20 and report['stopped']
        if a.cancel_after is not None:report['passed']=cancelled and result.status==5 and report['stopped']
        status('completed' if report['passed'] else 'failed',str(report.get('error_code')))
    except Exception as exc:
        report['error']=repr(exc);status('failed',str(exc))
    finally:
        if handle is not None and handle.accepted and (result_future is None or not result_future.done()):
            try:wait(handle.cancel_goal_async(),10)
            except Exception as exc:report['cleanup_error']=repr(exc)
        report['command_samples']=command_count;report['path_publications']=len(report['paths']);report['replans']=max(0,len(report['paths'])-1)
        report['first_planned_path_length_m']=length(report['paths'][0]['points']) if report['paths'] else None
        for field in ['world','odom','map']:report[field+'_travel_distance_m']=length([r[field] for r in report['trajectory'] if r.get(field)])
        report['recoveries']=max((f['recoveries'] for f in report['feedback']),default=0)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ['paths','trajectory','feedback']},indent=2))
        client.destroy();n.destroy_node();rclpy.shutdown()
    if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
