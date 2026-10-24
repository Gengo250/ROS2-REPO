"""Fail-closed stamped command interlock; no path planning or motion generation.

The only /cmd_vel writer during Nav2 operation. Passes Nav2 commands only while
scan, odom, clock and localization TF are fresh. Uses a wall-clock timer so a
paused simulator cannot freeze the guard. Original drive limits remain in force.
"""
import math
import os
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped,PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from tf2_ros import Buffer,TransformListener


def command_is_finite(command):
    return all(math.isfinite(v) for v in [command.twist.linear.x,command.twist.linear.y,command.twist.linear.z,
        command.twist.angular.x,command.twist.angular.y,command.twist.angular.z])


class CommandGuard(Node):
    def __init__(self):
        if not os.environ.get('GZ_PARTITION'):
            raise RuntimeError('Set GZ_PARTITION to the existing warehouse partition before starting navigation')
        super().__init__('navigation_command_guard',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.received={};self.stamps={};self.command=None;self.covariance=None
        self.last_clock=None;self.clock_changed=0.;self.last_motion=0.;self.emergency_latched=False;self.pause_process=None;self.pause_acknowledged=False;self.next_pause_retry=0.;self.reason='Waiting for sensors and localization';self.last_report=0.
        self.create_subscription(LaserScan,'/scan',lambda m:self.sensor('scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',lambda m:self.sensor('odom',m),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.localized,10)
        self.create_subscription(TwistStamped,'/nav_cmd_vel',self.velocity,10)
        self.publisher=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.diagnostics=self.create_publisher(DiagnosticArray,'/navigation/diagnostics',10)
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def sensor(self,key,msg):
        self.received[key]=time.monotonic();self.stamps[key]=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if key=='scan' and (msg.header.frame_id!='laser_frame' or not any(math.isfinite(r) and msg.range_min<=r<=msg.range_max for r in msg.ranges)):
            self.received.pop(key,None)
    def localized(self,msg):
        self.covariance=msg.pose.covariance
    def velocity(self,msg):
        self.command=msg;self.sensor('command',msg)
    def check(self,wall,sim):
        if self.last_clock is None or sim!=self.last_clock:
            if self.last_clock is not None and sim<self.last_clock:
                self.command=None;self.covariance=None;self.received.clear();self.buffer.clear()
                self.last_clock=sim;self.clock_changed=wall
                return 'Simulation clock reset'
            self.last_clock=sim;self.clock_changed=wall
        if sim<=0 or wall-self.clock_changed>1.:return 'Simulation clock stopped or reset'
        for key in ['scan','odom','command']:
            if key not in self.received or wall-self.received[key]>1.:return 'Missing or stale '+key
            age=sim-self.stamps[key]
            if age < -.1 or age > .4:return 'Invalid simulation timestamp: '+key
        if self.command is None or not command_is_finite(self.command):return 'Invalid command'
        if self.covariance is None or any(not math.isfinite(self.covariance[i]) or self.covariance[i]>1. for i in [0,7,35]):return 'Localization unavailable or uncertain'
        try:
            for parent,child in [('map','odom'),('odom','base_link')]:
                t=self.buffer.lookup_transform(parent,child,rclpy.time.Time())
                age=sim-(t.header.stamp.sec+t.header.stamp.nanosec/1e9)
                if age>.5 or age<-.8:return 'Stale localization/odometry TF'
            self.buffer.lookup_transform('base_link','laser_frame',rclpy.time.Time())
        except Exception:return 'Localization TF unavailable'
        others=[p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel') if p.node_name!=self.get_name()]
        if others:return 'Conflicting command publishers: '+','.join(others)
        v=self.command.twist
        if v.linear.x<0 or abs(v.linear.x)>.70 or abs(v.angular.z)>1.50:return 'Command outside drive limits'
        return 'ready'
    def tick(self):
        wall=time.monotonic();sim=self.get_clock().now().nanoseconds/1e9;reason=self.check(wall,sim)
        # ros2_control's simulation-clock update loop can stall while Gazebo keeps
        # integrating the last wheel command. Zero Twist alone cannot stop that
        # failure. Pause the SAME Gazebo partition through its native service.
        if reason=='Simulation clock reset' or (reason=='Simulation clock stopped or reset' and wall-self.last_motion<3.):
            self.emergency_latched=True
        if self.emergency_latched:
            reason='Emergency simulation pause: clock lost/reset; restart navigation'
            self.emergency_pause(wall)
        elif reason=='ready' and (abs(self.command.twist.linear.x)>.001 or abs(self.command.twist.angular.z)>.001):
            self.last_motion=wall
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        if reason=='ready':msg=self.command
        self.publisher.publish(msg)
        if reason!=self.reason or wall-self.last_report>1.:
            status=DiagnosticStatus(name='navigation/interlock',hardware_id='warehouse_amr',level=DiagnosticStatus.OK if reason=='ready' else DiagnosticStatus.WARN,message=reason,
                values=[KeyValue(key='command_topic_type',value='geometry_msgs/msg/TwistStamped'),KeyValue(key='localization_source',value='AMCL')])
            d=DiagnosticArray();d.header.stamp=self.get_clock().now().to_msg();d.status=[status];self.diagnostics.publish(d)
            if reason!=self.reason:self.get_logger().info(reason)
            self.last_report=wall;self.reason=reason
    def emergency_pause(self,wall):
        if self.pause_acknowledged:return
        if self.pause_process is not None:
            code=self.pause_process.poll()
            if code is None:return
            output=self.pause_process.communicate()[0]
            self.pause_process=None
            # gz service can exit zero on timeout; require the actual Boolean ACK.
            if code==0 and 'data: true' in output:
                self.pause_acknowledged=True
                self.get_logger().error('Gazebo confirmed emergency pause')
                return
            self.get_logger().error('Emergency pause not acknowledged: '+output.strip())
        if wall<self.next_pause_retry:return
        self.next_pause_retry=wall+2.
        # No unpause is issued automatically. A bounded subprocess leaves the
        # wall-clock zero-command timer responsive even if Gazebo is unavailable.
        if not os.environ.get('GZ_PARTITION'):
            self.get_logger().error('Cannot pause safely: GZ_PARTITION must match the existing warehouse')
            return
        self.get_logger().error('Pausing existing Gazebo after simulation clock loss')
        self.pause_process=subprocess.Popen(['gz','service','-s','/world/warehouse/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
            '--timeout','1000','--req','pause: true'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)

    def stop(self):
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link';self.publisher.publish(msg)


def main():
    rclpy.init();node=CommandGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.stop()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
