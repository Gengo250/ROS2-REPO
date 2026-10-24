"""Fail-closed stamped command interlock; no path planning or motion generation.

The only /cmd_vel writer during Nav2 operation. Passes Nav2 commands only while
scan, odom, clock and localization TF are fresh. Uses a wall-clock timer so a
paused simulator cannot freeze the guard. Original drive limits remain in force.
"""
import math
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
        super().__init__('navigation_command_guard',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.received={};self.stamps={};self.command=None;self.covariance=None
        self.last_clock=None;self.clock_changed=0.;self.reason='Waiting for sensors and localization';self.last_report=0.
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
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        if reason=='ready':msg=self.command
        self.publisher.publish(msg)
        if reason!=self.reason or wall-self.last_report>1.:
            status=DiagnosticStatus(name='navigation/interlock',hardware_id='warehouse_amr',level=0 if reason=='ready' else 1,message=reason,
                values=[KeyValue(key='command_topic_type',value='geometry_msgs/msg/TwistStamped'),KeyValue(key='localization_source',value='AMCL')])
            d=DiagnosticArray();d.header.stamp=self.get_clock().now().to_msg();d.status=[status];self.diagnostics.publish(d)
            if reason!=self.reason:self.get_logger().info(reason)
            self.last_report=wall;self.reason=reason
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
