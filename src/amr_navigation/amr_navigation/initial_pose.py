"""Publish an explicit map-frame initial estimate; never reads Gazebo poses."""
import argparse
import math
import time
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy
from tf2_ros import Buffer,TransformListener
from amr_navigation.goal_client import grid_coordinates

def main():
    p=argparse.ArgumentParser();p.add_argument('x',type=float);p.add_argument('y',type=float);p.add_argument('yaw',type=float);p.add_argument('--timeout',type=float,default=90.)
    a=p.parse_args()
    if not all(math.isfinite(x) for x in [a.x,a.y,a.yaw]):p.error('Pose must be finite')
    rclpy.init();n=rclpy.create_node('explicit_initial_pose',parameter_overrides=[Parameter('use_sim_time',value=True)])
    maps=[];estimates=[];b=Buffer(node=n);listener=TransformListener(b,n)
    n.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:estimates.append(m),10)
    pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    try:
        deadline=time.monotonic()+a.timeout
        while not maps or pub.get_subscription_count()==0 or n.get_clock().now().nanoseconds==0:
            if time.monotonic()>deadline:raise TimeoutError('Map, AMCL initialpose subscriber or simulation clock unavailable')
            rclpy.spin_once(n,timeout_sec=.1)
        m=maps[-1];x,y=grid_coordinates(m,a.x,a.y)
        if not (0<=x<m.info.width and 0<=y<m.info.height) or m.data[y*m.info.width+x]<0 or m.data[y*m.info.width+x]>=65:
            raise ValueError('Initial pose must lie in a known free map cell')
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg()
        msg.pose.pose.position.x=a.x;msg.pose.pose.position.y=a.y;msg.pose.pose.orientation.z=math.sin(a.yaw/2);msg.pose.pose.orientation.w=math.cos(a.yaw/2)
        msg.pose.covariance[0]=.16;msg.pose.covariance[7]=.16;msg.pose.covariance[35]=.04
        pub.publish(msg)
        deadline=time.monotonic()+a.timeout
        while not estimates or not b.can_transform('map','base_link',rclpy.time.Time()):
            if time.monotonic()>deadline:raise TimeoutError('AMCL did not produce a pose and connected map TF')
            rclpy.spin_once(n,timeout_sec=.1)
        print('AMCL accepted the explicit initial estimate and produced map → base_link TF. Validate convergence before driving.')
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
