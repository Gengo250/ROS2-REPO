#!/usr/bin/env python3
"""Measured ROS/Gazebo acceptance against an already running warehouse launch."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.world_stats_pb2 import WorldStatistics
from PIL import Image as PILImage

from generate_warehouse import ROOT, load_config


def yaw(p):
    q = p['orientation']
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))


def delta_angle(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))


def distance(a,b):
    return math.hypot(a['position']['x']-b['position']['x'], a['position']['y']-b['position']['y'])


class Acceptance:
    def __init__(self, scenario, output):
        self.scenario, self.output = scenario, output
        self.c, self.amr = load_config()
        self.manifest_digest = hashlib.sha256((ROOT/'config/generated_manifest.json').read_bytes()).hexdigest()
        rclpy.init()
        self.node = rclpy.create_node('warehouse_acceptance', parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.received, self.counts, self.poses, self.contacts = {}, Counter(), {}, Counter()
        self.rtf, self.history, self.checks, self.metrics = [], [], [], {}
        self.topics = {'/scan':LaserScan, '/imu/data':Imu, '/camera/image_raw':Image,
            '/camera/depth/image_raw':Image, '/camera/camera_info':CameraInfo,
            '/camera/points':PointCloud2, '/odom':Odometry, '/joint_states':JointState,
            self.c['camera']['topic']+'/image_raw':Image, self.c['camera']['topic']+'/camera_info':CameraInfo}
        self.subs = [self.node.create_subscription(kind, topic, lambda msg,t=topic:self.receive(t,msg),qos_profile_sensor_data)
                     for topic,kind in self.topics.items()]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self.node)
        self.pub = self.node.create_publisher(TwistStamped,'/cmd_vel',10)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V,'/world/warehouse/dynamic_pose/info',self.on_poses)
        self.gz.subscribe(WorldStatistics,'/world/warehouse/stats',self.on_stats)
        # Only subscribe to the collision probes. Sim 8's Contact system emits
        # at physics rate, including continuous floor contact for moving people.
        for name in ('human_03','wall','barrier'):
            self.gz.subscribe(Contacts,'/warehouse/contacts/'+name,lambda msg,n=name:self.on_contact(n,msg))

    def receive(self,topic,msg):
        self.received[topic] = msg
        self.counts[topic] += 1

    def on_poses(self,msg):
        now = msg.header.stamp.sec+msg.header.stamp.nsec/1e9
        people = {}
        for p in msg.pose:
            if p.name in ('warehouse_amr','human_01','human_02'):
                data = {'position':{k:getattr(p.position,k) for k in 'xyz'},
                        'orientation':{k:getattr(p.orientation,k) for k in 'xyzw'}}
                self.poses[p.name] = data
                if p.name.startswith('human_'):
                    people[p.name] = data
        if people and (not self.history or now-self.history[-1]['time'] >= .5):
            self.history.append({'time':now, **people})

    def on_stats(self,msg):
        if msg.real_time_factor > 0:
            self.rtf.append(msg.real_time_factor)

    def on_contact(self,name,msg):
        for contact in msg.contact:
            if 'warehouse_amr' in contact.collision1.name or 'warehouse_amr' in contact.collision2.name:
                self.contacts[name] += 1

    def require(self,condition,message):
        if not condition:
            raise AssertionError(message)
        self.checks.append(message)
        print('PASS:',message,flush=True)

    def now(self):
        return self.node.get_clock().now().nanoseconds/1e9

    def until(self,predicate,timeout=90):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end:
                raise TimeoutError('Condition timeout. Topics: '+str(self.counts))
            rclpy.spin_once(self.node,timeout_sec=.02)

    def wait(self,seconds):
        start = self.now()
        self.until(lambda:self.now()-start>=seconds,timeout=max(60,seconds*20))

    def drive(self,v,w,seconds):
        start,deadline = self.now(),time.monotonic()+max(90,seconds*20)
        while self.now()-start < seconds:
            if time.monotonic()>deadline:
                raise TimeoutError('Simulation stopped during drive')
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.angular.z = float(v),float(w)
            self.pub.publish(msg)
            rclpy.spin_once(self.node,timeout_sec=.02)

    def robot(self):
        return self.poses['warehouse_amr']

    def set_pose(self,pose):
        self.drive(0,0,.6)
        msg = Pose(name='warehouse_amr')
        msg.position.x,msg.position.y,msg.position.z = pose[:3]
        msg.orientation.z,msg.orientation.w = math.sin(pose[5]/2), math.cos(pose[5]/2)
        # Isolate synchronous test requests from contact observation. Contact
        # callbacks run at physics rate; the CLI leaves this observer's Python
        # callbacks free to progress while it waits for the same native service.
        result = subprocess.run(['gz','service','-s','/world/warehouse/set_pose',
            '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean',
            '--timeout','5000','--req',str(msg)],capture_output=True,text=True,timeout=15)
        self.require(result.returncode==0 and 'data: true' in result.stdout,
                     'test reposition accepted')
        self.wait(1.2)
        self.require(math.hypot(self.robot()['position']['x']-pose[0],self.robot()['position']['y']-pose[1])<.06,
                     'test reposition reached without drift')

    def capture(self,topic,name):
        msg = self.received[topic]
        self.require(msg.encoding in ('rgb8','bgr8'),topic+' RGB encoding')
        raw = bytes(msg.data)
        img = PILImage.frombytes('RGB',(msg.width,msg.height),raw,'raw','BGR' if msg.encoding=='bgr8' else 'RGB',msg.step)
        img.save(self.output/(name+'.png'))
        extrema = img.getextrema()
        self.require(max(hi-lo for lo,hi in extrema)>50,topic+' nonblank rendered image')

    def probe(self,key,label,expected):
        self.set_pose(self.c['validation'][key])
        scan = self.received['/scan']
        centre = scan.ranges[len(scan.ranges)//2]
        self.require(abs(centre-expected)<.14, label+f' LiDAR distance {centre:.3f} m (expected {expected:.3f})')
        self.metrics[label+'_scan_m'] = centre
        self.capture('/camera/image_raw',label+'_rgb')
        return centre

    def contact_drive(self,name,seconds):
        before = self.robot()
        count = self.contacts[name]
        self.drive(self.c['validation']['contact_speed'],0,seconds)
        self.drive(0,0,.8)
        after = self.robot()
        self.metrics[name+'_contact'] = {'events':self.contacts[name]-count,'start':before,'end':after}
        self.require(self.contacts[name]>count,name+' physical contact with AMR reported by Gazebo')
        return before,after

    def run(self):
        self.until(lambda:all(self.counts[t]>=3 for t in self.topics) and 'warehouse_amr' in self.poses)
        self.wait(1)
        from google.protobuf import text_format
        from gz.msgs10.scene_pb2 import Scene
        state = subprocess.run(['gz','service','-s','/world/warehouse/scene/info',
            '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','5000','--req',''],
            capture_output=True,text=True,check=True,timeout=15)
        scene = Scene(); text_format.Parse(state.stdout,scene)
        names = [m.name for m in scene.model]
        self.metrics['runtime_entities'] = names
        self.require('warehouse_amr' in names,'AMR entity exists in Gazebo scene')
        self.require(('corridor_barrier' in names)==(self.scenario=='corridor_blocked'),
                     'runtime barrier matches selected scenario')
        self.require(('aisle_pallet' in names)==(self.scenario=='obstacle_in_aisle'),
                     'runtime aisle load matches selected scenario')
        self.metrics['spawn_actual'] = self.robot()
        counts, start, wall = dict(self.counts), self.now(), time.monotonic()
        self.wait(3)
        self.metrics['topic_rates'] = {t:{'samples':self.counts[t]-counts[t],
            'hz_sim':(self.counts[t]-counts[t])/(self.now()-start),
            'hz_wall':(self.counts[t]-counts[t])/(time.monotonic()-wall)} for t in self.topics}
        self.require(abs(self.robot()['position']['z'])<.006,'AMR settles on floor')
        spawn = next(s['pose'] for s in self.c['spawns'] if s['active'])
        self.require(math.hypot(self.robot()['position']['x']-spawn[0],self.robot()['position']['y']-spawn[1])<.02,
                     'AMR spawns at configured clear pose')
        for topic in self.topics:
            msg = self.received[topic]
            age = self.now()-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
            self.require(-.1<age<1.5,topic+' publishes fresh samples')
        self.require(self.received['/scan'].header.frame_id=='laser_frame','scan frame laser_frame')
        self.require(len(self.received['/scan'].ranges)==self.amr['lidar_samples'],'LiDAR sample count preserved')
        self.require(abs(self.received['/imu/data'].linear_acceleration.z-9.81)<.2,'IMU gravity preserved')
        self.require(self.received['/camera/points'].header.frame_id=='camera_depth_frame','point cloud X-forward frame preserved')
        for root,frame in [('base_link','laser_frame'),('base_link','camera_optical_frame'),
                           ('odom','base_link'),('world',self.c['camera']['optical_frame'])]:
            self.until(lambda:self.buffer.can_transform(root,frame,rclpy.time.Time()),20)
            self.require(True,'TF '+root+' -> '+frame)
        camtopic = self.c['camera']['topic']
        self.require(self.received[camtopic+'/image_raw'].header.frame_id==self.c['camera']['optical_frame'],
                     'fixed camera optical frame')
        info = self.received[camtopic+'/camera_info']
        self.require(info.width==self.c['camera']['width'] and info.k[0]>0,'fixed camera resolution and intrinsics')
        self.capture(camtopic+'/image_raw','warehouse_camera')
        if self.scenario == 'normal':
            v = self.c['validation']
            before = self.robot()
            odom_before = self.received['/odom'].pose.pose.position.x
            self.metrics['motion_start_pose'] = before
            self.metrics['motion_start_odom'] = str(self.received['/odom'])
            self.drive(v['straight_speed'],0,v['straight_seconds'])
            self.wait(1.5)  # actual command timeout, no zero command
            after = self.robot()
            displacement = distance(before,after)
            odom_delta = self.received['/odom'].pose.pose.position.x-odom_before
            self.require(.7<displacement<1.6,'straight motion in main corridor')
            self.require(abs(odom_delta-displacement)<.1,'odometry agrees with ground truth translation')
            self.require(abs(self.received['/odom'].twist.twist.linear.x)<.015,'TwistStamped timeout stops AMR')
            self.metrics['straight_m'] = displacement
            self.metrics['odom_m'] = odom_delta
            self.metrics['motion_end_pose'] = after
            self.metrics['motion_end_odom'] = str(self.received['/odom'])
            before = self.robot()
            self.drive(0,v['turn_speed'],v['turn_seconds']); self.drive(0,0,.8)
            angle = delta_angle(yaw(self.robot()),yaw(before))
            self.require(.65<angle<1.3,'in-place turn')
            self.metrics['turn_rad'] = angle
            before = self.robot()
            self.drive(v['curve_speed'],v['curve_angular'],v['curve_seconds']); self.drive(0,0,.8)
            self.require(distance(before,self.robot())>.4 and delta_angle(yaw(self.robot()),yaw(before))>.3,'curved motion')
            self.set_pose(v['aisle_start'])
            before = self.robot()
            self.drive(v['aisle_speed'],0,v['aisle_seconds']); self.drive(0,0,.8)
            self.require(distance(before,self.robot())>4.5,'passes through rack aisle into main corridor')
            self.require(abs(self.robot()['position']['x']-v['aisle_start'][0])<.08,'rack corridor traversal stays centred')
            self.capture('/camera/image_raw','rack_aisle_rgb')
            lidar_x = self.amr['chassis_length']/2-self.amr['lidar_radius']
            wall_y = -self.c['building']['width']/2
            pillar_y = min(p[1] for p in self.c['building']['pillars']['positions'])
            expected = v['pillar_probe'][1]-pillar_y-self.c['building']['pillars']['size'][1]/2-lidar_x
            self.probe('pillar_probe','pillar',expected)
            self.probe('wall_probe','wall',v['wall_probe'][1]-wall_y-lidar_x)
            before,after = self.contact_drive('wall',v['contact_seconds']+4)
            self.require(after['position']['y']>wall_y+self.amr['chassis_length']/2-.04,'wall prevents AMR crossing perimeter')
            expected = v['rack_probe'][0]-(self.c['racks']['row_x'][0]+self.c['racks']['size'][0]/2)-lidar_x
            self.probe('rack_probe','rack',expected)
            # Push slowly against the rack, independently of wheel-encoder odometry.
            before = self.robot(); self.drive(.1,0,15); self.drive(0,0,.8)
            rack_edge = self.c['racks']['row_x'][0]+self.c['racks']['size'][0]/2
            self.require(self.robot()['position']['x']>rack_edge+self.amr['chassis_length']/2-.04,'rack collision prevents penetration')
            self.require(distance(before,self.robot())<1.2,'rack blocks physical displacement')
            load = self.c['floor_loads'][0]
            extent = self.c['load']['rows']*self.c['box']['size'][1]+(self.c['load']['rows']-1)*self.c['load']['gap']
            expected = load['pose'][1]-extent/2-v['load_probe'][1]-lidar_x
            self.probe('load_probe','loaded_pallet_boxes',expected)
            depth = self.received['/camera/depth/image_raw']
            self.require(depth.encoding=='32FC1','depth metric float encoding')
            centre = struct.unpack_from('<f',depth.data,depth.height//2*depth.step+depth.width//2*4)[0]
            cloud = self.received['/camera/points']
            offsets = {f.name:f.offset for f in cloud.fields}
            point = struct.unpack_from('<f',cloud.data,cloud.height//2*cloud.row_step+cloud.width//2*cloud.point_step+offsets['x'])[0]
            self.require(math.isfinite(centre) and .2<centre<2,'RGB-D measures logistics load')
            self.require(abs(point-centre)<.06,'point cloud and depth agree')
            self.metrics['depth_m'],self.metrics['cloud_x_m'] = centre,point
            self.set_pose(v['human_probe'])
            scan = self.received['/scan']; closest = min(scan.ranges[len(scan.ranges)//2-15:len(scan.ranges)//2+16])
            self.require(.7<closest<1.4,'human legs detected by GPU LiDAR')
            self.metrics['human_scan_m'] = closest
            self.capture('/camera/image_raw','human_rgb')
            before,after = self.contact_drive('human_03',v['contact_seconds']+3)
            human_x = self.c['humans'][2]['pose'][0]
            self.require(after['position']['x']<human_x-self.c['human_model']['radius']-.35,'human collision prevents traversal')
            self.metrics['human_stopped_gap_m'] = human_x-after['position']['x']
        elif self.scenario in ('obstacle_in_aisle','corridor_blocked'):
            pose = list(self.c['validation']['aisle_start'])
            self.set_pose(pose)
            scan = self.received['/scan']
            if self.scenario=='corridor_blocked':
                centre = scan.ranges[len(scan.ranges)//2]
                lidar_x = self.amr['chassis_length']/2-self.amr['lidar_radius']
                expected = self.c['obstacles']['barrier']['pose'][1]-self.c['obstacles']['barrier']['size'][1]/2-pose[1]-lidar_x
                self.require(abs(centre-expected)<.1,'barrier detected across blocked aisle')
                self.metrics['barrier_scan_m']=centre
                self.capture('/camera/image_raw','corridor_blocked_rgb')
                before,after = self.contact_drive('barrier',18)
                limit = self.c['obstacles']['barrier']['pose'][1]-self.c['obstacles']['barrier']['size'][1]/2
                self.require(after['position']['y']<limit-.42,'barrier physically blocks corridor')
            else:
                # Probe aligned to the load, then demonstrate the remaining free passage.
                # Aim at a carton, not the intentional gap between columns.
                pose[0] = self.c['obstacles']['aisle_load']['pose'][0]-(self.c['box']['size'][0]+self.c['load']['gap'])/2
                self.set_pose(pose)
                centre = self.received['/scan'].ranges[len(scan.ranges)//2]
                self.require(.8<centre<1.5,'temporary loaded pallet detected in aisle')
                self.capture('/camera/image_raw','obstacle_in_aisle_rgb')
                self.metrics['aisle_load_scan_m']=centre
                before = self.robot()
                self.drive(self.c['validation']['contact_speed'],0,18); self.drive(0,0,.8)
                limit = self.c['obstacles']['aisle_load']['pose'][1]-self.c['pallet']['size'][1]/2
                self.require(self.robot()['position']['y']<limit-self.amr['chassis_length']/2+.04,
                             'aisle pallet collision prevents penetration')
                self.require(distance(before,self.robot())<1.1,'aisle pallet blocks physical displacement')
                pose[0]=self.c['validation']['aisle_pass_x']
                self.set_pose(pose)
                self.drive(.3,0,10); self.drive(0,0,.8)
                self.require(self.robot()['position']['y']>-4.0,'AMR passes remaining width beside aisle pallet')
        # Observe physical humans over enough simulation time to include their motion.
        if self.now()<self.c['validation']['route_observation_seconds']:
            self.wait(self.c['validation']['route_observation_seconds']-self.now())
        for name in ('human_01','human_02'):
            samples = [s[name] for s in self.history if name in s]
            self.require(len(samples)>5,name+' trajectory ground truth available')
            span = max(p['position']['y'] for p in samples)-min(p['position']['y'] for p in samples)
            self.require(span>1.,name+' moves along configured route')
            self.require(max(abs(p['position']['z']) for p in samples)<.08,name+' remains upright on floor')
            tilt = max(math.acos(max(-1,min(1,1-2*(p['orientation']['x']**2+p['orientation']['y']**2))))
                       for p in samples)
            self.require(tilt<.15,name+' body tilt remains stable')
            self.metrics[name+'_y_span_m']=span
            self.metrics[name+'_max_tilt_rad']=tilt
        if self.scenario=='person_crossing':
            yy=[s['human_01']['position']['y'] for s in self.history if 'human_01' in s]
            self.require(min(yy)<-.5 and max(yy)>.5,'human_01 crosses main AMR route y=0')
        self.metrics['rtf'] = {'min':min(self.rtf),'max':max(self.rtf),'mean':sum(self.rtf)/len(self.rtf),
                               'last':self.rtf[-1], 'samples':len(self.rtf)}
        self.require(abs(self.robot()['position']['z'])<.01,'AMR remains stable after physical tests')
        self.drive(0,0,.5)

    def execute(self):
        status,error = 'passed',None
        try:
            self.run()
        except Exception as exc:
            status,error='failed',repr(exc)
            raise
        finally:
            report = {'status':status,'error':error,'scenario':self.scenario,'checks':self.checks,
                      'manifest_sha256':self.manifest_digest,
                      'topic_samples':dict(self.counts),'metrics':self.metrics,'contacts':dict(self.contacts),
                      'sim_seconds':self.now(),'human_trajectories':self.history}
            self.output.mkdir(parents=True,exist_ok=True)
            (self.output/'runtime.json').write_text(json.dumps(report,indent=2)+'\n')
            msg=TwistStamped(); msg.header.stamp=self.node.get_clock().now().to_msg(); self.pub.publish(msg)
            self.node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',default='normal')
    parser.add_argument('--output',type=Path,default=ROOT/'validation/normal')
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    Acceptance(args.scenario,args.output).execute()
