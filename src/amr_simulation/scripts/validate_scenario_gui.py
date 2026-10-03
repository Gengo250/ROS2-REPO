#!/usr/bin/env python3
"""Real GUI evidence, Transport state and ROS samples for four isolated launches.

Camera movement uses Gazebo's GUI service only; entities are never moved for
screenshots. Xvfb is used because Wayland window capture returns black here.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from generate_warehouse import ROOT, SCENARIOS
from run_acceptance import stop

CAPTURE = r'''
from PyQt5.QtGui import QGuiApplication
import subprocess,re,sys
app=QGuiApplication([])
s=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
lines=[l for l in s.splitlines() if 'Gazebo Sim' in l and re.search(r'\d{3,}x\d{3,}',l)]
assert lines,s
wid=int(re.search('0x[0-9a-f]+',lines[0])[0],16)
im=app.primaryScreen().grabWindow(wid)
assert im.width()>900 and im.save(sys.argv[1])
image=im.toImage()
colours={image.pixel(x,y) for x in range(0,image.width(),8) for y in range(110,image.height(),8)}
assert len(colours)>200, 'GUI scene not yet rendered'
'''


def service(name, reqtype, reptype, request):
    r = subprocess.run(['gz','service','-s',name,'--reqtype',reqtype,
        '--reptype',reptype,'--timeout','5000','--req',request],
        capture_output=True,text=True,timeout=12)
    assert r.returncode == 0, r.stderr
    return r.stdout


def processes():
    import psutil
    found = []
    for p in psutil.process_iter(['pid','ppid','cmdline']):
        args = p.info['cmdline'] or []
        if not args:
            continue
        if args[0].startswith('gz sim') or (len(args)>1 and args[1]=='sim') or any(
                '/ros_gz_bridge/parameter_bridge' in a or '/ros_gz_sim/create' in a for a in args):
            found.append(p.info)
    return found


def scene(scenario, out):
    import rclpy
    from gz.transport13 import Node  # Load transport before protobuf descriptors.
    from google.protobuf.json_format import MessageToDict
    from google.protobuf import text_format
    from gz.msgs10.scene_pb2 import Scene
    from validate_runtime import Acceptance, distance, yaw, delta_angle
    a = Acceptance(scenario, out)
    report = {'scenario':scenario,'captures':{},'status':'failed','gui_renderer':'ogre',
              'manifest_sha256':hashlib.sha256((ROOT/'config/generated_manifest.json').read_bytes()).hexdigest()}
    def pause(value):
        assert 'data: true' in service('/world/warehouse/control','gz.msgs.WorldControl',
                'gz.msgs.Boolean','pause: '+str(value).lower())
    def view(xyz, target):
        dx,dy,dz = [b-a for a,b in zip(xyz,target)]
        pitch = -math.atan2(dz,math.hypot(dx,dy)); angle = math.atan2(dy,dx)
        cy,sy,cp,sp = math.cos(angle/2),math.sin(angle/2),math.cos(pitch/2),math.sin(pitch/2)
        request = ('pose {position {x: %s y: %s z: %s} orientation {x: %s y: %s z: %s w: %s}}'
                   % (*xyz,-sy*sp,cy*sp,sy*cp,cy*cp))
        assert 'data: true' in service('/gui/move_to/pose','gz.msgs.GUICamera','gz.msgs.Boolean',request)
        time.sleep(1.4)
    def capture(name):
        pause(True)
        time.sleep(.3)
        deadline=time.monotonic()+90
        while True:
            attempt=subprocess.run([sys.executable,'-c',CAPTURE,str(out/(name+'.png'))],capture_output=True,text=True,timeout=15)
            if attempt.returncode==0:
                break
            assert time.monotonic()<deadline,attempt.stderr
            time.sleep(3)
        report['captures'][name]={'sim_time':a.now(),'poses':dict(a.poses)}
        pause(False)
    try:
        a.until(lambda:all(a.counts[t]>=3 for t in a.topics) and 'warehouse_amr' in a.poses,120)
        a.wait(1)
        state = Scene()
        text_format.Parse(service('/world/warehouse/scene/info','gz.msgs.Empty','gz.msgs.Scene',''),state)
        models = {m.name:MessageToDict(m,preserving_proto_field_name=True) for m in state.model}
        report['entities'] = {k:{f:v for f,v in m.items() if f in ('id','name','pose')}
                              for k,m in models.items()}
        assert 'warehouse_amr' in models
        assert ('corridor_barrier' in models) == (scenario=='corridor_blocked')
        assert ('aisle_pallet' in models) == (scenario=='obstacle_in_aisle')
        report['spawn_actual'] = a.robot()
        assert math.hypot(a.robot()['position']['x']+11,a.robot()['position']['y'])<.03
        time.sleep(3)
        capture(scenario+'_overview')
        # Measure actual receipt rates as well as timestamp validity.
        hz_jobs=[]
        if scenario=='normal':
            for topic in ('/odom','/scan','/imu/data','/camera/image_raw',
                          '/camera/depth/image_raw','/camera/points'):
                log=(out/('hz'+topic.replace('/','_')+'.txt')).open('w')
                job=subprocess.Popen(['ros2','topic','hz',topic],stdout=log,stderr=subprocess.STDOUT)
                hz_jobs.append((job,log))
        start, counts, wall = a.now(),dict(a.counts),time.monotonic()
        try:
            a.wait(5)
        finally:
            for job,log in hz_jobs:
                job.send_signal(signal.SIGINT)
                try:job.wait(timeout=5)
                except subprocess.TimeoutExpired:job.terminate();job.wait(timeout=5)
                log.close()
        report['rates'] = {t:{'samples':a.counts[t]-counts[t],
            'hz_sim':(a.counts[t]-counts[t])/(a.now()-start),
            'hz_wall':(a.counts[t]-counts[t])/(time.monotonic()-wall),
            'frame':a.received[t].header.frame_id} for t in a.topics}
        assert all(v['samples']>2 for v in report['rates'].values())
        # Fixed spawn and fixed GUI view: the existing AMR has a white deck
        # against a grey floor here. This catches the observed GUI-only missing
        # robot even when Transport poses and all ROS sensors are healthy.
        # Keep this supplementary to opening every image for visual review.
        from PIL import Image
        view([-14,-4,5],[-10,0,.1])
        capture(scenario+'_amr')
        roi=Image.open(out/(scenario+'_amr.png')).convert('RGB').crop((600,480,830,710))
        white=sum(min(p)>220 for p in roi.getdata())
        report['amr_gui_white_deck_pixels']=white
        assert white>1000, f'AMR deck missing from fixed GUI view: {white} pixels'
        if scenario=='normal':
            a.capture('/warehouse/camera_01/image_raw','warehouse_camera')
            capture('amr_before_motion')
            before, odom0 = a.robot(),str(a.received['/odom'])
            with (out/'cmd_vel_cli.txt').open('w') as command_log:
                command=subprocess.Popen(['ros2','topic','pub','--rate','10','/cmd_vel',
                    'geometry_msgs/msg/TwistStamped',
                    '{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}'],
                    stdout=command_log,stderr=subprocess.STDOUT)
                try:
                    # DDS discovery is wall-clock startup, not commanded motion.
                    a.until(lambda:distance(before,a.robot())>.03,30)
                    a.wait(7)
                finally:
                    command.send_signal(signal.SIGINT);command.wait(timeout=10)
            a.drive(0,0,.7)
            capture('amr_after_motion')
            report['straight']={'start':before,'end':a.robot(),'metres':distance(before,a.robot()),
                                'odom_before':odom0,'odom_after':str(a.received['/odom'])}
            assert report['straight']['metres']>1.2
            before=a.robot(); a.drive(0,.3,3); a.drive(0,0,.8)
            report['turn']={'start':before,'end':a.robot(),'radians':delta_angle(yaw(a.robot()),yaw(before))}
            assert report['turn']['radians']>.7
            before=a.robot(); a.drive(.15,.2,4); a.drive(0,0,.8)
            report['curve']={'start':before,'end':a.robot(),'metres':distance(before,a.robot())}
            assert report['curve']['metres']>.4
            before=a.robot(); a.drive(0,0,2)
            report['stop_drift_m']=distance(before,a.robot()); assert report['stop_drift_m']<.02
        elif scenario=='person_crossing':
            view([-14,-5,9],[-9,0,.3])
            a.until(lambda:a.poses['human_01']['position']['y'] < -1.4,150)
            capture('person_crossing_t0')
            a.until(lambda:a.poses['human_01']['position']['y'] > 1.4,150)
            capture('person_crossing_t1')
            # Directed moving-human sensor probe, after all GUI captures.
            a.set_pose([-10.5,0,.015,0,0,0])
            a.until(lambda:abs(a.poses['human_01']['position']['y'])<.12,150)
            scan=a.received['/scan']; mid=len(scan.ranges)//2
            report['crossing_lidar']={'human':a.poses['human_01'], 'robot':a.robot(),
                                     'nearest_centre_m':min(scan.ranges[mid-8:mid+9])}
            assert .7<report['crossing_lidar']['nearest_centre_m']<1.8
        else:
            view([-4.5,-10,6],[-4.5,-4.5,.1])
            capture(scenario+'_closeup')
        report['topic_samples']=dict(a.counts)
        report['human_trajectories']=a.history
        report['status']='passed'
    finally:
        (out/(scenario+'_gui.json')).write_text(json.dumps(report,indent=2)+'\n')
        a.node.destroy_node(); rclpy.shutdown()


def run(scenarios, out):
    out.mkdir(parents=True,exist_ok=True)
    xvfb=None
    try:
        with (out/'xvfb.log').open('w') as xlog:
            xvfb=subprocess.Popen(['Xvfb','-displayfd','1','-screen','0','1680x1080x24','-nolisten','tcp'],
                    stdout=subprocess.PIPE,stderr=xlog,text=True,start_new_session=True)
            display=xvfb.stdout.readline().strip(); assert display.isdigit()
        env={**os.environ,'DISPLAY':':'+display,'QT_QPA_PLATFORM':'xcb'}
        env.pop('WAYLAND_DISPLAY',None)
        # Do not force software rendering on the EGL sensor server; the GUI
        # selects Xvfb's Mesa renderer independently through its display.
        env.pop('LIBGL_ALWAYS_SOFTWARE',None)
        for s in scenarios:
            before=processes(); assert not before, before
            env.update(GZ_PARTITION='warehouse_gui_'+str(os.getpid())+'_'+s,
                       ROS_DOMAIN_ID='92',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
            lifecycle={'before':before,'partition':env['GZ_PARTITION']}
            with (out/(s+'.log')).open('w') as log:
                p=subprocess.Popen(['ros2','launch','amr_simulation','warehouse.launch.py',
                        'scenario:='+s,'gui_renderer:=ogre'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    subprocess.run([sys.executable,__file__,'--observe',s,'--output',str(out)],env=env,check=True,timeout=500)
                    assert p.poll() is None
                    lifecycle['running']=processes()
                finally:
                    # Verify the launch itself reaps children; no group cleanup
                    # until evidence has been recorded if the assertion fails.
                    p.send_signal(signal.SIGINT)
                    try:p.wait(timeout=25)
                    except subprocess.TimeoutExpired:pass
                    time.sleep(1)
                    lifecycle['after_sigint']=processes()
                    (out/(s+'_lifecycle.json')).write_text(json.dumps(lifecycle,indent=2)+'\n')
                    stop(p)
                assert not lifecycle['after_sigint'], lifecycle
            print('GUI+LIFECYCLE PASS',s,flush=True)
    finally:
        stop(xvfb)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',action='append',choices=SCENARIOS)
    parser.add_argument('--observe',choices=SCENARIOS)
    parser.add_argument('--output',type=Path,default=ROOT/'validation/scenario_gui')
    args=parser.parse_args()
    if args.observe:scene(args.observe,args.output)
    else:run(args.scenario or SCENARIOS,args.output)
