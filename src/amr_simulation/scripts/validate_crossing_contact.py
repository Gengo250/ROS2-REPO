#!/usr/bin/env python3
"""Additional physical contact test against moving human_01, without world edits.

Runs with worker yielding disabled (launch test mode) so physical collision response is
still verified; yielding itself is validated separately.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import time
import argparse

from generate_warehouse import ROOT
from run_acceptance import simulation_processes, stop


def run(out=ROOT/'validation/scenario_gui'):
    assert not simulation_processes(), 'Finish the previous launch first'
    out.mkdir(parents=True,exist_ok=True)
    os.environ.update(GZ_PARTITION='warehouse_human_contact_'+str(os.getpid()),
                      ROS_DOMAIN_ID='93',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    # Import transport before message modules and after choosing the partition.
    from validate_runtime import Acceptance
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.serialized_map_pb2 import SerializedStepMap
    from google.protobuf import text_format
    import rclpy
    report={'status':'failed'}
    sim=None; a=None
    with (out/'human_01_contact.log').open('w') as log:
        try:
            # Intentional contact needs a non-yielding worker: explicit test mode, same world.
            sim=subprocess.Popen(['ros2','launch','amr_simulation','warehouse.launch.py',
                    'scenario:=person_crossing','gui:=false','human_yielding:=false'],stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True)
            a=Acceptance('person_crossing',out)
            a.gz.subscribe(Contacts,'/warehouse/contacts/human_01',lambda msg:a.on_contact('human_01',msg))
            a.until(lambda:all(a.counts[t]>=3 for t in a.topics) and 'warehouse_amr' in a.poses)
            scene=Scene()
            reply=subprocess.run(['gz','service','-s','/world/warehouse/scene/info',
                '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','5000','--req',''],
                check=True,capture_output=True,text=True,timeout=15)
            text_format.Parse(reply.stdout,scene)
            person=next(m for m in scene.model if m.name=='human_01')
            leg=next(v for link in person.link for v in link.visual if v.name=='leg_1_visual')
            leg_samples=[]
            gui_states=set()
            def observe_state(msg):
                entity=msg.state.entities.get(leg.id)
                if entity is not None:
                    for component in entity.components.values():
                        gui_states.add(component.component)
            def observe_visual(msg):
                t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
                if leg_samples and t-leg_samples[-1]['time']<.1: return
                for pose in msg.pose:
                    if pose.id==leg.id:
                        leg_samples.append({'time':t,'quaternion_y':pose.orientation.y,
                                            'position_x':pose.position.x})
            a.gz.subscribe(Pose_V,'/world/warehouse/pose/info',observe_visual)
            a.gz.subscribe(SerializedStepMap,'/world/warehouse/state',observe_state)
            a.wait(12)
            assert len(leg_samples)>10, 'No visual pose evidence'
            swing=max(s['quaternion_y'] for s in leg_samples)-min(s['quaternion_y'] for s in leg_samples)
            assert swing>.2, 'Leg does not swing while walking'
            assert len(gui_states)>10, 'Animated visual changes are not sent to the GUI'
            report['walking_leg_quaternion_y_span']=swing
            report['distinct_gui_visual_states']=len(gui_states)
            a.set_pose([-8.7,0,.015,0,0,-math.pi/2])
            a.until(lambda:a.contacts['human_01']>0,180)
            records=[]
            start=a.now()
            while a.now()-start<4:
                a.drive(0,0,.1)
                records.append({'time':a.now(),'robot':a.robot(),'human':a.poses['human_01']})
            gaps=[abs(x['human']['position']['y']-x['robot']['position']['y']) for x in records]
            signs=[math.copysign(1,x['human']['position']['y']-x['robot']['position']['y']) for x in records]
            assert min(gaps)>.60, min(gaps)
            assert len(set(signs))==1, 'Moving person traversed the AMR'
            # Observe settled contact, not just a commanded zero speed: the
            # displacement-driven gait must stop when the body cannot advance.
            a.drive(0,0,3)
            stopped_start=a.now()
            start_pose=a.poses['human_01']
            a.drive(0,0,1.5)
            end_pose=a.poses['human_01']
            displacement=math.hypot(end_pose['position']['x']-start_pose['position']['x'],
                                    end_pose['position']['y']-start_pose['position']['y'])
            stopped=[s for s in leg_samples if s['time']>=stopped_start]
            assert len(stopped)>5
            assert displacement<.02, 'Contact did not stop human body'
            assert max(abs(s['quaternion_y']) for s in stopped)<.02, 'Walking continues at rest'
            report['stopped_displacement_m']=displacement
            report['visual_pose_samples']=leg_samples
            report.update(status='passed',contact_events=a.contacts['human_01'],min_y_gap_m=min(gaps),
                          samples=records,manifest_sha256=a.manifest_digest,
                          method='AMR stopped across human_01 route; physical force controller active')
        finally:
            if a is not None:
                a.node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
            stop(sim)
            deadline=time.monotonic()+10
            while simulation_processes() and time.monotonic()<deadline:time.sleep(.2)
            report['remaining_processes']=simulation_processes()
            (out/'human_01_contact.json').write_text(json.dumps(report,indent=2)+'\n')
    assert not report['remaining_processes']
    print('MOVING HUMAN CONTACT PASS',report['contact_events'],report['min_y_gap_m'],flush=True)


def run_yield(out=ROOT/'validation/scenario_gui'):
    """Yielding enabled (production default): human_01 must stop before the AMR placed
    across its route, without contact, with its gait at rest, then resume once the AMR
    leaves. Physical forces only; the worker is never repositioned."""
    assert not simulation_processes(), 'Finish the previous launch first'
    out.mkdir(parents=True,exist_ok=True)
    os.environ.update(GZ_PARTITION='warehouse_human_yield_'+str(os.getpid()),
                      ROS_DOMAIN_ID='92',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    from validate_runtime import Acceptance
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from google.protobuf import text_format
    import rclpy
    report={'status':'failed'}
    sim=None; a=None
    with (out/'human_01_yield.log').open('w') as log:
        try:
            sim=subprocess.Popen(['ros2','launch','amr_simulation','warehouse.launch.py',
                    'scenario:=person_crossing','gui:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            a=Acceptance('person_crossing',out)
            a.gz.subscribe(Contacts,'/warehouse/contacts/human_01',lambda msg:a.on_contact('human_01',msg))
            states=[]
            def on_state(m):
                state=json.loads(m.data)
                h=a.poses.get('human_01',{}).get('position')
                state['worker_xy']=[h['x'],h['y']] if h else None
                states.append(state)
            a.gz.subscribe(StringMsg,'/warehouse/humans/human_01/yield',on_state)
            a.until(lambda:all(a.counts[t]>=3 for t in a.topics) and 'warehouse_amr' in a.poses and states)
            reply=subprocess.run(['gz','service','-s','/world/warehouse/scene/info','--reqtype','gz.msgs.Empty',
                '--reptype','gz.msgs.Scene','--timeout','5000','--req',''],check=True,capture_output=True,text=True,timeout=15)
            scene=Scene(); text_format.Parse(reply.stdout,scene)
            person=next(m for m in scene.model if m.name=='human_01')
            leg=next(v for link in person.link for v in link.visual if v.name=='leg_1_visual')
            legs=[]
            def observe_visual(msg):
                t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
                for pose in msg.pose:
                    if pose.id==leg.id and (not legs or t-legs[-1]['time']>=.1):
                        legs.append({'time':t,'quaternion_y':pose.orientation.y})
            a.gz.subscribe(Pose_V,'/world/warehouse/pose/info',observe_visual)
            def worker():
                h=a.poses['human_01']['position']; return (h['x'],h['y'])
            def speed(seconds):
                p0=worker(); t0=a.now(); a.wait(seconds); p1=worker()
                return math.dist(p0,p1)/max(a.now()-t0,1e-6)
            report['free_walking_speed_mps']=speed(3)
            a.require(report['free_walking_speed_mps']>.8,'unobstructed worker keeps configured walking speed')
            a.set_pose([-8.7,0,.015,0,0,-math.pi/2])
            blocked_start=a.now()
            a.until(lambda:states and states[-1]['yielding'],120)
            a.require(True,'worker enters yielding before the AMR envelope')
            report['yield_start_envelope_distance_m']=states[-1]['envelope_distance_m']
            # The yield cap is braking-consistent: a yield that starts far away keeps the worker
            # walking to its stop point short of the envelope, where it then rests. Wait for the rest.
            rest_deadline=a.now()+30
            while speed(1)>=.05:
                a.require(a.now()<rest_deadline,'yielding worker comes to rest within 30 s')
            report['rest_envelope_distance_m']=states[-1]['envelope_distance_m']
            a.require(report['rest_envelope_distance_m']>.28,'worker rests clear of the AMR envelope (> worker radius)')
            report['yield_wait_speed_mps']=speed(2)
            rest=[s for s in legs if s['time']>=a.now()-1.5]
            a.require(report['yield_wait_speed_mps']<.05,'yielding worker is stationary')
            a.require(len(rest)>5 and max(abs(s['quaternion_y']) for s in rest)<.02,'gait at rest while yielding')
            report['min_envelope_distance_m']=min(s['envelope_distance_m'] for s in states if s['sim_time']>=0)
            a.require(a.contacts['human_01']==0,'no AMR contact while the worker yields')
            a.set_pose([-11.0,0,.015,0,0,0])
            a.until(lambda:not states[-1]['yielding'],60)
            a.wait(1.5)
            # Gait is judged in the same window as the resumed speed: a fixed later window can fall
            # into the worker's normal pause at the end of its route segment.
            walking_start=a.now()
            report['resumed_speed_mps']=speed(2)
            a.require(report['resumed_speed_mps']>.5,'worker resumes its route after the AMR leaves')
            moving=[s for s in legs if s['time']>=walking_start]
            a.require(max(s['quaternion_y'] for s in moving)-min(s['quaternion_y'] for s in moving)>.2,'gait resumes with walking')
            a.require(a.contacts['human_01']==0,'zero AMR contacts across the whole yield test')
            report.update(status='passed',yield_events=states[-1]['events'],total_yield_s=states[-1]['total_yield_s'],
                          reversals=states[-1]['reversals'],blocked_for_sim_s=a.now()-blocked_start,
                          contacts=a.contacts['human_01'],yield_states=states[::5],manifest_sha256=a.manifest_digest,
                          method='AMR placed across human_01 route with yielding enabled; physical force controller active')
        finally:
            # Keep the yield timeline even when an assertion fails (diagnosis).
            report.setdefault('yield_states',states[::2] if 'states' in locals() else [])
            if a is not None:
                a.node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
            stop(sim)
            deadline=time.monotonic()+10
            while simulation_processes() and time.monotonic()<deadline:time.sleep(.2)
            report['remaining_processes']=simulation_processes()
            (out/'human_01_yield.json').write_text(json.dumps(report,indent=2)+'\n')
    assert not report['remaining_processes']
    print('MOVING HUMAN YIELD PASS',report.get('min_envelope_distance_m'),report.get('total_yield_s'),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'validation/scenario_gui')
    parser.add_argument('--mode',choices=['contact','yield'],default='contact',
                        help='contact: yielding disabled, physical collision response; yield: production yielding')
    args=parser.parse_args()
    (run if args.mode=='contact' else run_yield)(args.output)
