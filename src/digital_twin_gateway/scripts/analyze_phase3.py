#!/usr/bin/env python3
"""Numerically compare real Gazebo, Gateway and browser samples at their timestamps."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics


def distance(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
def interpolate(a,b,alpha): return [x+(y-x)*alpha for x,y in zip(a,b)]
def slerp(a,b,alpha):
    dot=sum(x*y for x,y in zip(a,b))
    if dot < 0: b=[-x for x in b]; dot=-dot
    if dot > .999999:
        result=interpolate(a,b,alpha); norm=math.sqrt(sum(x*x for x in result))
        return [x/norm for x in result]
    angle=math.acos(min(1,dot))
    return [(x*math.sin((1-alpha)*angle)+y*math.sin(alpha*angle))/math.sin(angle) for x,y in zip(a,b)]
def quaternion_error(a,b): return min(distance(a,b),distance(a,[-v for v in b]))
def summary(values):
    ordered=sorted(values)
    return {'count':len(values),'mean':statistics.mean(values),'p95':ordered[int(.95*(len(ordered)-1))],'max':max(values)} if values else None


def analyze(directory):
    raw=json.loads((directory/'gazebo.json').read_text())
    browser=json.loads((directory/'workers-browser.json').read_text())
    gateway=json.loads((directory/'gateway.json').read_text())
    by_time=defaultdict(list)
    for sample in raw['samples']: by_time[round(sample['time']*1e9)].append(sample)
    def source(timestamp,wall):
        matches=by_time.get(round(timestamp*1e9),[])
        return min(matches,key=lambda s:abs(s['wall']-wall)) if matches else None
    pose_errors=[]; quaternion_errors=[]; latencies=[]; rendered_errors=[]; rendered_quaternions=[]; visual_errors=[]; delays=[]; seen=set()
    first=browser['samples'][0]['metrics']['rendered_wall_ms']/1000
    last=browser['samples'][-1]['metrics']['rendered_wall_ms']/1000
    for frame in browser['frames']:
        wall=frame['browser_received_at']/1000
        if not first <= wall <= last: continue
        for entity in frame['simulation_robots']+frame['humans']:
            if entity['status']!='online': continue
            sample=source(entity['source_timestamp'],wall)
            if sample is None or entity['id'] not in sample['poses']: continue
            actual=sample['poses'][entity['id']]
            pose_errors.append(distance(list(entity['pose']['position'].values()),actual['position']))
            quaternion_errors.append(quaternion_error(list(entity['pose']['orientation'].values()),actual['quaternion']))
            key=(entity['id'],entity['source_timestamp'])
            if key not in seen:
                latencies.append((wall-sample['wall'])*1000); seen.add(key)
    for frame in browser['samples']:
        wall=frame['metrics']['rendered_wall_ms']/1000
        for identity in ('amr_01','human_01','human_02','human_03'):
            rendered=frame[identity]
            a=source(rendered['from_time'],wall); b=source(rendered['to_time'],wall)
            if a is None or b is None: continue
            alpha=rendered['alpha']; pa=a['poses'][identity]; pb=b['poses'][identity]
            rendered_errors.append(distance(rendered['position'],interpolate(pa['position'],pb['position'],alpha)))
            rendered_quaternions.append(quaternion_error(rendered['quaternion'],slerp(pa['quaternion'],pb['quaternion'],alpha)))
            delays.append((wall-(a['wall']+(b['wall']-a['wall'])*alpha))*1000)
            for name,pose in frame['visuals'].items():
                if not name.startswith(identity+'/') or name not in a['poses'] or name not in b['poses']: continue
                va=a['poses'][name]; vb=b['poses'][name]
                visual_errors.append(distance(pose['position'],interpolate(va['position'],vb['position'],alpha)))
                rendered_quaternions.append(quaternion_error(pose['quaternion'],slerp(va['quaternion'],vb['quaternion'],alpha)))
    moving={}
    for identity in ('human_01','human_02','human_03'):
        poses=[s['poses'][identity] for s in raw['samples'] if first<=s['wall']<=last and identity in s['poses']]
        moving[identity]={'max_distance_from_first_m':max(distance(poses[0]['position'],p['position']) for p in poses),
                          'max_quaternion_change':max(quaternion_error(poses[0]['quaternion'],p['quaternion']) for p in poses)}
    rates=[s for s in gateway if first<=s['wall']<=last]
    memory=json.loads((directory/'memory.json').read_text())
    peak={}
    for sample in memory:
        for p in sample['rss']: peak[p['name']]=max(peak.get(p['name'],0),p['bytes'])
    stats=[s['rtf'] for s in raw['stats'] if first<=s['wall']<=last and not s['paused']]
    raw_window=[s for s in raw['samples'] if first<=s['wall']<=last]
    motion=json.loads((directory/'motion.json').read_text())
    amr=json.loads((directory/'amr-browser.json').read_text())
    amr_errors=[]; amr_rotation_errors=[]
    # The controlled-motion test precedes the workers/reset tests. Select that
    # epoch explicitly because simulation timestamps are reused after reset.
    motion_sources={round(s['time']*1e9):s for s in raw['samples'] if s['wall']<first}
    for frame in [amr['before'],*amr['samples'],amr['after']]:
        rendered=frame['amr_01']
        a=motion_sources.get(round(rendered['from_time']*1e9))
        b=motion_sources.get(round(rendered['to_time']*1e9))
        if a is None or b is None or 'amr_01' not in a['poses'] or 'amr_01' not in b['poses']: continue
        pa,pb=a['poses']['amr_01'],b['poses']['amr_01']
        amr_errors.append(distance(rendered['position'],interpolate(pa['position'],pb['position'],rendered['alpha'])))
        amr_rotation_errors.append(quaternion_error(rendered['quaternion'],slerp(pa['quaternion'],pb['quaternion'],rendered['alpha'])))
    topic='/digital_twin/simulation/poses'
    def received(sample):
        return next(t['messages_received'] for t in sample['state']['connection']['topics'] if t['topic']==topic)
    result={'gateway_vs_gazebo_position_m':summary(pose_errors),'gateway_vs_gazebo_quaternion_l2':summary(quaternion_errors),
        'render_vs_interpolated_gazebo_position_m':summary(rendered_errors),
        'render_vs_interpolated_gazebo_quaternion_l2':summary(rendered_quaternions),
        'visual_local_position_error_m':summary(visual_errors),'gazebo_receipt_to_browser_receipt_ms':summary(latencies),
        'gazebo_receipt_to_render_ms':summary(delays),'human_motion':moving,
        'fps':summary([s['metrics']['fps'] for s in browser['samples']]),'metrics':browser['metrics'],
        'graphics':browser['graphics'],'simulation_rtf':summary(stats),'process_peak_rss_bytes':peak,
        'gateway_hz':(len(rates)-1)/(rates[-1]['wall']-rates[0]['wall']),
        'gazebo_pose_hz_wall':(len(raw_window)-1)/(raw_window[-1]['wall']-raw_window[0]['wall']),
        'mean_v2_bytes':statistics.mean(s['bytes'] for s in rates),
        'ros_pose_hz_wall':(received(rates[-1])-received(rates[0]))/(rates[-1]['wall']-rates[0]['wall']),
        'controlled_amr':{'odometry_displacement_x_m':motion['displacement_m'],'stopped':motion['stopped'],
            'world_displacement_m':distance(amr['before']['amr_01']['position'],amr['after']['amr_01']['position']),
            'quaternion_change_l2':quaternion_error(amr['before']['amr_01']['quaternion'],amr['after']['amr_01']['quaternion']),
            'render_position_error_m':summary(amr_errors),'render_quaternion_error_l2':summary(amr_rotation_errors)}}
    assert motion['stopped'] and len(amr_errors)>=5 and max(amr_errors)<1e-6
    assert max(amr_rotation_errors)<1e-6
    assert len(pose_errors)>50 and max(pose_errors)<1e-12
    assert len(rendered_errors)>10 and max(rendered_errors)<1e-6
    assert len(visual_errors)>50 and max(visual_errors)<1e-6
    assert max(rendered_quaternions)<1e-6
    assert moving['human_03']['max_distance_from_first_m']<1e-9
    for name in ('human_01','human_02'): assert moving[name]['max_distance_from_first_m']>.05
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);args=parser.parse_args()
    result={d.name:analyze(d) for d in args.directory.iterdir() if d.is_dir() and (d/'workers-browser.json').exists()}
    assert result
    (args.directory/'numerical-validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
