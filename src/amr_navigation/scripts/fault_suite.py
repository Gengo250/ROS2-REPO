#!/usr/bin/env python3
"""Bounded real faults, each with a fresh owned stack and no manual cmd publisher."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import traceback
import urllib.request
import psutil

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('navigation_integration',Path(__file__).with_name('integration.py'))
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
CASES=['cancel','outside-map','occupied-goal','timeout','pause','localization-loss','controller-inactive','clock-bridge-loss','gazebo-reset']


def run_case(name,output):
    output.mkdir(parents=True,exist_ok=False)
    env={**os.environ,'ROS_DOMAIN_ID':'46','GZ_PARTITION':'phase4_fault_'+name+'_'+str(os.getpid()),'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
    children=[];files=[];suspended=[];report={'case':name,'passed':False}
    def launch(label,cmd):
        stream=(output/(label+'.log')).open('w');files.append(stream)
        process=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=stream,stderr=stream,start_new_session=True)
        children.append(process);return process
    def child(executable,exclude=None):
        matches=[]
        for process in psutil.Process(sim.pid).children(recursive=True):
            try:
                cmd=process.cmdline()
                if cmd and Path(cmd[0]).name==executable and not (exclude and any(exclude in arg for arg in cmd)):matches.append(process)
            except psutil.NoSuchProcess:pass
        if len(matches)!=1:raise RuntimeError(f'Expected one owned {executable}, found {len(matches)}')
        return matches[0]
    try:
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1',8000))==0:raise RuntimeError('Gateway port occupied; refuse duplicate stack')
        sim=launch('demo',['ros2','launch','amr_navigation','autonomous_demo.launch.py','initialize:=true','initial_x:=0','initial_y:=0','initial_yaw:=0'])
        audit=launch('runtime',['ros2','run','amr_navigation','runtime_audit','--timeout','120','--output',str(output/'runtime.json')])
        if audit.wait(timeout=130)!=0:raise RuntimeError('Runtime audit failed')
        xyz=[8,0,0];options=['--cancel-after','12'];timeout=75
        if name=='cancel':options=['--cancel-after','4']
        if name=='outside-map':xyz=[999,999,0];options=['--server-validation'];timeout=120
        if name=='occupied-goal':xyz=[10.979,-9.927,0];options=['--server-validation'];timeout=120
        if name=='timeout':timeout=2;options=[]
        process=launch('goal',['ros2','run','amr_navigation','navigate',*map(str,xyz),'--timeout',str(timeout),'--output',str(output/'goal.json'),*options])
        if name in ['pause','localization-loss','controller-inactive','clock-bridge-loss','gazebo-reset']:
            # Inject only after the real drive is moving, never during DDS startup.
            deadline=time.monotonic()+45
            while time.monotonic()<deadline and process.poll() is None:
                try:
                    state=json.load(urllib.request.urlopen('http://127.0.0.1:8000/api/v2/state',timeout=2))
                    robot=state['robots'][0]
                    if robot['status']=='online' and abs(robot['velocity']['linear']['x'])>.10:
                        report['velocity_before_fault']=robot['velocity'];break
                except (OSError,KeyError,IndexError):pass
                time.sleep(.1)
            else:raise RuntimeError('Goal did not produce real odometry motion before fault injection')
            report['fault_wall_time']=time.time()
            if name in ['pause','gazebo-reset']:
                subprocess.run(['gz','service','-s','/world/warehouse/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','reset: {all: true}' if name=='gazebo-reset' else 'pause: true'],env=env,check=True,timeout=5,stdout=subprocess.DEVNULL)
            elif name=='controller-inactive':
                fault=launch('deactivate',['ros2','service','call','/controller_manager/switch_controller','controller_manager_msgs/srv/SwitchController',json.dumps({'deactivate_controllers':['diff_drive_controller'],'strictness':2,'activate_asap':True,'timeout':{'sec':5,'nanosec':0}})])
                if fault.wait(timeout=15)!=0:raise RuntimeError('Controller deactivation failed')
                if 'ok=True' not in (output/'deactivate.log').read_text():raise RuntimeError('Controller manager did not confirm deactivation')
            else:
                target=child('amcl') if name=='localization-loss' else child('parameter_bridge','__node:=warehouse_bridge')
                target.send_signal(signal.SIGSTOP);suspended.append(target)
        report['goal_exit']=process.wait(timeout=timeout+100)
        result=json.loads((output/'goal.json').read_text())
        if name in ['pause','clock-bridge-loss','gazebo-reset']:
            stats=subprocess.run(['gz','topic','-e','-t','/world/warehouse/stats','-n','1'],env=env,capture_output=True,text=True,timeout=10)
            (output/'gazebo-stats.txt').write_text(stats.stdout+stats.stderr)
            report['gazebo_confirmed_paused']='paused: true' in stats.stdout
        report['result']={key:result.get(key) for key in ['accepted','status','error_code','stopped','error','timed_out','world_travel_distance_m']}
        rows=result['trajectory'];tail=[r for r in rows if rows[-1]['wall']-r['wall']<=2 and r.get('world')]
        displacement=sum(math.dist(a['world'][:2],b['world'][:2]) for a,b in zip(tail,tail[1:]))
        command_zero=bool(tail) and all(r.get('command') is not None and max(abs(v) for v in r['command'])<.001 for r in tail)
        report.update(final_two_seconds_world_travel_m=displacement,zero_commands=command_zero)
        stationary=len(tail)>=4 and displacement<.01
        if name in ['outside-map','occupied-goal']:valid=result.get('status')==6
        elif name=='timeout':valid=result.get('timed_out') and result.get('status')==5
        else:valid=result.get('status') in [5,6]
        report['passed']=bool(valid and stationary and command_zero and report.get('gazebo_confirmed_paused',True))
        # Paused/stalled odometry is never accepted as proof of zero wheel speed.
        report['physical_stop_basis']='Independent world trajectory stationary over final two wall seconds; paused cases remain paused until owned cleanup.'
    except Exception:report['error']=traceback.format_exc()
    finally:
        # Stop the owned simulator while fault remains in force. Resume stopped
        # children only so shutdown can reap them; do not resume physics.
        for process in suspended:
            try:process.send_signal(signal.SIGCONT)
            except psutil.NoSuchProcess:pass
        for process in reversed(children):helpers.stop(process)
        for stream in files:stream.close()
        report['owned_processes_stopped']=all(p.poll() is not None for p in children)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=CASES,action='append');args=parser.parse_args()
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    reports=[]
    for name in args.case or CASES:
        result=run_case(name,args.output/name);reports.append(result)
        print(json.dumps(result),flush=True)
    passed=all(r['passed'] for r in reports)
    (args.output/'report.json').write_text(json.dumps({'passed':passed,'cases':reports},indent=2)+'\n')
    if not passed:raise SystemExit(1)
if __name__=='__main__':main()
