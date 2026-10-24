#!/usr/bin/env python3
"""Own one real Gazebo + Gateway + Nav2 run; save all logs and bounded action evidence.

Run from a sourced workspace. Each goal is a map-frame pose. No velocity publisher
exists in this harness. Browser capture is optional and uses the existing frontend.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[3]

def stop(process):
    if process.poll() is not None:return
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)

def main():
    p=argparse.ArgumentParser();p.add_argument('--scenario',choices=['normal','person_crossing','obstacle_in_aisle','corridor_blocked'],default='normal')
    p.add_argument('--goal',type=float,nargs=3,action='append',required=True,metavar=('X','Y','YAW'))
    p.add_argument('--initial',type=float,nargs=3,required=True,metavar=('X','Y','YAW'))
    p.add_argument('--timeout',type=float,default=240);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--browser',action='store_true');p.add_argument('--domain',type=int,default=45)
    a=p.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    env={**os.environ,'ROS_DOMAIN_ID':str(a.domain),'GZ_PARTITION':'phase4_'+str(os.getpid()),'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
         'PATH':str(ROOT/'.tools/node_modules/.bin')+os.pathsep+os.environ['PATH']}
    children=[];logs=[];report={'scenario':a.scenario,'goals':a.goal,'passed':False,'runs':[]}
    def launch(name,cmd,cwd=ROOT,extra=None):
        f=(a.output/(name+'.log')).open('w');logs.append(f)
        proc=subprocess.Popen(cmd,cwd=cwd,env={**env,**(extra or {})},stdout=f,stderr=f,start_new_session=True);children.append(proc);return proc
    try:
        for port in ([8000,5173] if a.browser else [8000]):
            with socket.socket() as s:
                if s.connect_ex(('127.0.0.1',port))==0:raise RuntimeError(f'Port {port} occupied; stop the existing owned stack first')
        sim=launch('autonomous-demo',['ros2','launch','amr_navigation','autonomous_demo.launch.py','scenario:='+a.scenario,'initialize:=true',
            *[f'initial_{k}:={v}' for k,v in zip(['x','y','yaw'],a.initial)]])
        # Sensor audit owns no commands; startup may include renderer warm-up.
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            if sim.poll() is not None:raise RuntimeError('Simulation/navigation launch exited')
            try:
                import urllib.request
                state=json.load(urllib.request.urlopen('http://127.0.0.1:8000/api/v2/state',timeout=2))
                if state['simulation_robots'][0]['status']=='online' and state['robots'][0]['status']=='online':break
            except Exception:pass
            time.sleep(.5)
        else:raise TimeoutError('No live Gazebo world pose and odometry')
        audit=launch('sensors',['ros2','run','amr_navigation','sensor_audit','--output',str(a.output/'sensors.json')])
        if audit.wait(timeout=35)!=0:raise RuntimeError('Sensor audit failed')
        browser=None
        if a.browser:
            launch('vite',['npm','--prefix','web/digital-twin','run','dev','--','--strictPort'])
            browser=launch('browser',['npx','playwright','test','navigation.browser.ts'],ROOT/'web/digital-twin',
                {'NAV_EVIDENCE_DIR':str(a.output),'NAV_OBSERVE_SECONDS':str(min(300,int(a.timeout)))})
        for i,goal in enumerate(a.goal):
            proc=launch(f'goal-{i}',['ros2','run','amr_navigation','navigate',*map(str,goal),'--timeout',str(a.timeout),'--output',str(a.output/f'goal-{i}.json')])
            code=proc.wait(timeout=a.timeout+100)
            result=json.loads((a.output/f'goal-{i}.json').read_text());report['runs'].append({'returncode':code,'passed':result['passed']})
            if code!=0:raise RuntimeError(f'Goal {i} failed; inspect preserved action result and Nav2 log')
        if browser is not None:
            report['browser_returncode']=browser.wait(timeout=340)
            if browser.returncode!=0:raise RuntimeError('Browser synchronization assertions failed')
        report['passed']=True
    except Exception:report['error']=traceback.format_exc();raise
    finally:
        for child in reversed(children):stop(child)
        for f in logs:f.close()
        report['owned_processes_stopped']=all(c.poll() is not None for c in children)
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':main()
