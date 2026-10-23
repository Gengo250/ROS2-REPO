#!/usr/bin/env python3
"""Owned real Gazebo/ROS/browser validation, four scenarios and simulator restart."""
import argparse
import asyncio
from collections import deque
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import traceback
import httpx
import psutil
import websockets

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('phase1_helpers', ROOT/'src/digital_twin_gateway/scripts/validate_integration.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
analysis_spec = importlib.util.spec_from_file_location('phase3_analysis', Path(__file__).with_name('analyze_phase3.py'))
analysis = importlib.util.module_from_spec(analysis_spec)
analysis_spec.loader.exec_module(analysis)


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--scenarios', nargs='+', default=['normal','person_crossing','obstacle_in_aisle','corridor_blocked'])
    parser.add_argument('--headed', action='store_true', help='Use the display GPU instead of headless Chromium')
    args = parser.parse_args()
    output = (args.output or ROOT/'docs/digital-twin/evidence-phase3'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {'baseline':'00439c9','headed':args.headed,'scenarios':{},'passed':False,'shutdown':{}}
    processes, logs, numerical = [], [], {}
    observer = None
    states, memory, directory = None, [], None
    env = {**os.environ,'ROS_DOMAIN_ID':'214','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
           'GZ_PARTITION':'phase3_'+str(os.getpid()),'PATH':str(ROOT/'.tools/node_modules/.bin')+os.pathsep+os.environ['PATH']}
    def launch(name, command, directory, extra=None, cwd=ROOT):
        log = (directory/(name+'.log')).open('w'); logs.append(log)
        p = subprocess.Popen(command, cwd=cwd, env={**env,**(extra or {})}, stdout=log, stderr=log,start_new_session=True)
        processes.append((name,p)); return p
    async def stop(name, process):
        record={}
        await asyncio.to_thread(helpers.stop, process, record)
        report['shutdown'][name]=record
    async def ready(simulation):
        async with httpx.AsyncClient(timeout=2) as client:
            for _ in range(240):
                if simulation.poll() is not None:
                    raise RuntimeError('Simulation launch exited before telemetry became ready; inspect its log')
                try:
                    s=(await client.get('http://127.0.0.1:8000/api/v2/state')).json()
                    if all(e['status']=='online' for e in s['simulation_robots']+s['humans']) and s['robots'][0]['status']=='online': return s
                except (httpx.HTTPError,KeyError): pass
                await asyncio.sleep(.5)
            raise TimeoutError('Real ground truth/odometry did not become ready')
    try:
        for port in (8000,5173):
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1',port))==0: raise RuntimeError(f'Port {port} already occupied')
        web=launch('vite',['npm','--prefix','web/digital-twin','run','dev','--','--strictPort'],output)
        for scenario in args.scenarios:
            directory=output/scenario; directory.mkdir()
            extra={'TWIN_SCENARIO':scenario,'TWIN_SCENARIO_SOURCE':'environment'}
            sim=launch(scenario+'-simulation',['ros2','launch','amr_simulation','warehouse.launch.py','gui:=false','scenario:='+scenario],directory)
            adapter=launch(scenario+'-adapter',['ros2','run','digital_twin_gateway','simulation_pose_adapter'],directory,extra)
            gateway=launch(scenario+'-gateway',['ros2','run','digital_twin_gateway','gateway'],directory,extra)
            capture=launch(scenario+'-capture',[str(ROOT/'.venv/bin/python'),str(ROOT/'src/digital_twin_gateway/scripts/capture_simulation.py'),'--output',str(directory/'gazebo.json')],directory,extra)
            initial=await ready(sim)
            (directory/'initial.json').write_text(json.dumps(initial,indent=2))
            states=deque(maxlen=4000)
            async def observe():
                async with websockets.connect('ws://127.0.0.1:8000/ws/v2/twin') as ws:
                    async for data in ws:
                        states.append({'wall':time.time(),'bytes':len(data),'state':json.loads(data)})
            observer=asyncio.create_task(observe())
            browser=launch(scenario+'-browser',['npx','playwright','test','synchronization.browser.ts']+(['--headed'] if args.headed else []),directory,
                {**extra,'TWIN_SYNC_EVIDENCE_DIR':str(directory),'TWIN_SYNC_RESTART':'1',
                 'TWIN_SYNC_VIDEO':'1' if scenario=='normal' else '0'},ROOT/'web/digital-twin')
            deadline=time.monotonic()+300; restarted=False; memory=[]
            while browser.poll() is None:
                if time.monotonic()>deadline: raise TimeoutError('Browser suite timed out')
                if (directory/'restart.request').exists() and not restarted:
                    await stop(scenario+'-simulation-before-restart',sim)
                    await asyncio.sleep(11)
                    sim=launch(scenario+'-simulation-restarted',['ros2','launch','amr_simulation','warehouse.launch.py','gui:=false','scenario:='+scenario],directory)
                    await ready(sim); restarted=True
                memory.append({'wall':time.time(),'rss':[
                    {'pid':p.pid,'name':p.name(),'bytes':p.memory_info().rss}
                    for _,proc in processes if proc.poll() is None
                    for p in [psutil.Process(proc.pid),*psutil.Process(proc.pid).children(recursive=True)] if helpers.active(p)]})
                await asyncio.sleep(1)
            observer.cancel(); await asyncio.gather(observer,return_exceptions=True)
            (directory/'gateway.json').write_text(json.dumps(list(states)))
            (directory/'memory.json').write_text(json.dumps(memory))
            report['scenarios'][scenario]={'browser_returncode':browser.returncode,'simulator_restarted':restarted,
                'states':len(states),'session_id':initial['session_id']}
            await stop(scenario+'-capture',capture)
            for name,process in [(scenario+'-gateway',gateway),(scenario+'-adapter',adapter),(scenario+'-simulation',sim)]: await stop(name,process)
            assert browser.returncode==0, f'{scenario}: browser failed, inspect {directory}'
            assert restarted, 'Simulator restart was not tested'
            numerical[scenario] = await asyncio.to_thread(analysis.analyze, directory)
            (output/'numerical-validation.json').write_text(json.dumps(numerical, indent=2))
        report['passed']=True
    except Exception:
        report['error']=traceback.format_exc(); raise
    finally:
        if observer is not None:
            observer.cancel()
            await asyncio.gather(observer, return_exceptions=True)
        if states is not None and directory is not None:
            (directory/'gateway.json').write_text(json.dumps(list(states)))
            (directory/'memory.json').write_text(json.dumps(memory))
        for name,process in reversed(processes):
            if process.poll() is None: await stop(name,process)
        for log in logs: log.close()
        (output/'report.json').write_text(json.dumps(report,indent=2))
        print('Evidence:',output,flush=True)

if __name__=='__main__': asyncio.run(main())
