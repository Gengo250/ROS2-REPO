#!/usr/bin/env python3
"""Real headless simulation, independent recording, failure isolation and restart proof.

Run with the Gateway .venv and sourced ROS workspace. Uses native Mosaico services;
no fake publishers. Creates a new evidence directory and stops only owned processes.
"""
import argparse
import asyncio
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import httpx
import psutil
import websockets

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('phase1_validation',ROOT/'src/digital_twin_gateway/scripts/validate_integration.py')
helpers=importlib.util.module_from_spec(spec); spec.loader.exec_module(helpers)

async def rate(seconds):
    stamps=[]; first=last=None
    async with websockets.connect('ws://127.0.0.1:8000/ws/v1/twin') as socket:
        start=time.monotonic()
        while time.monotonic()-start<seconds:
            state=json.loads(await asyncio.wait_for(socket.recv(),timeout=3))
            stamps.append(time.monotonic()); first=first or state; last=state
    assert last['robots'][0]['status']=='online' and last['world']['status']=='running'
    return dict(messages=len(stamps),hz=(len(stamps)-1)/(stamps[-1]-stamps[0]),first=first,last=last)

async def wait_process(proc,timeout):
    deadline=time.monotonic()+timeout
    while proc.poll() is None:
        if time.monotonic()>deadline: raise TimeoutError(str(proc.args))
        await asyncio.sleep(.2)
    return proc.returncode

async def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--skip-browser',action='store_true'); args=parser.parse_args()
    output=ROOT/'docs/digital-twin/evidence-phase2'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output.mkdir(parents=True,exist_ok=False)
    env={**os.environ,'ROS_DOMAIN_ID':'212','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
         'GZ_PARTITION':'phase2_'+str(os.getpid()),'PATH':str(ROOT/'.tools/node_modules/.bin')+os.pathsep+os.environ['PATH']}
    report={'output':str(output),'shutdown':{},'commands':[]}; processes=[]; logs=[]
    def launch(name,cmd,extra=None,cwd=ROOT):
        log=(output/(name+'.log')).open('w'); logs.append(log)
        report['commands'].append(cmd)
        p=subprocess.Popen(cmd,cwd=cwd,env={**env,**(extra or {})},stdout=log,stderr=log,start_new_session=True)
        processes.append((name,p)); return p
    def services(action):
        subprocess.run(['python3',str(ROOT/'infrastructure/mosaico/local_services.py'),action],check=True,
                       stdout=(output/f'services-{action}-{time.time_ns()}.log').open('w'),stderr=subprocess.STDOUT)
    def recorder(name,duration):
        return launch(name,[str(ROOT/'.venv-recorder/bin/python'),'-m','digital_twin_recorder.ros.recorder_node',
            '--duration',str(duration),'--output',str(output/name)])
    try:
        services('start')
        sim=launch('simulation',['ros2','launch','digital_twin_gateway','digital_twin.launch.py','scenario:=normal'])
        async with httpx.AsyncClient() as client:
            for _ in range(180):
                if sim.poll() is not None: raise RuntimeError('Simulation launch exited')
                try:
                    state=(await client.get('http://127.0.0.1:8000/api/v1/state')).json()
                    if state['world']['status']=='running' and state['robots'][0]['status']=='online': break
                except (httpx.HTTPError,KeyError): pass
                await asyncio.sleep(1)
            else: raise TimeoutError('Real telemetry not ready')
        report['baseline']=await rate(5)
        recording=recorder('recording',20)
        started=time.monotonic(); report['during_recording']=await rate(22)
        assert await wait_process(recording,45)==0,'Recorder failed; inspect recording.log'
        elapsed=time.monotonic()-started
        reference=next((output/'recording').glob('*/report.json'))
        source=json.loads(reference.read_text()); assert source['state']=='completed'
        report['recording']=source
        report['recording_elapsed_seconds']=elapsed
        report['recording_records_per_wall_second']=sum(t['count'] for t in source['topics'].values())/20
        if not args.skip_browser:
            web=launch('vite',['npm','--prefix','web/digital-twin','run','dev'])
            await asyncio.sleep(2)
            browser=launch('browser',['npm','run','test:browser'],extra={'TWIN_SCENE_EVIDENCE_DIR':str(output/'browser')},cwd=ROOT/'web/digital-twin')
            assert await wait_process(browser,150)==0,'Browser checks failed'
        report['rss_bytes']={p.name():p.memory_info().rss for _,proc in processes if proc.poll() is None
                             for p in [psutil.Process(proc.pid),*psutil.Process(proc.pid).children(recursive=True)] if helpers.active(p)}
        # Simulate a real daemon outage while ROS and Gateway continue operating.
        failure=recorder('outage',60)
        for _ in range(75):
            paths=list((output/'outage').glob('*/report.json'))
            if paths and json.loads(paths[0].read_text())['state']=='recording': break
            if failure.poll() is not None: raise RuntimeError('Outage recorder did not start')
            await asyncio.sleep(.2)
        else: raise TimeoutError('Outage recorder not ready')
        await asyncio.sleep(2)
        services('stop')
        report['during_outage']=await rate(17)
        assert await wait_process(failure,25)!=0,'Storage outage must be reported as failure'
        report['outage']=json.loads(next((output/'outage').glob('*/report.json')).read_text())
        assert report['outage']['state']=='failed'
        # Shut down simulation before bringing storage back and retrieving old data.
        shutdown={}; helpers.stop(sim,shutdown); report['shutdown']['simulation']=shutdown
        report['simulation_stopped_before_retrieval']=sim.poll() is not None
        services('start')
        services('restart')  # Explicit restart of BOTH PostgreSQL and mosaicod with same persistent data.
        retrieval=launch('retrieval',[str(ROOT/'.venv-recorder/bin/python'),'-m','digital_twin_recorder.storage.retrieval',
            '--sequence',source['metadata']['execution_id'],'--expected',str(reference),'--output',str(output/'retrieved-after-restart.json')])
        assert await wait_process(retrieval,30)==0,'Historical retrieval failed'
        report['retrieval']=json.loads((output/'retrieved-after-restart.json').read_text())
        listing=launch('sequence-list',[str(ROOT/'.venv-recorder/bin/python'),'-m','digital_twin_recorder.storage.retrieval',
            '--output',str(output/'sequences.json')]); assert await wait_process(listing,15)==0
        report['passed']=True
    finally:
        for name,proc in reversed(processes):
            if proc.poll() is None:
                record={}; helpers.stop(proc,record); report['shutdown'][name]=record
        for log in logs: log.close()
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(output,flush=True)

if __name__=='__main__': asyncio.run(main())
