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
import socket
import traceback
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
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--skip-browser',action='store_true'); parser.add_argument('--output',type=Path); args=parser.parse_args()
    output=(args.output or ROOT/'docs/digital-twin/evidence-phase2'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')).resolve()
    output.mkdir(parents=True,exist_ok=False)
    env={**os.environ,'ROS_DOMAIN_ID':os.environ.get('TWIN_TEST_ROS_DOMAIN_ID','212'),'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
         'GZ_PARTITION':'phase2_'+str(os.getpid()),'PATH':str(ROOT/'.tools/node_modules/.bin')+os.pathsep+os.environ['PATH']}
    report={'output':str(output),'shutdown':{},'commands':[],'passed':False}; processes=[]; logs=[]; owns_services=False
    def launch(name,cmd,extra=None,cwd=ROOT):
        log=(output/(name+'.log')).open('w'); logs.append(log)
        report['commands'].append(cmd)
        p=subprocess.Popen(cmd,cwd=cwd,env={**env,**(extra or {})},stdout=log,stderr=log,start_new_session=True)
        processes.append((name,p)); return p
    def services(action):
        with (output/f'services-{action}-{time.time_ns()}.log').open('w') as log:
            subprocess.run(['python3',str(ROOT/'infrastructure/mosaico/local_services.py'),action],check=True,
                           stdout=log,stderr=subprocess.STDOUT,timeout=60)
    def recorder(name,duration):
        return launch(name,[str(ROOT/'.venv-recorder/bin/python'),'-m','digital_twin_recorder.ros.recorder_node',
            '--duration',str(duration),'--output',str(output/name)])
    try:
        # Do not attach to or shut down another developer's services.
        for port in (8000,5173,6726,55432):
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1',port)) == 0:
                    raise RuntimeError(f'Port {port} occupied; stop its owner before validation')
        owns_services=True
        services('start')
        report['service_pids_before']={name:(ROOT/'.runtime/mosaico'/name).read_text().splitlines()[0]
            for name in ('mosaicod.pid','postgres/postmaster.pid')}

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
        async with httpx.AsyncClient() as client:
            for scenario in ('normal','person_crossing','obstacle_in_aisle','corridor_blocked'):
                response=await client.get('http://127.0.0.1:8000/api/v1/scene',params={'scenario':scenario})
                response.raise_for_status()
                (output/f'scene-{scenario}.json').write_text(json.dumps(response.json(),indent=2)+'\n')
        report['process_commands']=[p.cmdline() for p in psutil.Process(sim.pid).children(recursive=True) if helpers.active(p)]
        assert any('--headless-rendering' in ' '.join(c) and ' -s ' in ' '.join(c) for c in report['process_commands'])
        for topic in ('/odom','/imu/data'):
            result=await asyncio.to_thread(subprocess.run,['ros2','topic','info',topic,'--verbose','--no-daemon','--spin-time','3'],
                env=env,capture_output=True,text=True,check=True,timeout=20)
            (output/(topic.strip('/').replace('/','-')+'-info.txt')).write_text(result.stdout)
            assert 'Publisher count: 1' in result.stdout, result.stdout
        connection=launch('sdk-connectivity',[str(ROOT/'.venv-recorder/bin/python'),'-c',
            'from mosaicolabs import MosaicoClient; import json,dataclasses; '
            'c=MosaicoClient.connect("127.0.0.1",6726); '
            'print(json.dumps(dataclasses.asdict(c.info()),indent=2)); c.close()'])
        assert await wait_process(connection,15)==0

        recording=recorder('recording',0 if not args.skip_browser else 20)
        started=time.monotonic()
        if not args.skip_browser:
            launch('vite',['npm','--prefix','web/digital-twin','run','dev','--','--strictPort'])
            async with httpx.AsyncClient() as client:
                for _ in range(60):
                    try:
                        if (await client.get('http://127.0.0.1:5173')).status_code==200: break
                    except httpx.HTTPError: pass
                    await asyncio.sleep(.5)
                else: raise TimeoutError('Vite not ready')
            (output/'browser').mkdir()
            browser=launch('browser',['npm','run','test:browser'],extra={
                'TWIN_SCENE_EVIDENCE_DIR':str(output/'browser'),
                'TWIN_EVIDENCE_DIR':str(output/'browser')},cwd=ROOT/'web/digital-twin')
            rates=[]; deadline=time.monotonic()+240
            while browser.poll() is None:
                rates.append(await rate(5))
                assert recording.poll() is None,'Recorder exited during rendering'
                if time.monotonic()>deadline: raise TimeoutError('Browser validation timed out')
            assert browser.returncode==0,'Browser checks failed'
            report['concurrent_browser_recording_rates']=rates
            report['during_recording']={'hz':sum(r['hz'] for r in rates)/len(rates),
                'messages':sum(r['messages'] for r in rates)}
            recording.send_signal(signal.SIGINT)
        else:
            report['during_recording']=await rate(22)
        assert await wait_process(recording,45)==0,'Recorder failed; inspect recording.log'
        elapsed=time.monotonic()-started
        reference=next((output/'recording').glob('*/report.json'))
        source=json.loads(reference.read_text()); assert source['state']=='completed'
        report['recording']=source
        report['recording_elapsed_seconds']=elapsed
        report['recording_records_per_wall_second']=sum(t['count'] for t in source['topics'].values())/elapsed
        # Real odometry with an absent IMU must never finalize successfully.
        missing=launch('missing-imu',[str(ROOT/'.venv-recorder/bin/python'),'-m',
            'digital_twin_recorder.ros.recorder_node','--duration','2',
            '--imu-topic','/phase2_missing_imu','--output',str(output/'missing-imu')])
        assert await wait_process(missing,45)!=0
        report['missing_imu']=json.loads(next((output/'missing-imu').glob('*/report.json')).read_text())
        assert report['missing_imu']['state']=='failed'
        assert report['missing_imu']['topics']['/odom']['count']>0
        assert report['missing_imu']['topics']['/phase2_missing_imu']['count']==0
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
        def recorder_memory():
            try:
                owned=[psutil.Process(failure.pid),*psutil.Process(failure.pid).children(recursive=True)]
                return {str(p.pid):p.memory_info().rss for p in owned if helpers.active(p)}
            except psutil.NoSuchProcess: return {}
        outage_pids=list(recorder_memory())
        report['outage_memory_samples']=[recorder_memory()]
        stopping=asyncio.create_task(asyncio.to_thread(services,'stop'))
        report['during_storage_shutdown']=[]
        while not stopping.done():
            report['during_storage_shutdown'].append(await rate(5))
            report['outage_memory_samples'].append(recorder_memory())
        await stopping
        rates=[]
        for _ in range(4):
            rates.append(await rate(5)); report['outage_memory_samples'].append(recorder_memory())
        report['during_outage']={'hz':sum(r['hz'] for r in rates)/len(rates),'samples':rates}

        assert await wait_process(failure,25)!=0,'Storage outage must be reported as failure'
        report['outage']=json.loads(next((output/'outage').glob('*/report.json')).read_text())
        assert report['outage']['state']=='failed'
        report['outage_processes_remaining']=[pid for pid in outage_pids if psutil.pid_exists(int(pid))
            and helpers.active(psutil.Process(int(pid)))]
        assert not report['outage_processes_remaining']
        unavailable=recorder('unavailable-at-start',2)
        assert await wait_process(unavailable,25)!=0
        report['unavailable_at_start']=json.loads(next((output/'unavailable-at-start').glob('*/report.json')).read_text())
        assert report['unavailable_at_start']['state']=='failed'
        async with httpx.AsyncClient() as client:
            health=await client.get('http://127.0.0.1:8000/api/v1/health')
            health.raise_for_status(); report['health_during_outage']=health.json()

        # Shut down simulation before bringing storage back and retrieving old data.
        shutdown={}; helpers.stop(sim,shutdown); report['shutdown']['simulation']=shutdown
        report['simulation_stopped_before_retrieval']=sim.poll() is not None
        assert not shutdown.get('remaining_after_sigint') and not shutdown.get('forced_shutdown')

        services('start')
        services('restart')  # Explicit restart of BOTH PostgreSQL and mosaicod with same persistent data.
        report['service_pids_after']={name:(ROOT/'.runtime/mosaico'/name).read_text().splitlines()[0]
            for name in ('mosaicod.pid','postgres/postmaster.pid')}
        assert all(report['service_pids_before'][name]!=pid for name,pid in report['service_pids_after'].items())
        retrieval=launch('retrieval',[str(ROOT/'.venv-recorder/bin/python'),str(ROOT/'src/digital_twin_recorder/scripts/verify_persistence.py'),
            '--sequence',source['metadata']['execution_id'],'--expected',str(reference),'--output',str(output/'retrieved-after-restart.json')])
        assert await wait_process(retrieval,30)==0,'Historical retrieval failed'
        report['retrieval']=json.loads((output/'retrieved-after-restart.json').read_text())
        invalid=launch('reject-incomplete',[str(ROOT/'.venv-recorder/bin/python'),'-m',
            'digital_twin_recorder.storage.retrieval','--sequence',report['outage']['metadata']['execution_id']])
        assert await wait_process(invalid,15)!=0,'Incomplete outage recording was accepted'
        report['incomplete_sequence_rejected']=True
        listing=launch('sequence-list',[str(ROOT/'.venv-recorder/bin/python'),'-m','digital_twin_recorder.storage.retrieval',
            '--output',str(output/'sequences.json')]); assert await wait_process(listing,15)==0
        report['passed']=True
    except Exception:
        report['error']=traceback.format_exc()
        raise
    finally:
        for name,proc in reversed(processes):
            if proc.poll() is None:
                record={}; helpers.stop(proc,record); report['shutdown'][name]=record
        if owns_services:
            try:
                services('stop'); report['shutdown']['storage']='stopped'
            except Exception as error:
                report['passed']=False; report['shutdown']['storage']=str(error)
        for log in logs: log.close()
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(output,flush=True)
    assert report['passed'] and report['shutdown'].get('storage')=='stopped'

if __name__=='__main__': asyncio.run(main())
