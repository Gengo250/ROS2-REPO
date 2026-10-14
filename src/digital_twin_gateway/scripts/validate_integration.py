#!/usr/bin/env python3
"""Owned headless launch → ROS → REST/WebSocket → Chromium integration evidence."""
import argparse
import asyncio
from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import shlex
import subprocess
import time
import httpx
from jsonschema import Draft202012Validator, FormatChecker
import psutil
import websockets

ROOT = Path(__file__).resolve().parents[3]


def active(process):
    try:
        return process.is_running() and process.status() != psutil.STATUS_ZOMBIE
    except psutil.NoSuchProcess:
        return False


def stop(process, records):
    if process is None:
        return
    try:
        children = psutil.Process(process.pid).children(recursive=True)
    except psutil.NoSuchProcess:
        children = []
    try:
        if 'launch' in process.args:
            # Launch forwards shutdown itself; avoid sending a second signal to
            # every ROS child while it is already destroying its context.
            process.send_signal(signal.SIGINT)
        else:
            os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=25)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        records['forced_shutdown'] = True
    _, alive = psutil.wait_procs(children, timeout=5)
    alive = [p for p in alive if active(p)]
    records['remaining_after_sigint'] = [{'pid': p.pid, 'cmd': p.cmdline()} for p in alive]
    if alive or process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        for p in alive:
            try:
                p.terminate()
            except psutil.NoSuchProcess:
                pass
        _, alive = psutil.wait_procs(alive, timeout=5)
        for p in alive:
            if active(p):
                p.kill()
        process.wait(timeout=5)
    records['returncode'] = process.returncode


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--scenario', default='normal', choices=[
        'normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked'])
    parser.add_argument('--skip-browser', action='store_true')
    args = parser.parse_args()
    output = (args.output or ROOT / 'docs/digital-twin/evidence' /
              datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = {**os.environ, 'ROS_DOMAIN_ID': os.environ.get('TWIN_TEST_ROS_DOMAIN_ID', '212'),
           'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
           'GZ_PARTITION': 'digital_twin_validation_'+str(os.getpid()),
           'TWIN_EVIDENCE_DIR': str(output), 'TWIN_PORT': '8000',
           'PATH': str(ROOT / '.tools/node_modules/.bin')+os.pathsep+os.environ['PATH']}
    report = {'scenario': args.scenario, 'ros_domain_id': env['ROS_DOMAIN_ID'],
              'gz_partition': env['GZ_PARTITION'], 'checks': [], 'shutdown': {}}
    sim = web = observer = None
    schema = json.loads((ROOT / 'shared/schemas/twin-state.schema.json').read_text())
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    samples = deque(maxlen=600)
    arrival_times = deque(maxlen=600)
    disconnected = asyncio.Event()
    def check(condition, label):
        assert condition, label
        report['checks'].append(label)
        print('PASS:', label, flush=True)
    async def observe():
        try:
            async with websockets.connect('ws://127.0.0.1:8000/ws/v1/twin?protocol_version=1.0',
                                          origin='http://127.0.0.1:5173') as socket:
                async for data in socket:
                    state = json.loads(data)
                    validator.validate(state)
                    if samples:
                        assert state['sequence'] > samples[-1]['sequence']
                    samples.append(state)
                    arrival_times.append(time.monotonic())
        except websockets.exceptions.ConnectionClosedError as error:
            report['websocket_close_code'] = error.rcvd.code if error.rcvd else None
            if report['websocket_close_code'] != 1012:
                raise
        finally:
            disconnected.set()
    with (output/'launch.log').open('w') as simlog, (output/'vite.log').open('w') as weblog:
        try:
            async with httpx.AsyncClient(timeout=2) as client:
                # Refuse to mix this validation with an existing API instance.
                try:
                    await client.get('http://127.0.0.1:8000/api/v1/health')
                except httpx.TransportError:
                    pass
                else:
                    raise RuntimeError('Port 8000 already serves an HTTP application')
                sim = subprocess.Popen(['ros2', 'launch', 'digital_twin_gateway', 'digital_twin.launch.py',
                                        'scenario:='+args.scenario], cwd=ROOT, env=env,
                                       stdout=simlog, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+120
                while True:
                    if sim.poll() is not None:
                        raise RuntimeError('Launch exited: inspect launch.log')
                    try:
                        response = await client.get('http://127.0.0.1:8000/api/v1/state')
                        response.raise_for_status()
                        initial = response.json()
                        if initial['robots'][0]['status'] == 'online' and initial['world']['status'] == 'running':
                            break
                    except httpx.TransportError:
                        pass
                    if time.monotonic() > deadline:
                        raise TimeoutError('No real odometry/advancing clock within 120 seconds')
                    await asyncio.sleep(.5)
                validator.validate(initial)
                (output/'initial-state.json').write_text(json.dumps(initial, indent=2)+'\n')
                check(initial['world']['scenario'] == args.scenario, 'scenario is carried from the composed launch')
                check(initial['robots'][0]['frame_id'] == 'odom', 'real odometry retains odom frame')
                processes = psutil.Process(sim.pid).children(recursive=True)
                # gz's Ruby wrapper rewrites argv into a single process-title string.
                commands = [shlex.split(' '.join(p.cmdline())) for p in processes if active(p)]
                report['process_commands'] = commands
                check(any('--headless-rendering' in c and '-s' in c for c in commands), 'Gazebo server uses headless rendering')
                check(not any(('gz' in c and '-g' in c) or any('rviz2' in x for x in c) for c in commands), 'no Gazebo GUI or RViz process')
                for name, command in [('controllers', ['ros2', 'service', 'call',
                                      '/controller_manager/list_controllers',
                                      'controller_manager_msgs/srv/ListControllers', '{}']),
                                      ('topics', ['ros2', 'topic', 'list', '-t', '--no-daemon'])]:
                    result = await asyncio.to_thread(subprocess.run, command, env=env,
                                                     capture_output=True, text=True, timeout=30, check=True)
                    (output/f'{name}.txt').write_text(result.stdout)
                    if name == 'controllers':
                        check('diff_drive_controller' in result.stdout and
                              result.stdout.count('active') >= 2, 'both existing controllers are active')
                observer = asyncio.create_task(observe())
                await asyncio.sleep(3)
                check(len(samples) >= 10, 'WebSocket receives multiple schema-valid sequenced snapshots')
                if not args.skip_browser:
                    web = subprocess.Popen(['npm', 'run', 'dev'], cwd=ROOT/'web/digital-twin', env=env,
                                           stdout=weblog, stderr=subprocess.STDOUT, start_new_session=True)
                    deadline = time.monotonic()+30
                    while True:
                        try:
                            response = await client.get('http://127.0.0.1:5173')
                            if response.status_code == 200:
                                break
                        except httpx.TransportError:
                            pass
                        if time.monotonic() > deadline:
                            raise TimeoutError('Vite did not start')
                        await asyncio.sleep(.25)
                    result = await asyncio.to_thread(subprocess.run, ['npm', 'run', 'test:browser'],
                        cwd=ROOT/'web/digital-twin', env=env, capture_output=True, text=True, timeout=210)
                    (output/'browser-test.log').write_text(result.stdout+result.stderr)
                    check(result.returncode == 0, 'Chromium displays odometry, motion, disconnection and reconnection')
                    check(max(s['connection']['active_websocket_clients'] for s in samples) >= 2,
                          'simultaneous browser and Python WebSocket clients receive telemetry')
                report['sample_count_retained'] = len(samples)
                report['measured_websocket_hz'] = (len(arrival_times)-1)/(arrival_times[-1]-arrival_times[0])
                report['mean_snapshot_bytes'] = sum(len(json.dumps(s)) for s in samples)/len(samples)
                (output/'final-state.json').write_text(json.dumps(samples[-1], indent=2)+'\n')
                (output/'status.json').write_text(json.dumps((await client.get(
                    'http://127.0.0.1:8000/api/v1/status')).json(), indent=2)+'\n')
                await asyncio.to_thread(stop, sim, report['shutdown'])
                sim = None
                await asyncio.wait_for(disconnected.wait(), 10)
                await observer
                observer = None
                check(not report['shutdown'].get('remaining_after_sigint') and
                      not report['shutdown'].get('forced_shutdown'), 'SIGINT terminates all launch-owned processes')
                check(disconnected.is_set(), 'server shutdown closes the active WebSocket')
                simlog.flush()
                check('[gateway-8] ERROR:' not in (output/'launch.log').read_text(),
                      'Gateway log has no unhandled ASGI errors during disconnect or shutdown')
                try:
                    await client.get('http://127.0.0.1:8000/api/v1/health')
                except httpx.TransportError:
                    check(True, 'Gateway port is released after shutdown')
                else:
                    raise AssertionError('Gateway still responds after shutdown')
        finally:
            if observer:
                observer.cancel()
                await asyncio.gather(observer, return_exceptions=True)
            if sim:
                await asyncio.to_thread(stop, sim, report['shutdown'])
            if web:
                await asyncio.to_thread(stop, web, report.setdefault('web_shutdown', {}))
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print('Evidence:', output, flush=True)


if __name__ == '__main__':
    asyncio.run(main())
