#!/usr/bin/env python3
"""Own one real Gazebo + Gateway + Nav2 stack and execute graph routes end to end.

Run from a sourced workspace. Steps are ``execute_route`` invocations; each may
declare its expected exit code (negative tests). No velocity publisher exists in
this harness. Optional browser capture observes the existing Three.js viewer.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import threading
import time
import traceback
import urllib.request

ROOT = Path(__file__).resolve().parents[3]
NAV = ROOT/'src/amr_navigation/scripts'


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def get(path):
    return json.load(urllib.request.urlopen('http://127.0.0.1:8000'+path, timeout=2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scenario', choices=['normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked'], default='normal')
    p.add_argument('--initial', type=float, nargs=3, required=True, metavar=('X', 'Y', 'YAW'), help='AMCL initial map pose')
    p.add_argument('--step', action='append', required=True,
                   help="execute_route arguments, e.g. '--to dropoff_station_02 --expect 0'")
    p.add_argument('--close-after', nargs=3, action='append', default=[], metavar=('STEP', 'SECONDS', 'CONNECTION'),
                   help='Operator closure published on /route_planner/block_connection during a step')
    p.add_argument('--planner-scenario', help='Scenario the planner is told (default: the simulated one). Telling it '
                   '"normal" while simulating corridor_blocked makes the barrier an unannounced physical obstruction.')
    p.add_argument('--browser', action='store_true')
    p.add_argument('--step-delay', type=float, default=0., help='Wall seconds before each step (0-120): varies AMR/worker timing')
    p.add_argument('--candidate', type=Path, help='Candidate map package (warehouse.yaml, registration, graph) to validate before approval')
    p.add_argument('--amcl-scan-topic', default='', help='AMCL scan topic; must match the sensor that built the map')
    p.add_argument('--domain', type=int, default=47)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    env = {**os.environ, 'ROS_DOMAIN_ID': str(a.domain), 'GZ_PARTITION': f'phase5_{os.getpid()}',
           'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'PATH': str(ROOT/'.tools/node_modules/.bin')+os.pathsep+os.environ['PATH']}
    children, logs = [], []
    planner_scenario = a.planner_scenario or a.scenario
    report = {'scenario': a.scenario, 'planner_scenario': planner_scenario, 'initial_map_pose': a.initial, 'steps': [], 'passed': False,
              'step_delay_s': a.step_delay}
    done = threading.Event()

    def launch(name, cmd, cwd=ROOT, extra=None):
        f = (out/(name+'.log')).open('w')
        logs.append(f)
        proc = subprocess.Popen(cmd, cwd=cwd, env={**env, **(extra or {})}, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(proc)
        return proc

    def record_gateway():
        # Independent observation of what the Gateway serves to the browser.
        with (out/'gateway-route-status.jsonl').open('w') as f:
            while not done.is_set():
                try:
                    f.write(json.dumps({'wall': time.time(), 'status': get('/api/v1/routes/status')})+'\n')
                    f.flush()
                except Exception as exc:
                    f.write(json.dumps({'wall': time.time(), 'error': repr(exc)})+'\n')
                done.wait(1.)
    try:
        for port in ([8000, 5173] if a.browser else [8000]):
            with socket.socket() as s:
                if s.connect_ex(('127.0.0.1', port)) == 0:
                    raise RuntimeError(f'Port {port} occupied; stop the existing owned stack first')
        extra = (['map:='+str((a.candidate/'warehouse.yaml').resolve())] if a.candidate else []) + \
            (['amcl_scan_topic:='+a.amcl_scan_topic] if a.amcl_scan_topic else [])
        report['candidate'] = str(a.candidate) if a.candidate else None
        # The Gateway serves the candidate's graph so the browser overlay can be validated too.
        gateway_env = {'TWIN_ROUTE_GRAPH_PATH': str((a.candidate/'warehouse_graph.json').resolve())} if a.candidate else None
        sim = launch('autonomous-demo', ['ros2', 'launch', 'amr_navigation', 'autonomous_demo.launch.py', 'scenario:='+a.scenario,
                                         'initialize:=true', *[f'initial_{k}:={v}' for k, v in zip(['x', 'y', 'yaw'], a.initial)], *extra],
                     extra=gateway_env)
        deadline = time.monotonic()+150
        while time.monotonic() < deadline:
            if sim.poll() is not None:
                raise RuntimeError('Simulation/navigation launch exited')
            try:
                state = get('/api/v2/state')
                if state['simulation_robots'][0]['status'] == 'online' and state['robots'][0]['status'] == 'online':
                    break
            except Exception:
                pass
            time.sleep(.5)
        else:
            raise TimeoutError('No live Gazebo world pose and odometry')
        if launch('sensors', ['ros2', 'run', 'amr_navigation', 'sensor_audit', '--output', str(out/'sensors.json')]).wait(timeout=40) != 0:
            raise RuntimeError('Sensor audit failed')
        report['route_graph_endpoint'] = get('/api/v1/routes/graph')['graph_version']
        duration = str(900*len(a.step))
        launch('contacts', [str(ROOT/'.venv/bin/python'), str(NAV/'observe_contacts.py'), '--duration', duration, '--output', str(out/'contacts.json')])
        launch('obstacles', ['python3', str(NAV/'observe_obstacles.py'), '--duration', duration, '--output', str(out/'obstacles.json')]
               + (['--candidate', str(a.candidate.resolve())] if a.candidate else []))
        threading.Thread(target=record_gateway, daemon=True).start()
        browser = None
        if a.browser:
            launch('vite', ['npm', '--prefix', 'web/digital-twin', 'run', 'dev', '--', '--strictPort'])
            browser = launch('browser', ['npx', 'playwright', 'test', 'routes.browser.ts'], ROOT/'web/digital-twin',
                             {'ROUTE_EVIDENCE_DIR': str(out), 'ROUTE_OBSERVE_SECONDS': str(min(7200, 900*len(a.step)))})
            deadline = time.monotonic()+90
            while not (out/'browser.ready').exists():
                if browser.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Browser did not become ready before routing')
                time.sleep(.2)
        if not 0 <= a.step_delay <= 120:
            raise ValueError('step-delay must be 0-120 s')
        for i, step in enumerate(a.step):
            time.sleep(a.step_delay)
            args = shlex.split(step)
            expect = 0
            if '--expect' in args:
                k = args.index('--expect')
                expect = int(args[k+1])
                del args[k:k+2]
            if a.candidate:
                args += ['--candidate', str(a.candidate.resolve())]
            proc = launch(f'route-{i}', ['ros2', 'run', 'amr_route_planner', 'execute_route', *args, '--scenario', planner_scenario,
                                        '--output', str(out/f'route-{i}.json')])
            closures = []
            for step, seconds, connection in a.close_after:
                if int(step) == i:
                    timer = threading.Timer(float(seconds), lambda c=connection: closures.append(subprocess.run(
                        ['ros2', 'topic', 'pub', '--once', '-w', '1', '/route_planner/block_connection', 'std_msgs/msg/String',
                         json.dumps({'data': c})], env=env, capture_output=True, text=True, timeout=30).returncode))
                    timer.start()
            code = proc.wait(timeout=3600)
            result = json.loads((out/f'route-{i}.json').read_text())
            report['steps'].append({'args': args, 'expected': expect, 'returncode': code,
                                    'final_status': result['final_status']['status'], 'failure': result['final_status']['failure_reason'],
                                    'closures_published': closures})
            (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            if code != expect:
                raise RuntimeError(f'Step {i} exited {code}, expected {expect}; inspect route-{i}.json')
        (out/'navigation.done').write_text('finished')
        if browser is not None:
            report['browser_returncode'] = browser.wait(timeout=600)
            if browser.returncode != 0:
                raise RuntimeError('Browser route visualization assertions failed')
        report['passed'] = True
    except Exception:
        report['error'] = traceback.format_exc()
    finally:
        done.set()
        (out/'navigation.done').write_text('finished')
        for child in reversed(children):
            stop(child)
        for f in logs:
            f.close()
        report['owned_processes_stopped'] = all(c.poll() is not None for c in children)
        contacts = out/'contacts.json'
        if contacts.exists():
            samples = json.loads(contacts.read_text())['amr_contact_samples']
            report['scoped_amr_contact_samples'] = samples
            if samples:
                report['passed'] = False
                report['contact_failure'] = 'AMR contact detected by existing worker/wall/barrier sensors'
        else:
            report['passed'] = False
            report['contact_failure'] = 'Contact observer produced no report'
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'error'}, indent=2))
    if report.get('error'):
        print(report['error'])
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
