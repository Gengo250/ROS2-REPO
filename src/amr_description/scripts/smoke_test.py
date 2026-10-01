#!/usr/bin/env python3
"""Isolated RViz + Gazebo acceptance run; always clean up only our own processes."""
import argparse
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
from amr_parameters import ROOT


def stop(process):
    if process is None:
        return
    for sig,timeout in ((signal.SIGINT,15),(signal.SIGTERM,5),(signal.SIGKILL,3)):
        try:
            os.killpg(process.pid,sig)
            process.wait(timeout=timeout)
            # The launcher may have exited before its Gazebo descendants.
            try:
                os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--only',choices=('display','simulation','all'),default='all')
    args = parser.parse_args()
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(170+os.getpid()%30)
    env['GZ_PARTITION'] = 'amr_acceptance_'+str(os.getpid())
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    output = ROOT/'validation'
    output.mkdir(exist_ok=True)
    if args.only in ('display','all'):
        if not shutil.which('Xvfb'):
            raise RuntimeError('RViz automated capture requires xvfb (manual display.launch.py does not).')
        xvfb = display = None
        with (output/'xvfb.log').open('w') as server_log, (output/'rviz.log').open('w') as log:
            try:
                xvfb = subprocess.Popen(['Xvfb','-displayfd','1','-screen','0','1400x900x24','-nolisten','tcp'],
                    stdout=subprocess.PIPE,stderr=server_log,text=True,start_new_session=True)
                if not select.select([xvfb.stdout],[],[],15)[0]:
                    raise TimeoutError('Xvfb did not start')
                number = xvfb.stdout.readline().strip()
                if not number.isdigit():
                    raise RuntimeError('Invalid Xvfb display: '+number)
                display_env = {**env,'DISPLAY':':'+number,'QT_QPA_PLATFORM':'xcb','LIBGL_ALWAYS_SOFTWARE':'1'}
                display_env.pop('WAYLAND_DISPLAY',None)
                display = subprocess.Popen(['ros2','launch','amr_description','display.launch.py'],
                    env=display_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                subprocess.run([sys.executable,str(ROOT/'scripts/validate_display.py')],env=display_env,check=True,timeout=55)
                assert display.poll() is None,'RViz launch exited unexpectedly'
                log.flush()
                assert '[ERROR]' not in (output/'rviz.log').read_text(),'RViz reported an error'
            finally:
                stop(display)
                stop(xvfb)
    if args.only in ('simulation','all'):
        sim = None
        with (output/'gazebo.log').open('w') as log:
            try:
                sim = subprocess.Popen(['ros2','launch','amr_description','sim.launch.py','gui:=false'],
                    env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                subprocess.run([sys.executable,str(ROOT/'scripts/validate_runtime.py')],env=env,check=True,timeout=240)
                assert sim.poll() is None,'Gazebo launch exited unexpectedly'
                log.flush()
                text = (output/'gazebo.log').read_text()
                assert '[ERROR]' not in text and '[Err]' not in text,'Gazebo reported an error; see validation/gazebo.log'
            finally:
                stop(sim)
    print('AMR ACCEPTANCE PASSED:',args.only,flush=True)


if __name__ == '__main__':
    main()
