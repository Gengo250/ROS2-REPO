#!/usr/bin/env python3
"""Isolated launches with bounded execution and cleanup of only owned processes."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from generate_warehouse import ROOT, SCENARIOS


def simulation_processes():
    """Read-only guard: never terminate a process we did not launch."""
    import psutil
    found = []
    for p in psutil.process_iter(['pid','ppid','cmdline']):
        args = p.info['cmdline'] or []
        if args and (args[0].startswith('gz sim') or (len(args)>1 and args[1]=='sim') or
                     any('/ros_gz_bridge/parameter_bridge' in a for a in args)):
            found.append(p.info)
    return found


def stop(process):
    if process is None:
        return
    for sig,timeout in ((signal.SIGINT,12),(signal.SIGTERM,5),(signal.SIGKILL,3)):
        try:
            # ros2 launch forwards SIGINT to its children. Sending it to the
            # whole group first delivers the signal twice and creates false
            # shutdown errors (-2) in otherwise healthy ROS nodes.
            if sig == signal.SIGINT:
                process.send_signal(sig)
            else:
                os.killpg(process.pid,sig)
            process.wait(timeout=timeout)
            try:
                os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def run(scenarios, output_root=ROOT/'validation'):
    for i,scenario in enumerate(scenarios):
        assert not simulation_processes(), 'Existing simulation: stop its owning launch first'
        output=output_root/scenario; output.mkdir(parents=True,exist_ok=True)
        env={**os.environ,'ROS_DOMAIN_ID':str(85+i),'GZ_PARTITION':'warehouse_acceptance_'+str(os.getpid())+'_'+scenario,
             'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
        sim=None
        with (output/'gazebo.log').open('w') as log:
            try:
                sim=subprocess.Popen(['ros2','launch','amr_simulation','warehouse.launch.py',
                    'gui:=false','scenario:='+scenario],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                subprocess.run([sys.executable,str(ROOT/'scripts/validate_runtime.py'),'--scenario',scenario,
                    '--output',str(output)],env=env,check=True,timeout=900)
                assert sim.poll() is None,'Gazebo exited unexpectedly'
                log.flush()
                errors=[line for line in (output/'gazebo.log').read_text().splitlines() if '[Err]' in line or '[ERROR]' in line]
                assert not errors,'\n'.join(errors)
            finally:
                stop(sim)
        deadline=time.monotonic()+10
        while simulation_processes() and time.monotonic()<deadline:
            time.sleep(.2)
        assert not simulation_processes(), 'Simulation survived cleanup'
        print('WAREHOUSE ACCEPTANCE PASSED:',scenario,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=SCENARIOS,action='append')
    parser.add_argument('--output-root',type=Path,default=ROOT/'validation')
    args=parser.parse_args(); run(args.scenario or SCENARIOS,args.output_root)
