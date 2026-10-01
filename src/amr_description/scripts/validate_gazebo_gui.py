#!/usr/bin/env python3
"""Capture the actual Harmonic GUI on an isolated display without UI interaction."""
import json
import os
import re
import select
import subprocess
import sys
import time
from amr_parameters import ROOT
from smoke_test import stop


def main():
    output = ROOT/'validation'
    env = {**os.environ,'ROS_DOMAIN_ID':str(200+os.getpid()%20),
           'GZ_PARTITION':'amr_gui_'+str(os.getpid()),'QT_QPA_PLATFORM':'xcb',
           'LIBGL_ALWAYS_SOFTWARE':'1','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
    env.pop('WAYLAND_DISPLAY',None)
    server = sim = gui = None
    with (output/'gazebo_gui.log').open('w') as log, (output/'gazebo_xvfb.log').open('w') as xlog:
        try:
            server = subprocess.Popen(['Xvfb','-displayfd','1','-screen','0','1400x900x24','-nolisten','tcp'],
                stdout=subprocess.PIPE,stderr=xlog,text=True,start_new_session=True)
            assert select.select([server.stdout],[],[],15)[0],'Xvfb startup timeout'
            display = server.stdout.readline().strip()
            assert display.isdigit()
            env['DISPLAY'] = ':'+display
            sim_env = {**os.environ,'ROS_DOMAIN_ID':env['ROS_DOMAIN_ID'],'GZ_PARTITION':env['GZ_PARTITION'],
                       'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
            sim = subprocess.Popen(['ros2','launch','amr_description','sim.launch.py','gui:=false'],
                env=sim_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline = time.monotonic()+90
            while 'Configured and activated all' not in (output/'gazebo_gui.log').read_text():
                assert time.monotonic()<deadline and sim.poll() is None,'Gazebo GUI startup failed'
                time.sleep(0.2)
            gui = subprocess.Popen(['gz','sim','-g','--render-engine-gui','ogre',
                                    '--gui-config',str(ROOT/'config/gazebo_gui.config')],
                env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            # Allow the scene broadcaster to deliver the new model to the GUI renderer.
            time.sleep(8)
            script = r'''
import re,subprocess
from PyQt5.QtGui import QGuiApplication
app=QGuiApplication([])
tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
lines=[line for line in tree.splitlines() if 'Gazebo' in line and re.search(r'\d{3,}x\d{3,}',line)]
assert lines,tree
window=int(re.search(r'0x[0-9a-f]+',lines[0]).group(),16)
capture=app.primaryScreen().grabWindow(window)
assert capture.save(OUTPUT)
'''
            script = 'OUTPUT='+repr(str(output/'gazebo.png'))+'\n'+script
            subprocess.run([sys.executable,'-c',script],env=env,check=True,timeout=20)
            assert '[Err]' not in (output/'gazebo_gui.log').read_text(),'Gazebo GUI reported an error'
            (output/'gazebo_gui.json').write_text(json.dumps({'status':'passed','capture':'gazebo.png',
                'gui_render_engine':'Ogre','sensor_render_engine':'Ogre2',
                'automated_ui_interactions':0},indent=2)+'\n')
            print('PASS: Harmonic GUI, spawned AMR, active controllers, screenshot saved')
        finally:
            stop(gui)
            stop(sim)
            stop(server)


if __name__ == '__main__':
    main()
