#!/usr/bin/env python3
"""Open the real Ogre GUI on Xvfb; inspect only, with no mouse/keyboard automation."""
import json
import hashlib
import os
import select
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from generate_warehouse import ROOT
from run_acceptance import stop


def run():
    out=ROOT/'validation/gui'; out.mkdir(parents=True,exist_ok=True)
    env={**os.environ,'ROS_DOMAIN_ID':'92','GZ_PARTITION':'warehouse_gui_'+str(os.getpid()),
         'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
    xvfb=sim=gui=None
    with (out/'gazebo.log').open('w') as log, (out/'xvfb.log').open('w') as xlog:
        try:
            xvfb=subprocess.Popen(['Xvfb','-displayfd','1','-screen','0','1600x1000x24','-nolisten','tcp'],
                stdout=subprocess.PIPE,stderr=xlog,text=True,start_new_session=True)
            assert select.select([xvfb.stdout],[],[],15)[0],'Xvfb startup timeout'
            display=xvfb.stdout.readline().strip(); assert display.isdigit()
            sim=subprocess.Popen(['ros2','launch','amr_simulation','warehouse.launch.py','gui:=false'],
                env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+120
            while 'Configured and activated all' not in (out/'gazebo.log').read_text():
                assert time.monotonic()<deadline and sim.poll() is None,'Gazebo/controller startup failed'
                time.sleep(.2)
            gui_env={**env,'DISPLAY':':'+display,'QT_QPA_PLATFORM':'xcb','LIBGL_ALWAYS_SOFTWARE':'1',
                     'GZ_SIM_RESOURCE_PATH':str(ROOT/'models')+os.pathsep+str(__import__('pathlib').Path(get_package_share_directory('amr_description')).parent)}
            gui_env.pop('WAYLAND_DISPLAY',None)
            gui=subprocess.Popen(['gz','sim','-g','--render-engine-gui','ogre',
                    '--gui-config',str(ROOT/'config/warehouse_gui.config')],env=gui_env,
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            script='OUTPUT='+repr(str(out/'warehouse_gui.png'))+'\n'+r'''
import re, subprocess
from PyQt5.QtGui import QGuiApplication
app=QGuiApplication([])
tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
lines=[line for line in tree.splitlines() if 'Gazebo' in line and re.search(r'\d{3,}x\d{3,}',line)]
assert lines,tree
def area(line):
    w,h=map(int,re.search(r'(\d{3,})x(\d{3,})',line).groups())
    return w*h
window=int(re.search(r'0x[0-9a-f]+',max(lines,key=area)).group(),16)
capture=app.primaryScreen().grabWindow(window)
image=capture.toImage()
colours={image.pixel(x,y) for x in range(0,image.width(),8) for y in range(0,image.height(),8)}
assert capture.save(OUTPUT)
print('GUI sample colours:',len(colours),'size:',image.width(),image.height(),flush=True)
assert len(colours)>60, 'Scene is still loading or GUI image is empty'
'''
            deadline=time.monotonic()+90
            while True:
                assert gui.poll() is None,'GUI terminated unexpectedly'
                time.sleep(3)
                capture=subprocess.run([sys.executable,'-c',script],env=gui_env,
                                       capture_output=True,text=True,timeout=20)
                print(capture.stdout.strip(),flush=True)
                if capture.returncode==0:
                    break
                assert time.monotonic()<deadline,capture.stdout+capture.stderr
            assert gui.poll() is None,'GUI terminated unexpectedly'
            text=(out/'gazebo.log').read_text()
            assert '[Err]' not in text and '[ERROR]' not in text,'GUI/server errors; inspect gazebo.log'
            report={'status':'passed','gui_renderer':'ogre','sensor_renderer':'ogre2',
                    'manifest_sha256':hashlib.sha256((ROOT/'config/generated_manifest.json').read_bytes()).hexdigest(),
                    'capture':'warehouse_gui.png','interaction':'read-only screenshot, no input automation'}
            (out/'gui.json').write_text(json.dumps(report,indent=2)+'\n')
            print('WAREHOUSE GUI PASSED',flush=True)
        finally:
            stop(gui); stop(sim); stop(xvfb)


if __name__=='__main__':
    run()
