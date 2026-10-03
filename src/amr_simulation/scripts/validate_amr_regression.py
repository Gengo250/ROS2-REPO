#!/usr/bin/env python3
"""Run original AMR checks in a copy so existing user validation artifacts survive."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from generate_warehouse import ROOT


def run():
    source=ROOT.parent/'amr_description'
    if not source.is_dir():
        source=Path(get_package_share_directory('amr_description'))
    output=ROOT/'validation/amr_regression'; output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='warehouse_amr_regression_') as temp:
        amr=Path(temp)/'amr_description'; shutil.copytree(source,amr)
        (amr/'validation').mkdir(exist_ok=True)
        for script in ('validate_urdf.py','smoke_test.py','validate_gazebo_gui.py'):
            with (output/(script+'.log')).open('w') as log:
                subprocess.run([sys.executable,str(amr/'scripts'/script)],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=360)
        for name in ('urdf.json','runtime.json','display.json','gazebo_gui.json','rviz.png','gazebo.png'):
            shutil.copy2(amr/'validation'/name,output/name)
    print('ORIGINAL AMR REGRESSION PASSED',flush=True)


if __name__=='__main__':
    run()
