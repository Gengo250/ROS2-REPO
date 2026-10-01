#!/usr/bin/env python3
"""Check the display launch's URDF/TF and capture RViz on a test X11 display."""
import json
import re
import subprocess
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from amr_parameters import ROOT


def main():
    rclpy.init()
    node = rclpy.create_node('amr_display_acceptance')
    descriptions,states = [],[]
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    desc_sub = node.create_subscription(String,'/robot_description',descriptions.append,qos)
    state_sub = node.create_subscription(JointState,'/joint_states',states.append,10)
    buffer = Buffer()
    listener = TransformListener(buffer,node)
    deadline = time.monotonic()+40
    while not descriptions or len(states)<30:
        if time.monotonic()>deadline:
            raise TimeoutError('Display nodes did not publish description/joint states')
        rclpy.spin_once(node,timeout_sec=0.1)
    robot = ET.fromstring(descriptions[0].data)
    links = [link.attrib['name'] for link in robot.findall('link')]
    for link in links:
        while not buffer.can_transform('base_link',link,rclpy.time.Time()):
            if time.monotonic()>deadline:
                raise TimeoutError('Missing TF for '+link)
            rclpy.spin_once(node,timeout_sec=0.1)
    # Screen reads are inspection only. No mouse, keyboard or UI actions are performed.
    from PyQt5.QtGui import QGuiApplication
    app = QGuiApplication([])
    tree = subprocess.check_output(['xwininfo','-root','-tree'],text=True)
    line = next(line for line in tree.splitlines() if ' - RViz' in line and '("rviz2" "rviz2")' in line)
    window = int(re.search(r'0x[0-9a-f]+',line).group(),16)
    screenshot = app.primaryScreen().grabWindow(window)
    image = screenshot.toImage()
    colours = {image.pixel(x,y) for x in range(0,image.width(),16) for y in range(0,image.height(),16)}
    assert len(colours)>25,'RViz screenshot is blank'
    assert screenshot.save(str(ROOT/'validation/rviz.png'))
    report = {'status':'passed','tf_links':links,'joint_names':states[-1].name,
              'capture':'rviz.png','capture_unique_sample_colours':len(colours)}
    (ROOT/'validation/display.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: display nodes, all TFs, wheel joint states and nonblank RViz capture',flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
