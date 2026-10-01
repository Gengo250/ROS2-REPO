#!/usr/bin/env python3
"""Validate expanded URDFs, frame positions, inertia and primitive-only collisions."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from amr_parameters import ROOT, load_parameters


def numbers(value):
    return [float(v) for v in value.split()]


def validate(xml, p, simulation):
    robot = ET.fromstring(xml)
    links = {link.attrib['name']:link for link in robot.findall('link')}
    joints = robot.findall('joint')
    assert len(links) == 15 and len(joints) == 14, (len(links),len(joints))
    children = [j.find('child').attrib['link'] for j in joints]
    assert len(set(children)) == len(children)
    assert set(links)-set(children) == {'base_link'}
    xyz = {'base_link':[0,0,0]}
    pending = list(joints)
    while pending:
        old = len(pending)
        for joint in pending[:]:
            parent,child = (joint.find(s).attrib['link'] for s in ('parent','child'))
            if parent in xyz:
                local = numbers(joint.find('origin').attrib.get('xyz','0 0 0'))
                # All parent frames are ROS-aligned; optical frame is the only rotated leaf.
                xyz[child] = [a+b for a,b in zip(xyz[parent],local)]
                pending.remove(joint)
        assert len(pending)<old,'Disconnected or cyclic TF tree'
    expected = {'chassis_link':[0,0,p['chassis_z']], 'platform_link':[0,0,p['platform_z']],
                'left_wheel_link':[0,p['wheel_y'],p['wheel_z']], 'right_wheel_link':[0,-p['wheel_y'],p['wheel_z']],
                'laser_frame':[p['lidar_x'],0,p['laser_z']], 'camera_link':[p['camera_x'],0,p['camera_z']]}
    for name,point in expected.items():
        assert all(abs(a-b)<1e-8 for a,b in zip(xyz[name],point)),name
    virtual = {'base_link','laser_frame','camera_optical_frame','camera_depth_frame'}
    mass = 0
    for name,link in links.items():
        inertial = link.find('inertial')
        if name in virtual:
            continue
        assert inertial is not None,name
        m = float(inertial.find('mass').attrib['value'])
        assert m>0 and math.isfinite(m)
        mass += m
        inertia = inertial.find('inertia').attrib
        diag = [float(inertia[n]) for n in ('ixx','iyy','izz')]
        assert all(math.isfinite(v) and v>0 for v in diag),name
        assert all(diag[i]<=sum(diag)-diag[i]+1e-8 for i in range(3)),name
        assert all(float(inertia[n]) == 0 for n in ('ixy','ixz','iyz')),name
    assert not robot.findall('.//collision/geometry/mesh')
    for joint in joints:
        if joint.attrib['type']=='continuous':
            assert joint.find('axis').attrib['xyz']=='0 1 0'
    for mesh in robot.findall('.//visual/geometry/mesh'):
        assert mesh.attrib['scale']=='1 1 1'
        path = ROOT/mesh.attrib['filename'].split('package://amr_description/')[1]
        assert path.stat().st_size>84,path
    assert len(robot.findall('ros2_control')) == int(simulation)
    assert len(robot.findall('.//sensor')) == (3 if simulation else 0)
    return {'links':list(links),'joints':[{**j.attrib,'parent':j.find('parent').attrib['link'],
            'child':j.find('child').attrib['link']} for j in joints],'link_positions':xyz,'mass_kg':mass}


def main():
    p = load_parameters()
    report = {}
    with tempfile.TemporaryDirectory(prefix='amr_urdf_') as temp:
        for simulation in (False,True):
            xml = subprocess.check_output(['xacro',str(ROOT/'urdf/amr.urdf.xacro'),
                'gazebo:='+str(simulation).lower(),'controllers_file:='+str(ROOT/'config/controllers.yaml')],text=True)
            result = validate(xml,p,simulation)
            urdf = Path(temp)/'amr.urdf'
            urdf.write_text(xml)
            if shutil.which('check_urdf'):
                result['check_urdf'] = subprocess.check_output(['check_urdf',str(urdf)],text=True)
            else:
                result['check_urdf'] = 'unavailable'
            if simulation and shutil.which('gz'):
                sdf = subprocess.run(['gz','sdf','-p',str(urdf)],text=True,capture_output=True,check=True)
                converted = ET.fromstring(sdf.stdout)
                assert converted.find('model').attrib['name'] == 'warehouse_amr'
                assert len(converted.findall('.//sensor')) == 3
                assert 'Error' not in sdf.stderr,sdf.stderr
                # gz_frame_id is a Gazebo extension retained by SDFormat, not a physics warning.
                assert all('gz_frame_id' in line for line in sdf.stderr.splitlines() if 'Warning' in line),sdf.stderr
                result['sdf_conversion_stderr'] = sdf.stderr
            report['simulation' if simulation else 'display'] = result
    output = ROOT/'validation'
    output.mkdir(exist_ok=True)
    (output/'urdf.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: Xacro, XML, check_urdf, tree, positions, inertia, collision, mesh paths and Harmonic SDF')
    print('Total prototype mass:',report['display']['mass_kg'],'kg')


if __name__ == '__main__':
    main()
