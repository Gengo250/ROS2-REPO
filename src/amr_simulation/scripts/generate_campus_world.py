#!/usr/bin/env python3
"""Simplified, headless-friendly Gazebo world of the ATLAS campus for physical validation.

Built from the same compiled scene the lite engine uses (atlas_scene): the prototype's coarse
solids as 1.2 m blocks, plus every primitive of a solid layer that reaches robot height (≤ 0.6 m),
on a ground plane. One light, no shadows, 4 ms physics. No workers, vehicles or cameras: this world
validates sensing and contact physics against the lite engine, nothing else.
Usage: generate_campus_world.py OUTPUT.sdf [--scene path]
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def fmt(values):
    return ' '.join(f'{v:.6g}' for v in values)


def shape(link, name, geometry, pose, colour):
    for kind in ('collision', 'visual'):
        element = ET.SubElement(link, kind, name=f'{name}_{kind}')
        ET.SubElement(element, 'pose').text = fmt(pose)
        g = ET.SubElement(ET.SubElement(element, 'geometry'), geometry[0])
        if geometry[0] == 'box':
            ET.SubElement(g, 'size').text = fmt(geometry[1])
        else:
            ET.SubElement(g, 'radius').text = fmt([geometry[1][0]])
            if geometry[0] == 'cylinder':
                ET.SubElement(g, 'length').text = fmt([geometry[1][1]])
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            ET.SubElement(material, 'ambient').text = fmt(colour)
            ET.SubElement(material, 'diffuse').text = fmt(colour)


def build(scene):
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='campus_validation')
    physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.004'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    for filename, name in (('gz-sim-physics-system', 'gz::sim::systems::Physics'),
                           ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
                           ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
                           ('gz-sim-imu-system', 'gz::sim::systems::Imu')):
        ET.SubElement(world, 'plugin', filename=filename, name=name)
    sensors = ET.SubElement(world, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
    ET.SubElement(sensors, 'render_engine').text = 'ogre2'
    scene_element = ET.SubElement(world, 'scene')
    ET.SubElement(scene_element, 'ambient').text = '0.6 0.6 0.6 1'
    ET.SubElement(scene_element, 'shadows').text = 'false'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'direction').text = '-0.4 0.2 -1'
    ET.SubElement(light, 'cast_shadows').text = 'false'
    ground_model = ET.SubElement(world, 'model', name='ground')
    ET.SubElement(ground_model, 'static').text = 'true'
    ground = ET.SubElement(ground_model, 'link', name='floor')
    plane = ET.SubElement(ET.SubElement(ET.SubElement(ground, 'collision', name='floor_collision'), 'geometry'), 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = '220 180'
    model = ET.SubElement(world, 'model', name='campus')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='structure')
    count = 0
    for s in scene.solids:
        shape(link, f"solid_{s['id']}", ('box', [s['length'], s['width'], 1.2]), [s['x'], s['y'], .6, 0, 0, s['yaw']], [.45, .5, .55, 1])
        count += 1
    for kind, prims in scene.primitives.items():
        keep = scene.solid_mask(prims) & (prims.z_min < .6) & (prims.z_max > 0)
        for i, (x, y, z, length, width, height, yaw) in enumerate(prims.data[keep]):
            colour = [.5, .55, .6, 1]
            if kind == 'cube':
                shape(link, f'{kind}_{i}', ('box', [length, width, height]), [x, y, z, 0, 0, yaw], colour)
            elif kind == 'cylinder':
                shape(link, f'{kind}_{i}', ('cylinder', [length, height]), [x, y, z, 0, 0, yaw], colour)
            else:
                shape(link, f'{kind}_{i}', ('sphere', [length]), [x, y, z, 0, 0, 0], colour)
            count += 1
    ET.indent(sdf, space=' ')
    return '<?xml version="1.0"?>\n'+ET.tostring(sdf, encoding='unicode')+'\n', count


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    p.add_argument('--scene', default='')
    a = p.parse_args()
    from atlas_scene.scene import Scene
    text, count = build(Scene.load(a.scene or None))
    a.output.write_text(text)
    print(f'{a.output}: {count} static shapes')


if __name__ == '__main__':
    main()
