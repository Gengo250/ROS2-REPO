#!/usr/bin/env python3
"""Draw the configured warehouse as a self-contained, deterministic SVG plan."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def render(c, output, amr):
    length,width = c['building']['length'],c['building']['width']
    scale,margin = 30,65
    root = ET.Element('svg',xmlns='http://www.w3.org/2000/svg',
                      viewBox=f'0 0 {length*scale+2*margin} {width*scale+2*margin+75}')
    def el(name, **attrs):
        return ET.SubElement(root,name,{k.replace('_','-'):str(v) for k,v in attrs.items()})
    def xy(x,y):
        return margin+(x+length/2)*scale,margin+(width/2-y)*scale
    def rect(centre,size,colour,stroke='#405060',opacity=1,yaw=0):
        x,y=xy(*centre); sx,sy=(v*scale for v in size)
        return el('rect',x=x-sx/2,y=y-sy/2,width=sx,height=sy,fill=colour,
                  stroke=stroke,opacity=opacity,transform=f'rotate({-yaw*180/3.141592653589793} {x} {y})')
    def label(x,y,text,size=11,colour='#182c40'):
        x,y=xy(x,y)
        el('text',x=x,y=y,font_family='sans-serif',font_size=size,fill=colour,
           text_anchor='middle',dominant_baseline='middle').text=text
    el('rect',width='100%',height='100%',fill='white')
    rect((0,0),(length,width),'#edf0f2',stroke='#26394b')
    for door in c['building']['doors']:
        vertical=door['wall'] in ('east','west')
        sign=-1 if door['wall'] in ('west','south') else 1
        centre=(sign*length/2,door['centre']) if vertical else (door['centre'],sign*width/2)
        rect(centre,(.15,door['width']) if vertical else (door['width'],.15),'white','white')
    for aisle in c['navigation']['aisles']:
        rect(aisle['centre'],aisle['size'],'#fff0ad','#e1c957',.55)
    for i,x in enumerate(c['racks']['row_x']):
        for j,y in enumerate(c['racks']['bay_y']):
            rect((x,y),c['racks']['size'][:2],'#6894bd','#325477')
            label(x,y,f'R{i+1}.{j+1}',9,'white')
    for item in c['floor_loads']:
        rect(item['pose'][:2],c['pallet']['size'][:2],'#bc9760',yaw=item['pose'][5])
    for p in c['building']['pillars']['positions']:
        rect(p,c['building']['pillars']['size'],'#38414a')
    for item in c['stations']:
        colour='#'+''.join(f'{round(v*255):02x}' for v in c['appearance'][item['colour']][:3])
        rect(item['pose'][:2],item['size'],colour,colour,.25,item['pose'][5])
        label(*item['pose'][:2],item['label'],9)
        x,y=xy(*item.get('goal_pose',item['pose'])[:2])
        el('circle',cx=x,cy=y,r=4,fill=colour)
    for human in c['humans']:
        points=[human['pose'][:2],*human['route']]
        el('polyline',points=' '.join(f'{x},{y}' for x,y in map(lambda p:xy(*p),points)),
           fill='none',stroke='#c14d39',stroke_width=2,stroke_dasharray='5 4')
        x,y=xy(*human['pose'][:2]); el('circle',cx=x,cy=y,r=6,fill='#c14d39')
        label(human['pose'][0],human['pose'][1]-.5,human['id'],9,'#9b3826')
    for spawn in c['spawns']:
        rect(spawn['pose'][:2],(amr['chassis_length'],amr['chassis_width']),
             '#168478' if spawn['active'] else '#bdd4d1',yaw=spawn['pose'][5])
    cam=c['camera']; x,y=xy(*cam['pose'][:2])
    el('circle',cx=x,cy=y,r=5,fill='#8044b0'); label(cam['pose'][0],cam['pose'][1]-.5,'CAM 01',9)
    label(0,0,'MAIN AISLE: 4 m / 3 AMRs',12)
    label(0,width/2+1.1,f'WAREHOUSE {length:g} m x {width:g} m — NORTH (+Y)',17)
    label(0,-width/2-1,'Blue: racks | Brown: loads | Red: people/routes | Dots: station goals',12)
    label(0,-width/2-1.8,'Origin: floor centre (0, 0, 0). East: +X. Normal scenario; schematic plan.',12)
    ET.indent(root)
    Path(output).write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))


if __name__ == '__main__':
    root=Path(__file__).resolve().parents[1]
    from generate_warehouse import load_config
    config,amr=load_config()
    render(config,root/'config/warehouse_layout.svg',amr)
