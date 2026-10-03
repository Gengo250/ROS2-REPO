#!/usr/bin/env python3
"""Offline acceptance: generation, freshness, SDF resolution and scoped names."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from generate_warehouse import ROOT, generate, load_config


def validate_tree(path):
    tree = ET.parse(path)
    # Names are scoped in SDF; repeated 'body' in distinct models is valid.
    def walk(element):
        seen = set()
        for child in element:
            if 'name' in child.attrib:
                key = (child.tag, child.attrib['name'])
                if key in seen:
                    raise AssertionError(f'{path}: duplicate {key}')
                seen.add(key)
            walk(child)
    walk(tree.getroot())
    for pose in tree.findall('.//pose'):
        assert len(pose.text.split()) == 6, str(path)+': invalid pose'
    for uri in tree.findall('.//uri'):
        assert not uri.text.startswith(('http:', 'https:')), 'Network asset dependency'
    return tree


def run(root=ROOT, check_sdf=True):
    root = Path(root)
    c, amr = load_config(root/'config/warehouse_layout.json')
    manifest = json.loads((root/'config/generated_manifest.json').read_text())
    checked = []
    for relative, expected in manifest['files'].items():
        path = root/relative
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, 'Stale/generated file changed: '+relative
    with tempfile.TemporaryDirectory(prefix='warehouse_regenerate_') as temp:
        fresh = generate(root/'config/warehouse_layout.json', Path(temp))
        assert fresh == manifest, 'Configuration/generator changed: regenerate warehouse'
        second = generate(root/'config/warehouse_layout.json', Path(temp))
        assert fresh == second, 'Generator is not idempotent'
    env = {**os.environ, 'SDF_PATH':str(root/'models'),
           'GZ_SIM_RESOURCE_PATH':str(root/'models')+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')}
    for path in sorted(root/name for name in manifest['files'] if name.endswith('.sdf')):
        validate_tree(path)
        if check_sdf:
            result = subprocess.run(['gz','sdf','--check',str(path)],env=env,text=True,capture_output=True,timeout=45)
            assert result.returncode == 0 and 'Error' not in result.stderr, result.stdout+result.stderr
            assert 'Warning' not in result.stderr, result.stderr
        checked.append(str(path.relative_to(root)))
    report = {'status':'passed','deterministic':True,'idempotent':True,'network_assets':False,
              'manifest_sha256':hashlib.sha256((root/'config/generated_manifest.json').read_bytes()).hexdigest(),
              'sdf_files':checked, 'counts':{k:manifest[k] for k in ('racks','pallets_normal','boxes_normal','humans')},
              'turning_diameter_with_clearance':manifest['turning_diameter_with_clearance'],
              'aisle_widths':{v['id']:min(v['size']) for v in c['navigation']['aisles']}}
    (root/'validation').mkdir(exist_ok=True)
    (root/'validation/static.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    args = parser.parse_args()
    run(args.root)
