#!/usr/bin/env python3
"""Check deterministic rebuild, non-AMR scene preservation and a parameter variant."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import bpy
sys.path.insert(0,str(Path(__file__).resolve().parent))
from generate_amr import ROOT, main


def hashes(root):
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((root/'meshes/visual').glob('*.stl'))}


def run():
    unrelated = bpy.data.scenes.new('AMR_Test_Unrelated_User_Scene')
    mesh = bpy.data.meshes.new('AMR_Test_User_Mesh')
    mesh.from_pydata([(10,20,30)],[],[])
    sentinel = bpy.data.objects.new('AMR_Test_User_Object',mesh)
    unrelated.collection.objects.link(sentinel)
    with tempfile.TemporaryDirectory(prefix='amr_rebuild_') as temp:
        root = Path(temp)
        (root/'config').mkdir()
        source = (ROOT/'config/amr_dimensions.json').read_text()
        (root/'config/amr_dimensions.json').write_text(source)
        main(root,renders=False)
        first = hashes(root)
        first_bytes = {path.name:path.read_bytes() for path in (root/'meshes/visual').glob('*.stl')}
        object_count = len(bpy.data.scenes['AMR_Generation'].objects)
        main(root,renders=False)
        for path in (root/'meshes/visual').glob('*.stl'):
            old,new = first_bytes[path.name],path.read_bytes()
            if old != new:
                differing = [i for i,(a,b) in enumerate(zip(old,new)) if a != b]
                print('STL DIFFERENCE',path.name,len(old),len(new),'changed bytes',len(differing),
                      'first offsets',differing[:20],flush=True)
        assert first == hashes(root),'Same parameters produced different STL bytes'
        assert len(bpy.data.scenes['AMR_Generation'].objects) == object_count
        assert unrelated.objects.get(sentinel.name) == sentinel
        assert tuple(sentinel.data.vertices[0].co) == (10,20,30)
        alternative = json.loads(source)
        alternative.update(chassis_length=1.0,chassis_width=0.74,chassis_height=0.26,
                           platform_length=0.92,platform_width=0.67,track_width=0.60,wheel_radius=0.125)
        (root/'config/amr_dimensions.json').write_text(json.dumps(alternative,indent=2))
        main(root,renders=False)
        assert hashes(root)['chassis.stl'] != first['chassis.stl']
        assert hashes(root)['wheel.stl'] != first['wheel.stl']
        report = {'status':'passed','identical_stl_bytes_on_rebuild':True,
                  'unrelated_scene_preserved':True,'generated_object_count':object_count,
                  'alternative_dimensions_passed':{key:alternative[key] for key in
                      ('chassis_length','chassis_width','chassis_height','platform_length','platform_width','track_width','wheel_radius')}}
        (ROOT/'validation/rebuild.json').write_text(json.dumps(report,indent=2)+'\n')
        print('REBUILD TEST PASSED',json.dumps(report))


if __name__ == '__main__':
    run()
