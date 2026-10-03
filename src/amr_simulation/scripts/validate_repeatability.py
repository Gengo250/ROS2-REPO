#!/usr/bin/env python3
"""Repeat a clean crossing scene and compare physical trajectories in sim time."""
from bisect import bisect_left
import json
import math

from generate_warehouse import ROOT
from run_acceptance import run


def interpolate(samples, name, time):
    points=[(s['time'],s[name]['position']) for s in samples if name in s]
    i=bisect_left([t for t,_ in points],time)
    assert 0<i<len(points),'Time outside recorded trajectory'
    t0,p0=points[i-1]; t1,p1=points[i]
    alpha=(time-t0)/(t1-t0)
    return [p0[k]+alpha*(p1[k]-p0[k]) for k in ('x','y','z')]


def validate():
    base=ROOT/'validation'
    original=json.loads((base/'person_crossing/runtime.json').read_text())
    run(['person_crossing'],base/'repeat')
    repeated=json.loads((base/'repeat/person_crossing/runtime.json').read_text())
    assert original['manifest_sha256']==repeated['manifest_sha256'],'Different generated inputs'
    start=max(original['human_trajectories'][0]['time'],repeated['human_trajectories'][0]['time'])+1
    end=min(original['human_trajectories'][-1]['time'],repeated['human_trajectories'][-1]['time'])-1
    assert end-start>15,'Insufficient shared trajectory observation'
    times=[start+i*.25 for i in range(int((end-start)/.25))]
    results={}
    for name in ('human_01','human_02'):
        errors=[math.dist(interpolate(original['human_trajectories'],name,t),
                          interpolate(repeated['human_trajectories'],name,t)) for t in times]
        results[name]={'max_position_difference_m':max(errors),'mean_position_difference_m':sum(errors)/len(errors),
                       'compared_sim_seconds':end-start,'samples':len(errors)}
    passed=all(r['max_position_difference_m']<.05 for r in results.values())
    report={'status':'passed' if passed else 'failed','manifest_sha256':original['manifest_sha256'],
            'method':'Two independent worlds, linear interpolation at common simulation times; no person contact',
            'tolerance_m':.05,'humans':results}
    (base/'repeatability.json').write_text(json.dumps(report,indent=2)+'\n')
    assert passed,report
    print('PHYSICAL HUMAN REPEATABILITY PASSED:',results,flush=True)


if __name__=='__main__':
    validate()
