#!/usr/bin/env python3
"""Additional physical contact test against moving human_01, without world edits."""
import json
import math
import os
from pathlib import Path
import subprocess
import time

from generate_warehouse import ROOT
from run_acceptance import simulation_processes, stop


def run():
    assert not simulation_processes(), 'Finish the previous launch first'
    out=ROOT/'validation/scenario_gui'
    os.environ.update(GZ_PARTITION='warehouse_human_contact_'+str(os.getpid()),
                      ROS_DOMAIN_ID='93',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    # Import transport before message modules and after choosing the partition.
    from validate_runtime import Acceptance
    from gz.msgs10.contacts_pb2 import Contacts
    import rclpy
    report={'status':'failed'}
    sim=None; a=None
    with (out/'human_01_contact.log').open('w') as log:
        try:
            sim=subprocess.Popen(['ros2','launch','amr_simulation','warehouse.launch.py',
                    'scenario:=person_crossing','gui:=false'],stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True)
            a=Acceptance('person_crossing',out)
            a.gz.subscribe(Contacts,'/warehouse/contacts/human_01',lambda msg:a.on_contact('human_01',msg))
            a.until(lambda:all(a.counts[t]>=3 for t in a.topics) and 'warehouse_amr' in a.poses)
            a.set_pose([-8.7,0,.015,0,0,-math.pi/2])
            a.until(lambda:a.contacts['human_01']>0,180)
            records=[]
            start=a.now()
            while a.now()-start<4:
                a.drive(0,0,.1)
                records.append({'time':a.now(),'robot':a.robot(),'human':a.poses['human_01']})
            gaps=[abs(x['human']['position']['y']-x['robot']['position']['y']) for x in records]
            signs=[math.copysign(1,x['human']['position']['y']-x['robot']['position']['y']) for x in records]
            assert min(gaps)>.60, min(gaps)
            assert len(set(signs))==1, 'Moving person traversed the AMR'
            report.update(status='passed',contact_events=a.contacts['human_01'],min_y_gap_m=min(gaps),
                          samples=records,manifest_sha256=a.manifest_digest,
                          method='AMR stopped across the original human_01 route; follower forces unchanged')
        finally:
            if a is not None:
                a.node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
            stop(sim)
            deadline=time.monotonic()+10
            while simulation_processes() and time.monotonic()<deadline:time.sleep(.2)
            report['remaining_processes']=simulation_processes()
            (out/'human_01_contact.json').write_text(json.dumps(report,indent=2)+'\n')
    assert not report['remaining_processes']
    print('MOVING HUMAN CONTACT PASS',report['contact_events'],report['min_y_gap_m'],flush=True)


if __name__=='__main__':
    run()
