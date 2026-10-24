#!/usr/bin/env python3
"""Read-only existing Gazebo worker/wall contact sensors; scoped coverage only."""
import argparse,json,signal,time
from pathlib import Path
from threading import Lock
from gz.transport13 import Node
from gz.msgs10.contacts_pb2 import Contacts
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=300);a=p.parse_args()
n=Node();lock=Lock();events=[];counts={};running=True
sources=['human_01','human_02','human_03','wall']
def receive(name,msg):
 with lock:
  counts[name]=counts.get(name,0)+1
  for c in msg.contact:
   if 'warehouse_amr' in c.collision1 or 'warehouse_amr' in c.collision2:
    events.append({'wall':time.time(),'sensor':name,'collision1':c.collision1,'collision2':c.collision2})
subscriptions={name:n.subscribe(Contacts,'/warehouse/contacts/'+name,lambda m,k=name:receive(k,m)) for name in sources}
def stop(*_):
 global running
 running=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
end=time.monotonic()+a.duration
try:
 while running and time.monotonic()<end:time.sleep(.1)
finally:
 with lock:a.output.write_text(json.dumps({'subscriptions':subscriptions,'message_counts':counts,'amr_contact_samples':len(events),'events':events,'coverage':'Only the existing named worker/wall contact sensors; not a whole-world collision count.'},indent=2)+'\n')
