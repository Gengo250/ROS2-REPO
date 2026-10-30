#!/usr/bin/env python3
"""Read-only existing Gazebo worker/wall contact sensors; scoped coverage only."""
import argparse,json,signal,time
from pathlib import Path
from threading import Lock
from gz.transport13 import Node
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.stringmsg_pb2 import StringMsg
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=300);a=p.parse_args()
n=Node();lock=Lock();events=[];counts={};running=True
sources=['human_01','human_02','human_03','wall','barrier']
def receive(name,msg):
 with lock:
  counts[name]=counts.get(name,0)+1
  for c in msg.contact:
   if 'warehouse_amr' in c.collision1.name or 'warehouse_amr' in c.collision2.name:
    events.append({'wall':time.time(),'sim':msg.header.stamp.sec+msg.header.stamp.nsec/1e9,'sensor':name,'collision1':c.collision1.name,'collision2':c.collision2.name})
subscriptions={name:n.subscribe(Contacts,'/warehouse/contacts/'+name,lambda m,k=name:receive(k,m)) for name in sources}
# Pedestrian yielding state published by HumanSystem (simulation-side evaluation only).
yields={}
def yielded(name,msg):
 with lock:
  try:yields.setdefault(name,[]).append(json.loads(msg.data))
  except ValueError:pass
yield_subscriptions={name:n.subscribe(StringMsg,'/warehouse/humans/'+name+'/yield',lambda m,k=name:yielded(k,m)) for name in ('human_01','human_02')}
def stop(*_):
 global running
 running=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
end=time.monotonic()+a.duration
try:
 while running and time.monotonic()<end:time.sleep(.1)
finally:
 with lock:
  episodes=[];last_by_pair={}
  for event in events:
   key=(event['sensor'],event['collision1'],event['collision2'])
   if key not in last_by_pair or event['sim']-last_by_pair[key]>.1:episodes.append(event)
   last_by_pair[key]=event['sim']
  summary={k:{'events':v[-1]['events'],'total_yield_s':v[-1]['total_yield_s'],'reversals':v[-1]['reversals'],
              'min_envelope_distance_m':min(x['envelope_distance_m'] for x in v),'samples':len(v)} for k,v in yields.items() if v}
  a.output.write_text(json.dumps({'subscriptions':subscriptions,'yield_subscriptions':yield_subscriptions,'yielding':summary,'yield_samples':yields,'message_counts':counts,'amr_contact_samples':len(events),'contact_episodes':len(episodes),'episode_definition':'Same sensor/collision pair with gaps greater than 0.1 simulation seconds starts a new episode.','events':events,'coverage':'Only existing named worker/south-wall/barrier contact sensors; not a whole-world collision count.'},indent=2)+'\n')
