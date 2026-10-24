#!/usr/bin/env python3
"""Evaluate recorded estimates against timestamp-aligned world poses, offline."""
import argparse,json,math,statistics
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('result',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=json.loads(a.result.read_text());rows=r['trajectory'];world={x['world_stamp']:x['world'] for x in rows if x.get('world') and x.get('world_stamp') is not None};stamps=sorted(world)
errors=[];angles=[]
def wrap(v):return math.atan2(math.sin(v),math.cos(v))
for row in rows:
 stamp=row.get('map_stamp')
 if row.get('map') is None or stamp is None:continue
 import bisect
 i=bisect.bisect_left(stamps,stamp)
 if i==0 or i==len(stamps):continue
 lo,hi=stamps[i-1:i+1]
 if hi-lo>.6:continue
 alpha=(stamp-lo)/(hi-lo);x0,y0,q0=world[lo];x1,y1,q1=world[hi]
 truth=[x0+alpha*(x1-x0),y0+alpha*(y1-y0),q0+alpha*wrap(q1-q0)]
 # Explicit initial registration validated at the unchanged warehouse spawn.
 errors.append(math.hypot(row['map'][0]-(truth[0]+11.),row['map'][1]-truth[1]))
 angles.append(abs(wrap(row['map'][2]-truth[2])))
sim=rows[-1]['sim']-rows[0]['sim'];wall=rows[-1]['wall']-rows[0]['wall']
result={k:r.get(k) for k in ['accepted','passed','status','error_code','wall_time_s','sim_time_s','goal_position_error_m','goal_orientation_error_rad','first_planned_path_length_m','world_travel_distance_m','odom_travel_distance_m','map_travel_distance_m','replans','path_publications','recoveries','acceptance_latency_s','stopped','collision_events']}
result.update(aligned_samples=len(errors),registration={'map_from_world':[11.,0.,0.],'source':'Initial real SLAM scan at verified existing world spawn; validation only'},simulation_real_time_factor=sim/wall if wall else None)
if errors:result.update(localization_rmse_m=math.sqrt(statistics.mean(v*v for v in errors)),localization_max_error_m=max(errors),localization_final_error_m=errors[-1],localization_yaw_mean_rad=statistics.mean(angles))
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
