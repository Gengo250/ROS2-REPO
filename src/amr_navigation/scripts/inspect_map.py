#!/usr/bin/env python3
"""Validate a saved occupancy image; never creates or repairs map pixels."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('map',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
c=yaml.safe_load(a.map.read_text());impath=a.map.parent/c['image'];im=Image.open(impath).convert('L');hist=im.histogram()
assert c['resolution']>0 and len(c['origin'])==3
assert 0<c['free_thresh']<c['occupied_thresh']<1
assert im.width>50 and im.height>50 and hist[0]>100 and hist[254]>100
r={'yaml':c,'width':im.width,'height':im.height,'image_sha256':hashlib.sha256(impath.read_bytes()).hexdigest(),
   'image_value_counts':{str(i):n for i,n in enumerate(hist) if n},'extent_m':[im.width*c['resolution'],im.height*c['resolution']],
   'note':'Image integrity only. Coverage and geometric accuracy require sensor and physical validation.'}
a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
