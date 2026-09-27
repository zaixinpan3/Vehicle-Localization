"""Development sweep of confidence-conditional distribution gates."""
import csv,json
from pathlib import Path
import numpy as np
from analyzeContext import metric
P=Path(__file__).resolve().parent
path=P/'context_continuity_features.csv'
if not path.exists():path=P/'expanded_features.csv'
r=list(csv.DictReader(path.open()));d={k:np.array([float(x[k]) for x in r]) for k in r[0]}
_,i,g=np.unique([f"{x['frame']}:{x['pillar']}" for x in r],return_index=True,return_inverse=True);fine=d['finePointCount'][i]
t=list(csv.DictReader((P/'development_frames.csv').open()));den=sum(int(x['finePoints']) for x in t);pden=sum(int(x['finePillars']) for x in t)
rows=[]
def evaluate(rule,keep):
 m=metric(keep,g,fine,den,pden)
 if m['coverage']>=.93 and m['recall']>=.84:rows.append(dict(rule=rule,**m))
for count in [15,20,25,30,40,50,75,100]:
 for tilt in [2,3,4,5,6,7,8]:
  for rms in [.06,.08,.10,.12,.15,.20]:
   keep=(d['ownerCount']>=count)|((d['tilt']<=tilt)&(d['radialRms']<=rms))
   evaluate(f'owner>={count} OR (tilt<={tilt} AND rms<={rms})',keep)
for name in d:
 if not name.startswith('contextHeight'):continue
 for x in np.arange(.1,2.01,.1):
  evaluate(f'{name}<={x:.1f}',d[name]<=x)
  for s in [.06,.08,.1]:evaluate(f'{name}<={x:.1f} OR rms<={s}',(d[name]<=x)|(d['radialRms']<=s))
rows.sort(key=lambda r:(r['extra'],-r['coverage']))
with (P/'conditional_gates.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
print(json.dumps(rows[:15],indent=2))
