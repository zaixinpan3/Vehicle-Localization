"""Screen physically interpretable crop-completion gates on development only."""
import csv,json
from pathlib import Path
import numpy as np
from analyzeContext import metric
P=Path(__file__).resolve().parent
r=list(csv.DictReader((P/'expanded_features.csv').open()))
d={k:np.array([float(x[k]) for x in r]) for k in r[0]}
keys=np.array([f"{int(x['frame'])}:{int(x['pillar'])}" for x in r])
_,i,g=np.unique(keys,return_index=True,return_inverse=True);fine=d['finePointCount'][i]
t=list(csv.DictReader((P/'development_frames.csv').open()))
den=sum(int(x['finePoints']) for x in t);pden=sum(int(x['finePillars']) for x in t)
rows=[]
def evaluate(name,keep):
 m=metric(keep,g,fine,den,pden)
 if m['coverage']>=.9 and m['recall']>=.80: rows.append(dict(rule=name,**m))
for radius in ['30','35','40','50','60','75']:
 for std in [.09,.1,.11,.12,.13,.14,.15,.18,.2]:
  for aspect in [1,1.5,2,2.5,3,4,5]:
   keep=(d['std'+radius]<=std)|(d['aspect'+radius]<=aspect)
   evaluate(f'width({radius},{std},{aspect})',keep)
for name in d:
 if name in ['frame','pillar','hypothesis','finePointCount']:continue
 for op in ['le','ge']:
  for x in np.unique(np.quantile(d[name],np.linspace(0,1,51))):
   evaluate(f'{name} {op} {x:.5g}', d[name]<=x if op=='le' else d[name]>=x)
rows.sort(key=lambda r:(r['extra'],-r['coverage']))
with (P/'expanded_gates.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
print(json.dumps(rows[:15],indent=2))
print('WIDTH',json.dumps([x for x in rows if x['rule'].startswith('width')][:12],indent=2))
