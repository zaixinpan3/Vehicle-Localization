"""Select development-only high-confidence errors for geometric diagnosis."""
from pathlib import Path
import csv,pickle,json
import numpy as np
P=Path(__file__).resolve().parent;OUT=P.parents[1]/'output/pole_precision_20260927'
rows=[]
for dataset in ['mississippi','downtown']:rows+=list(csv.DictReader((P/(dataset+'_features.csv')).open()))
m=pickle.load((OUT/'gb3_160.pickle').open('rb'));X=np.nan_to_num(np.array([[float(r[k]) for k in m['names']] for r in rows]),nan=0,posinf=0,neginf=0)
score=np.load(OUT/'gb3_160_oof.npy');chosen={}
for dataset,limit in [('Mississippi',780),('Downtown',360)]:
 indices=[i for i,r in enumerate(rows) if r['dataset']==dataset and int(r['frame'])<=limit and int(r['finePointCount'])==0]
 indices.sort(key=lambda i:-score[i]);selected=[];used=[]
 for i in indices:
  r=rows[i];frame=int(r['frame'])
  if any(abs(frame-x)<25 for x in used):continue
  selected.append(dict(frame=frame,pillar=int(r['pillar']),score=float(score[i]),ownerCount=float(r['ownerCount']),height=float(r['supportHeight']),tilt=float(r['tilt']),rms=float(r['radialRms']),circleRadius=float(r['circleRadius'])));used.append(frame)
  if len(selected)==8:break
 chosen[dataset]=selected
(P/'diagnostic_examples.json').write_text(json.dumps(chosen,indent=2)+'\n');print(json.dumps(chosen,indent=2))
