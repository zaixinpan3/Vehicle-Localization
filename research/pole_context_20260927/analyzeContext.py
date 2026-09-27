"""Development-only geometry screening with exact owner-union point metrics."""
import csv,json
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parent

def load():
 with (P/'development_features.csv').open() as f:
  r=list(csv.DictReader(f))
 data={k:np.array([float(x[k]) for x in r]) for k in r[0]}
 keys=np.array([f"{int(x['frame'])}:{int(x['pillar'])}" for x in r])
 owners,index,groups=np.unique(keys,return_index=True,return_inverse=True)
 fine=data['finePointCount'][index];total=list(csv.DictReader((P/'development_frames.csv').open()))
 denom=sum(int(x['finePoints']) for x in total);pillarDenom=sum(int(x['finePillars']) for x in total)
 return data,index,groups,fine,denom,pillarDenom

def metric(keep,groups,fine,denom,pillarDenom):
 selected=np.unique(groups[keep]);matched=int(np.count_nonzero(fine[selected]));points=fine[selected].sum()
 return dict(selected=len(selected),matched=matched,extra=len(selected)-matched,points=int(points),
             coverage=float(points/denom),recall=float(matched/pillarDenom),precision=float(matched/max(len(selected),1)))

def main():
 d,i,g,f,den,pden=load();features=[k for k in d if k not in ['frame','pillar','hypothesis','finePointCount']]
 output=[]
 for name in features:
  for direction in ['>=','<=']:
   values=d[name];valid=np.isfinite(values)
   for threshold in np.unique(np.quantile(values[valid],np.linspace(0,1,101))):
    keep=valid & ((values>=threshold) if direction=='>=' else (values<=threshold))
    m=metric(keep,g,f,den,pden)
    if m['recall']>=.8 and m['coverage']>=.90:
     output.append(dict(feature=name,direction=direction,threshold=float(threshold),**m))
 baseline=metric(np.ones(len(g),bool),g,f,den,pden)
 stable=(d['tilt']<=4)&(d['radialRms']<=.10)
 masks={'baseline':np.ones(len(g),bool),'sparse_shaft_only':(d['acceptedCount']>=30)|stable,
        'weak_owner_only':(d['ownerCount']>=15)|stable,
        'fixed_count_combined':((d['acceptedCount']>=30)&(d['ownerCount']>=15))|stable,
        'blanket_owner_20':d['ownerCount']>=20}
 ranges=list(csv.DictReader((P/'development_ranges.csv').open()))
 assert all(int(r['frame'])==d['frame'][j] and int(r['pillar'])==d['pillar'][j] and int(r['hypothesis'])==d['hypothesis'][j] for j,r in enumerate(ranges))
 scale=np.minimum(1,(10/np.maximum([float(r['axisRange']) for r in ranges],1e-12))**2)
 masks['count_scaled_combined']=((d['acceptedCount']>=30*scale)&(d['ownerCount']>=15*scale))|stable
 distance=np.array([float(r['axisRange']) for r in ranges])
 blend=np.clip((distance-15)/5,0,1)
 stable_range=(d['tilt']<=4+2*blend)&(d['radialRms']<=.10+.02*blend)
 masks['range_geometry_combined']=((d['acceptedCount']>=30)&(d['ownerCount']>=15))|stable_range
 ablation=[dict(variant=k,**metric(v,g,f,den,pden)) for k,v in masks.items()]
 with (P/'ablation_development.csv').open('w') as stream:
  writer=csv.DictWriter(stream,fieldnames=list(ablation[0]),lineterminator='\n');writer.writeheader();writer.writerows(ablation)

 output.sort(key=lambda x:(x['extra'],-x['coverage']))
 with (P/'single_gates.csv').open('w') as stream:
  writer=csv.DictWriter(stream,fieldnames=list(output[0]),lineterminator='\n');writer.writeheader();writer.writerows(output)
 best={str(c):next((v for v in output if v['coverage']>=c),None) for c in [.90,.93,.95]}
 result=dict(baseline=baseline,best=best)
 (P/'single_gate_summary.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))

if __name__=='__main__':main()
