"""Select a physically constrained rescue using prefix purged OOF scores only."""
from pathlib import Path
import os,json,pickle,itertools,sys
os.environ['OMP_NUM_THREADS']='1'
import numpy as np
import pandas as pd
P=Path(__file__).resolve().parent;ROOT=P.parents[1];OUT=ROOT/'output/pole_geometry_20260927'
sys.path.insert(0,str(ROOT/'research/pole_geometry_20260927'))
from trainGeometryModel import load
rows,den,names,X,ds,frames,labels,y,ix,groups,dev=load(True,True)
bundle=pickle.load((OUT/'stable_moments_compact_d5_w1.0.pickle').open('rb'))
score=bundle['model'].predict_proba(X[:,[names.index(n) for n in bundle['names']]])[:,1]
oof=np.load(OUT/'stable_moments_compact_d5_w1.0_oof.npy')
mask=(ds=='Mississippi');prefix=mask&(frames<=780)
# Work at unique-owner level, so repeated hypotheses never duplicate labels.
g=groups[prefix];target=labels[ix];base=np.zeros(len(ix),bool);np.logical_or.at(base,g,oof[prefix]>=.8701683227450074)
F={n:X[prefix,names.index(n)] for n in ['supportHeight','longestHeight','radialRms','isolation','ownerCount','ownerHeight','minimumQuarterCount','fraction010','quarterCenterStep']}
results=[]
for floor,height,concentration,quarter,step in itertools.product([.6,.7,.75,.8,.82,.84],[1.5,2,2.5,3],[.5,.7,.85,.95],[2,3,4,6],[.06,.1,.15,.2]):
    rescue=(oof[prefix]>=floor)&(F['supportHeight']>=height)&(F['radialRms']<=.12)&(F['isolation']>=.7)&(F['longestHeight']>=.5*F['supportHeight'])&(F['ownerCount']>=5)&(F['ownerHeight']>=.5)&(F['minimumQuarterCount']>=quarter)&(F['fraction010']>=concentration)&(F['quarterCenterStep']<=step)
    own=base.copy();np.logical_or.at(own,g,rescue);new=own&~base;count=new.sum();fp=(target[new]==0).sum()
    if count<10 or fp/max(count,1)>.1:continue
    selected=own.sum();bad=(target[own]==0).sum()
    if bad/selected>.08:continue
    results.append(dict(floor=floor,height=height,radial=.12,isolation=.7,continuity=.5,concentration=concentration,quarter=quarter,step=step,added=int(count),addedEmpty=int(fp),addedPoints=int(target[new].sum()),total=int(selected),empty=int(bad)))
assert results,'No safe rescue on prefix OOF'
results.sort(key=lambda r:(-r['addedPoints'],r['addedEmpty'],-r['floor'],r['radial']))
r=results[0];(P/'concentration_selection.json').write_text(json.dumps(dict(chosen=r,eligible=len(results),selection='Prefix <=780 existing five-block20-frame-purged OOF; incremental false <=10%, pooled false <=8%; maximize reference points; no suffix selection',alternatives=results[:20]),indent=2)+'\n')
print('FROZEN RULE',r,flush=True)
# Freeze first; only then evaluate final fitted scores and the reused suffix.
f=lambda n:X[:,names.index(n)]
rescue=(score>=r['floor'])&(f('supportHeight')>=r['height'])&(f('radialRms')<=r['radial'])&(f('isolation')>=r['isolation'])&(f('longestHeight')>=r['continuity']*f('supportHeight'))&(f('ownerCount')>=5)&(f('ownerHeight')>=.5)&(f('minimumQuarterCount')>=r['quarter'])&(f('fraction010')>=r['concentration'])&(f('quarterCenterStep')<=r['step'])
base=np.zeros(len(ix),bool);new=base.copy();np.logical_or.at(base,groups,score>=.8701683227450074);np.logical_or.at(new,groups,(score>=.8701683227450074)|rescue)
metrics=[]
for label,part in [('prefix',frames[ix]<=780),('suffix',frames[ix]>780),('all',np.ones(len(ix),bool))]:
    part&=ds[ix]=='Mississippi';total=sum(int(a['finePoints']) for a in den if a['dataset']=='Mississippi' and (label=='all' or (int(a['frame'])<=780)==(label=='prefix')))
    for name,sel in [('baseline',base),('recovery',new)]:
        a=part&sel;metrics.append(dict(part=label,variant=name,selected=int(a.sum()),empty=int((target[a]==0).sum()),covered=int(target[a].sum()),fine=total,falseFraction=float((target[a]==0).sum()/a.sum()),coverage=float(target[a].sum()/total)))
pd.DataFrame(metrics).to_csv(P/'concentration_metrics.csv',index=False)
pred=pd.DataFrame([dict(frame=int(rows[j]['frame']),pillar=int(rows[j]['pillar']),finePointCount=int(target[k]),baseline=bool(base[k]),selected=bool(new[k])) for k,j in enumerate(ix) if ds[j]=='Mississippi'])
pred.to_csv(ROOT/'output/pole_selective_recovery_20260928/concentration_expected.csv',index=False)
print(pd.DataFrame(metrics).to_string(index=False));print(pred[(pred.frame==857)&pred.selected].to_string(index=False))
