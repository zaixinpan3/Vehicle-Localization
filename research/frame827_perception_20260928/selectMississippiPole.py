"""Calibrate a Mississippi-only precision budget using existing purged scores."""
from pathlib import Path
import json
import numpy as np,pandas as pd
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent
cache=ROOT/'output/pole_geometry_20260927'
m=pd.read_csv(cache/'mississippi_features.csv',usecols=['dataset','frame','pillar','finePointCount']);d=pd.read_csv(cache/'downtown_features.csv',usecols=['dataset','frame','pillar','finePointCount'])
t=pd.concat([m,d],ignore_index=True).assign(score=np.load(cache/'stable_moments_compact_d5_w1.0_oof.npy'));t=t[(t.dataset=='Mississippi')&(t.frame<=780)].groupby(['frame','pillar']).agg(score=('score','max'),finePointCount=('finePointCount','max')).sort_values('score',ascending=False)
n=np.arange(1,len(t)+1);f=(t.finePointCount==0).cumsum().to_numpy();covered=t.finePointCount.cumsum().to_numpy();last=np.r_[t.score.to_numpy()[:-1]!=t.score.to_numpy()[1:],True];choices=np.flatnonzero((f/n<=.075)&last);a=choices[np.argmax(covered[choices])];threshold=float(t.score.iloc[a])
p=pd.read_csv(ROOT/'research/pole_geometry_20260927/stable_moments_model_predictions.csv');p=p[p.dataset=='Mississippi'];den=pd.read_csv(ROOT/'research/pole_precision_20260927/mississippi_frames.csv');result={}
for label,score in [('before',.9371209151674477),('after',threshold)]:
 r={}
 for name,mask,dm in [('suffix',p.frame>780,den.frame>780),('all',p.frame>0,den.frame>0),('frame827',p.frame==827,den.frame==827)]:
  take=p[mask&(p.score>=score)];total=int(den.loc[dm,'finePoints'].sum());false=int((take.finePointCount==0).sum());points=int(take.finePointCount.sum());r[name]=dict(selected=len(take),false=false,falseFraction=false/len(take) if len(take) else None,coveredPoints=points,referencePoints=total,coverage=points/total)
 result[label]=r
report=dict(threshold=threshold,prefixOOF=dict(selected=int(n[a]),false=int(f[a]),falseFraction=float(f[a]/n[a]),coveredPoints=int(covered[a]),referencePoints=124708),validation=result,policy='Mississippi only; maximize reference coverage at <=7.5% prefix OOF reference-empty fraction. Other profiles keep previous threshold. Reused suffix and frame827 are development checks, not independent testing.')
(DEST/'pole_calibration.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
