"""Rejected general geometric rescue sweep on reused Mississippi data."""
import pandas as pd,numpy as np,itertools
x=pd.read_csv('output/pole_geometry_20260927/mississippi_features.csv');b=pd.read_csv('research/pole_selective_recovery_20260928/specialist_expected.csv');p=np.loadtxt('output/pole_selective_recovery_20260928/inference_expected.csv',delimiter=',');o=np.load('output/pole_selective_recovery_20260928/oof_d5_l10_w1.npy');u=pd.MultiIndex.from_frame(x[['frame','pillar']]);_,ix,g=np.unique(u,return_index=True,return_inverse=True);y=x.finePointCount.to_numpy()[ix];frames=x.frame.to_numpy()[ix];base=np.zeros(len(ix),bool);np.logical_or.at(base,g,p>=.8747229048074372)
baseo=np.zeros(len(ix),bool);np.logical_or.at(baseo,g,o>=.8747229048074372)
fixed=(x.supportHeight>=1.5)&(x.longestHeight>=.9*x.supportHeight)&(x.isolation>=.95)&(x.minimumQuarterCount>=3)&(x.ownerCount>=5)&(x.ownerHeight>=.8)&(x.ownerSupportFraction>=.3)&(x.tilt<=5)&(x.ownerFraction>=.2)
res=[]
for rms,step,frac,gap,asp,floor in itertools.product([.08,.1],[.025,.05,.08],[.9,.94],[.2,.3,.5],[1.5,2,3],[0,.1,.2]):
 rule=fixed&(x.radialRms<=rms)&(x.quarterCenterStep<=step)&(x.fraction015>=frac)&(x.maximumGap<=gap)&(x.aspect<=asp)
 own=np.zeros(len(ix),bool);np.logical_or.at(own,g,rule&(p>=floor));a=own&~base;new=own|base
 oo=np.zeros(len(ix),bool);np.logical_or.at(oo,g,rule&(o>=floor));add=oo&~baseo&(frames<=780);keep=(oo|baseo)&(frames<=780)
 r=dict(rms=rms,step=step,frac=frac,gap=gap,asp=asp,floor=floor,oofAdd=int(add.sum()),oofBad=int((y[add]==0).sum()),oofPoints=int(y[add].sum()),oofFP=float((y[keep]==0).mean()),add=int(a.sum()),bad=int((y[a]==0).sum()),suffixFP=float((y[new&(frames>780)]==0).mean()),target=int(new[(frames==856)&np.isin(x.pillar.to_numpy()[ix],[7944,7945])].sum()))
 res.append(r)
pd.DataFrame(res).to_csv('research/pole_boundary_recovery_20260929/geometric_sweep.csv',index=False)
# Initial broad shape-only relaxation, before imposing split/clutter constraints.
r=(x.supportHeight>=1.5)&(x.longestHeight>=.9*x.supportHeight)&(x.radialRms<=.1)&(x.isolation>=.95)&(x.minimumQuarterCount>=3)&(x.fraction015>=.9)&(x.quarterCenterStep<=.08)&(x.ownerCount>=5)&(x.ownerHeight>=.8)&(x.tilt<=10)
own=np.zeros(len(ix),bool);np.logical_or.at(own,g,r);keep=base|own
import json
from pathlib import Path
Path('research/pole_boundary_recovery_20260929/broad_rejected.json').write_text(json.dumps(dict(selected=int(keep.sum()),empty=int((y[keep]==0).sum()),covered=int(y[keep].sum()),added=int((own&~base).sum())),indent=2)+'\n')
