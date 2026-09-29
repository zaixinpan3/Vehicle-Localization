"""Audit a targeted split-shaft recovery; this route is reused development data."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent
x=pd.read_csv(ROOT/'output/pole_geometry_20260927/mississippi_features.csv')
b=pd.read_csv(ROOT/'research/pole_selective_recovery_20260928/specialist_expected.csv')
thresholds=dict(minimumHeight=1.5,minimumContinuousFraction=.9,maximumRadialRms=.1,minimumIsolation=.95,minimumQuarterCount=3,minimumCoreFraction=.9,maximumQuarterCenterStep=.05,maximumGap=.3,maximumAspect=2,maximumTilt=5,minimumOwnerCount=5,minimumOwnerHeight=1,minimumOwnerFraction=.2,minimumOwnerSupportFraction=.3,maximumOwnerSupportFraction=.8,maximumOwnerHeightFraction=.8,maximumAxisOwnerDistance=.1,minimumOwners=2)
(P/'recovery_config.json').write_text(json.dumps(thresholds,indent=2)+'\n')
h=pd.MultiIndex.from_frame(x[['frame','hypothesis']]);_,hi,g=np.unique(h,return_index=True,return_inverse=True)
own=(x.ownerCount>=5)&(x.ownerHeight>=1)&(x.ownerFraction>=.2)&(x.ownerSupportFraction>=.3)&(x.ownerSupportFraction<=.8)&(x.ownerHeight<=.8*x.ownerTotalHeight)&(x.axisOwnerDistance<=.1)
physical=(x.supportHeight>=1.5)&(x.longestHeight>=.9*x.supportHeight)&(x.isolation>=.95)&(x.minimumQuarterCount>=3)&(x.tilt<=5)&(x.radialRms<=.1)&(x.quarterCenterStep<=.05)&(x.fraction015>=.9)&(x.maximumGap<=.3)&(x.aspect<=2)
count=np.bincount(g,weights=own,minlength=len(hi));row=physical&own&(count[g]>=2)
keys=pd.MultiIndex.from_frame(x.loc[row,['frame','pillar']]);b['recovered']=pd.MultiIndex.from_frame(b[['frame','pillar']]).isin(keys);b=b.rename(columns={'selected':'baseline'});b['selected']=b.baseline|b.recovered
b.to_csv(P/'expected.csv',index=False);b[b.selected&~b.baseline].to_csv(P/'added_pillars.csv',index=False)
rows=[];den=pd.read_csv(ROOT/'research/pole_precision_20260927/mississippi_frames.csv')
for name,mask,dm in [('all',b.frame>0,den.frame>0),('prefix',b.frame<=780,den.frame<=780),('suffix',b.frame>780,den.frame>780)]:
 for variant in ['baseline','selected']:
  a=b[mask&b[variant]];n=int(den.loc[dm,'finePoints'].sum());rows.append(dict(part=name,variant=variant,selected=len(a),empty=int((a.finePointCount==0).sum()),falseFraction=float((a.finePointCount==0).mean()),covered=int(a.finePointCount.sum()),fine=n,coverage=float(a.finePointCount.sum()/n)))
pd.DataFrame(rows).to_csv(P/'metrics.csv',index=False);print(pd.DataFrame(rows).to_string(index=False));print(b[b.selected&~b.baseline].to_string(index=False))
