"""Independently audit owner coverage and export route comparison."""
from pathlib import Path
import json,hashlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent;ROOT=P.parents[1]
a=pd.read_csv(ROOT/'research/current_matching_20260929/full_route.csv');b=pd.read_csv(P/'full_route.csv');raw=pd.read_csv(P/'deployed_pole_replay.csv');m=pd.read_csv(P/'metrics.csv');target=m[(m.part=='all')&(m.variant=='selected')].iloc[0]
assert len(raw)==len(b)==1170
for col,field in [('candidatePillarCount','selected'),('extraPillarCount','empty'),('coveredFinePointCount','covered'),('finePointCountInRoi','fine')]:assert raw[col].sum()==target[field]
fig,ax=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for df,label in [(a,'Before'),(b,'Split-shaft recovery')]:
 ax[0].plot(df.frame,df.errorM*100,label=label,lw=.8);sel=df.frame.between(810,875);ax[1].plot(df.frame[sel],df.errorM[sel]*100,label=label,lw=1.2)
for z in ax:z.set(xlabel='Frame',ylabel='Position error (cm)');z.grid(alpha=.25);z.legend()
ax[0].set_title('Recursive Mississippi replay');ax[1].set_title('Recovery around frames 820 and 856');ax[1].axvline(856,color='gray',ls=':',lw=1)
fig.savefig(P/'route_comparison.png',dpi=160);fig.savefig(P/'route_comparison.pdf');plt.close(fig)
validation=json.loads((P/'validation.json').read_text());summary=json.loads((P/'replay_summary.json').read_text());print(json.dumps(dict(validation=validation,summary=summary),indent=2))
source=[ROOT/'config/pillarPoleDistributionConfig.m',ROOT/'perception/offGroundFeatures/classifyPillarPoleSupport.m',ROOT/'perception/offGroundFeatures/recoverSplitPoleShaft.m',ROOT/'tests/pillarPoleDistributionTest.m']
files=source+[p for p in P.iterdir() if p.is_file() and p.name!='artifact_hashes.json']+[ROOT/'output/pole_boundary_recovery_20260929/replay.mat']
(P/'artifact_hashes.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)},indent=2)+'\n')
