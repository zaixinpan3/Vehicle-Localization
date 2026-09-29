"""Independently audit the deployed detector and causal route outputs."""
from pathlib import Path
import json,hashlib
import numpy as np,pandas as pd
P=Path(__file__).resolve().parent;ROOT=P.parents[1]
a=pd.read_csv(P/'raw_perception.csv');b=pd.read_csv(P/'deployed_pole_replay.csv')
assert len(a)==len(b)==1170
cols=['frame','candidatePillarCount','extraPillarCount','coveredFinePointCount','finePointCountInRoi']
assert np.array_equal(a[cols].values,b[cols].values)
screen=pd.read_csv(P/'specialist_metrics.csv')
for part,mask in [('all',b.frame>0),('prefix',b.frame<=780),('suffix',b.frame>780)]:
 q=b[mask];expected=screen[screen.part==part].iloc[0]
 assert q.candidatePillarCount.sum()==expected.selected and q.extraPillarCount.sum()==expected['empty']
 assert q.coveredFinePointCount.sum()==expected.covered and q.finePointCountInRoi.sum()==expected.fine
assert b[b.frame==857].coveredFinePointCount.iloc[0]==13
r=pd.read_csv(P/'full_route.csv');old=pd.read_csv(ROOT/'research/revised_route_max_20260928/full_route.csv')
ref=pd.read_csv(ROOT/'research/line_direction_matching_20260928/production/calls.csv')
assert len(r)==1170
assert np.allclose(np.hypot(r.x-ref.referenceX,r.y-ref.referenceY),r.errorM,atol=1e-7,rtol=0)
model=json.loads((ROOT/'config/mississippiPolePillarDistributionModel.json').read_text());candidate=json.loads((ROOT/'output/pole_selective_recovery_20260928/model.json').read_text());model.pop('decisionThresholds');assert model==candidate
analysis=json.loads((P/'code_analysis.json').read_text());assert all(not x for x in analysis['issues'])
t=pd.read_csv(P/'tests.csv');assert t.passed.all() and not t.failed.any() and len(t)==32
v=json.loads((P/'features_0857.json').read_text());assert v['features']['pole']['selectedPillarCount']==1 and v['displayedPoints']==65536 and not v['featureOverlay']
runtime=pd.read_csv(P/'paired_runtime.csv');med=runtime[runtime['repeat']>1].groupby('variant').milliseconds.median()
summary=dict(rawFrames=1170,testsPassed=len(t),cleanMatlabFiles=len(analysis['files']),frame857BeforeM=float(old.loc[old.frame==857,'errorM'].iloc[0]),frame857AfterM=float(r.loc[r.frame==857,'errorM'].iloc[0]),maxAfterFrame=int(r.iloc[1:].loc[r.iloc[1:].errorM.idxmax(),'frame']),maxBeforeM=float(old.iloc[1:].errorM.max()),maxAfterM=float(r.iloc[1:].errorM.max()),rmseBeforeM=float(np.sqrt(np.mean(old.errorM**2))),rmseAfterM=float(np.sqrt(np.mean(r.errorM**2))),medianPerceptionBeforeMs=float(med.loc[1]),medianPerceptionAfterMs=float(med.loc[2]),wholePillarStatistics=True,matchingParametersChanged=False,allMetrics=screen.to_dict('records'))
(P/'validation.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
