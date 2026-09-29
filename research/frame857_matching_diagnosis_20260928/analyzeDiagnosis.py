"""Audit the frame-857 causal controls and illustrate the competing effects."""
from pathlib import Path
import csv
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P = Path(__file__).resolve().parent
c = pd.read_csv(P/'controls.csv').set_index('variant')
o = pd.read_csv(P/'oracle_pole_controls.csv')
v = json.loads((P/'features_0857.json').read_text())
assert v['displayedPoints'] == 65536 and v['markerSize'] == 4 and not v['featureOverlay']
assert abs(c.loc['production','errorM']-.473865024705953)<1e-9
assert abs(c.loc['production','errorM']-c.loc['reference_seed','errorM'])<1e-5
assert c.loc['no_map_merge','errorM']<.121 and c.loc['unlimited_refinement','errorM']<.121
assert all(o.loc[o.iloc[:,0]>1,'matchedPoles']==1)
assert o.iloc[2].errorM<.07 and o.iloc[6].errorM<.079
score = pd.read_csv(P/'pole_scores.csv').iloc[0]
assert abs(score.score-.8301088520930122)<1e-12
pred = pd.read_csv(P.parent/'pole_geometry_20260927/stable_moments_model_predictions.csv')
pred = pred[pred.dataset=='Mississippi']
rows=[]
for threshold in [.8701683227450074,.83]:
    for label,subset in [('all1170',pred),('reused_suffix781to1170',pred[pred.frame>780])]:
        selected=subset[subset.score>=threshold];bad=int((selected.finePointCount==0).sum())
        rows.append(dict(threshold=threshold,scope=label,selected=len(selected),emptyReference=bad,falseFraction=bad/len(selected)))
pd.DataFrame(rows).to_csv(P/'threshold_screen.csv',index=False)
fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
labels=['Current','Exact reference seed','Allow existing fine result','No map merge','Oracle pole owners','Threshold 0.83']
values=[c.loc['production','errorM'],c.loc['reference_seed','errorM'],c.loc['unlimited_refinement','errorM'],c.loc['no_map_merge','errorM'],o.iloc[2].errorM,o.iloc[6].errorM]
axes[0].barh(labels,np.array(values)*100,color=['#bf4b45','#bf4b45','#687da9','#687da9','#228575','#228575'])
axes[0].invert_yaxis();axes[0].set(xlabel='Position error (cm)',title='Same frame and prediction; diagnostic interventions')
m=pd.read_csv(P/'original_map.csv').set_index('globalId');pairs=pd.read_csv(P/'coarse_pairs.csv');a=axes[1]
for i in [1263,1264]:
    q=m.loc[i];a.scatter(q.x,q.y,s=100,facecolors='none',edgecolors='#687da9');a.annotate(f'Map {i}',(q.x,q.y),xytext=(3,8),textcoords='offset points')
for _,q in pairs[pairs.semanticName=='trafficSign'].iterrows():
    if '1263' not in q.mapMembers:continue
    a.scatter(q.sourceBody_1,q.sourceBody_2,marker='+',s=80,color='#be36a2')
    a.annotate(f'Source {int(q.source)}',(q.sourceBody_1,q.sourceBody_2),xytext=(-10,-14),textcoords='offset points')
    a.scatter(q.targetReferenceBody_1,q.targetReferenceBody_2,marker='X',s=75,color='black')
a.set(xlabel='Reference-body forward (m)',ylabel='Left (m)',title='Two sign-map modes merged across 0.964 m')
a.set_aspect('equal',adjustable='datalim')
for a in axes:a.grid(alpha=.2)
fig.suptitle('Frame 857: pole omission and map/trust-gate bias both matter\nDiagnostic results only; production configuration unchanged')
fig.savefig(P/'diagnosis.png',dpi=150);fig.savefig(P/'diagnosis.pdf');plt.close(fig)
checks=json.loads((P/'code_analysis.json').read_text());assert all(not x for x in checks['issues'])
facts=dict(baselineReproduced=True,viewerVerified=True,oraclePoleErrorM=float(o.iloc[2].errorM),threshold083ErrorM=float(o.iloc[6].errorM),currentErrorM=float(c.loc['production','errorM']),noMapMergeErrorM=float(c.loc['no_map_merge','errorM']),poleScore=float(score.score),thresholdScreenUsesFrozenPredictions=True,productionChanged=False)
(P/'validation.json').write_text(json.dumps(facts,indent=2)+'\n');print(json.dumps(facts,indent=2))
