"""Audit the new causal replay and plot the maximum-frame failure controls."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent;ROOT=P.parents[1]
r=pd.read_csv(P/'full_route.csv');before=pd.read_csv(ROOT/'research/pole_selective_recovery_20260928/full_route.csv');ref=pd.read_csv(ROOT/'research/line_direction_matching_20260928/production/calls.csv')
assert len(r)==1170 and np.array_equal(r.frame,np.arange(1,1171))
assert np.allclose(r[['x','y','psi']],before[['x','y','psi']],atol=1e-9,rtol=0)
assert np.allclose(np.hypot(r.x-ref.referenceX,r.y-ref.referenceY),r.errorM,atol=1e-7,rtol=0)
maximum=r.iloc[1:].loc[r.iloc[1:].errorM.idxmax()];assert maximum.frame==856
c=pd.read_csv(P/'controls.csv').set_index('variant');o=pd.read_csv(P/'oracle_pole_controls.csv');s=json.loads((P/'summary.json').read_text());viewer=json.loads((P/'features_0856.json').read_text())
assert s['reproductionMaxAbs']==0 and abs(c.loc['production','errorM']-maximum.errorM)<1e-12
assert abs(c.loc['reference_seed','errorM']-maximum.errorM)<1e-5
assert c.loc['no_map_merge','errorM']<.102 and c.loc['unlimited_refinement','errorM']<.102
assert o.iloc[2].matchedPoles==1 and o.iloc[2].errorM<.073
assert viewer['displayedPoints']==65536 and viewer['markerSize']==4 and not viewer['featureOverlay']
assert viewer['features']['pole']['selectedPillarCount']==0 and viewer['features']['pole']['referencePointCount']==18
scores=pd.read_csv(P/'pole_scores.csv');original=pd.read_csv(P/'pole_feature_neighborhood.csv');original=original[original.frame==856]
assert np.array_equal(np.sort(scores.globalOwner),np.sort(original.pillar))
assert np.allclose(scores.sort_values('globalOwner').score,original.sort_values('pillar').score,atol=1e-12,rtol=0)
issues=json.loads((P/'code_analysis.json').read_text());assert all(not x for x in issues['issues'])
fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
a=axes[0,0];n=r[r.frame.between(850,860)];a.plot(n.frame,n.errorM*100,'o-',color='#b64b3c');a.axvline(856,color='gray',ls=':');a.set(xlabel='Frame',ylabel='Position error (cm)',title='Current recursive replay: 1,170 raw scans')
a=axes[0,1];labels=['Current','Reference seed (oracle)','Allow existing fine result','No map merge','Curb only','Add missed pole (oracle)'];values=[c.loc[x,'errorM'] for x in ['production','reference_seed','unlimited_refinement','no_map_merge','only_curb']]+[o.iloc[2].errorM]
a.barh(labels,np.array(values)*100,color=['#b64b3c','#b64b3c','#567aab','#567aab','#888888','#168879']);a.invert_yaxis();a.set(xlabel='Position error (cm)',title='Same input except the named intervention')
a=axes[1,0];pts=pd.read_csv(P/'reference_pole_points.csv');a.scatter(pts.x,pts.y,s=20,color='#ec8733');a.axhline(-3.5,color='black',ls='--',label='0.6 m pillar boundary');a.set(xlabel='Sensor X (m)',ylabel='Sensor Y (m)',title='18 reference pole points split into 11 + 7');a.set_aspect('equal',adjustable='datalim');a.legend()
a=axes[1,1];m=pd.read_csv(P/'original_map.csv').set_index('globalId');pairs=pd.read_csv(P/'coarse_pairs.csv');q=pairs[pairs.source==14].iloc[0]
for i in [1263,1264]:
 v=m.loc[i];a.scatter(v.x,v.y,facecolors='none',edgecolors='#567aab',s=85,label=f'Original map {i}')
a.scatter(q.targetReferenceBody_1,q.targetReferenceBody_2,marker='X',color='black',s=70,label='Merged target')
a.scatter(q.sourceBody_1,q.sourceBody_2,marker='+',color='#bb329e',s=100,label='Source 14 at reference pose')
a.set(xlabel='Reference-body forward (m)',ylabel='Left (m)',title='Merged sign target is 54.5 cm from source');a.set_aspect('equal',adjustable='datalim');a.legend(fontsize=8)
for a in axes.flat:a.grid(alpha=.2)
fig.suptitle('Mississippi frame 856: missing pole plus map-merge and refinement-gate bias\nDiagnostic controls only; production parameters unchanged')
fig.savefig(P/'diagnosis.png',dpi=160);fig.savefig(P/'diagnosis.pdf');plt.close(fig)
facts=dict(frames=1170,currentProductionReplayed=True,previousTrajectoryReproduced=True,maximumFrame=856,errorM=float(maximum.errorM),yawErrorDeg=float(maximum.yawErrorDeg),timeSeconds=float(ref.loc[ref.frame==856,'timeSeconds'].iloc[0]),noMapMergeErrorM=float(c.loc['no_map_merge','errorM']),oraclePoleErrorM=float(o.iloc[2].errorM),referencePolePoints=18,missedOwnerScores=scores.score.to_list(),cleanMatlabFiles=len(issues['files']),productionChanged=False)
(P/'validation.json').write_text(json.dumps(facts,indent=2)+'\n');print(json.dumps(facts,indent=2))
