"""Summarize executed full-route comparisons and validated production artifacts."""
from pathlib import Path
import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
summaries=pd.concat([pd.read_csv(p) for p in sorted(D.glob('*_summary.csv'))],ignore_index=True)
summaries.to_csv(D/'all_replay_summaries.csv',index=False)
base=json.loads((D.parent/'iterative_matching_20260929/final_summary.json').read_text())
b=pd.read_csv(D.parent/'iterative_matching_20260929/final_raw.csv')
f=pd.read_csv(D/'finalSurface.csv');s=pd.read_csv(D/'finalSurface_summary.csv').iloc[0]
v=pd.read_csv(D/'final_raw_verification.csv');t=pd.read_csv(D/'final_tests.csv');a=pd.read_csv(D/'code_analysis.csv')
assert len(f)==len(v)==1170 and v.unchangedDetections.eq(1).all()
assert t.passed.eq(1).all() and not t.failed.any() and not t.incomplete.any() and not a.findings.any()
summary=dict(baseline=base,final=s.to_dict(),completedReplayLabels=len(summaries),frames=1170,
    maximumPolicy='Exclude configured first frame only; RMSE/P95 include all 1170 frames.',
    unchangedDetectionFrames=int(v.unchangedDetections.sum()),testsPassed=len(t),staticAnalysisFiles=len(a),
    geometryValidation=dict(maximumMeanDifference=float(v.meanDifference.max()),maximumCovarianceDifference=float(v.covarianceDifference.max())),
    runtime=dict(baselinePerceptionMedianMs=float(v.baselinePerceptionMs.median()),finalPerceptionMedianMs=float(v.finalPerceptionMs.median()),pairedMedianExtraMs=float((v.finalPerceptionMs-v.baselinePerceptionMs).median()),note='Alternating execution order on this machine; other validation processes can affect timings.'),
    limitations=['Reused development route/map, not independent-drive validation.','Original fine annotations and INSPVA are comparison references, not surveyed truth.','No substantial stable peak-error reduction established; no physical lower-bound claim.'])
(D/'final_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
f.iloc[1:].nlargest(15,'errorM').to_csv(D/'final_largest_errors.csv',index=False)
fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
ax=axes[0];ax.plot(b.frame.iloc[1:],100*b.errorM.iloc[1:],label='Previous production',lw=.9,alpha=.7);ax.plot(f.frame.iloc[1:],100*f.errorM.iloc[1:],label='Boundary geometry',lw=.9)
ax.set(xlabel='Mississippi frame',ylabel='Position discrepancy (cm)',title='Full causal replay; configured first-frame offset excluded from this plot');ax.legend();ax.grid(alpha=.2)
frames=[178,806,854,1096];before=[];after=[]
for k in frames:
 c=pd.read_csv(D/f'curb_surface_{k}.csv');before.append(100*c.before.mean());after.append(100*c.after.mean())
x=np.arange(4);ax=axes[1];ax.bar(x-.18,before,.36,label='Whole-pillar means');ax.bar(x+.18,after,.36,label='Boundary model');ax.set(xticks=x,xticklabels=[str(k) for k in frames],xlabel='Diagnostic frame',ylabel='Transverse mean absolute discrepancy (cm)',title='Curb locations against original fine-point centers (development diagnostics)');ax.legend();ax.grid(axis='y',alpha=.2)
fig.savefig(D/'route_geometry_comparison.png',dpi=160);fig.savefig(D/'route_geometry_comparison.pdf')
print(json.dumps(summary,indent=2))
