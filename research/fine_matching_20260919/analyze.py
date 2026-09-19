"""Verify and compare complete fresh fine/coarse matching trajectories."""
from pathlib import Path
import json
import shutil
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/fine_matching_20260919'
DEST=Path(__file__).resolve().parent

def wrap(a):
    return np.arctan2(np.sin(a),np.cos(a))

def metrics(c):
    e=np.hypot(c.x-c.referenceX,c.y-c.referenceY).to_numpy()
    yaw=np.rad2deg(wrap(c.psi-c.referencePsi))
    return dict(samples=len(c),rmseM=float(np.sqrt(np.mean(e*e))),
                medianM=float(np.median(e)),p95M=float(np.quantile(e,.95,method='hazen')),
                maximumM=float(e.max()),yawRmseDeg=float(np.sqrt(np.mean(yaw*yaw))),
                aboveHalfMeter=int(sum(e>.5)),aboveOneMeter=int(sum(e>1)))

tables={};rows=[];checks={}
baseline=pd.read_csv(ROOT/'output/matching_refinement_20260919/validated/recursive/calls.csv')
for name in ['coarse_recursive','fine_recursive','coarse_binary_quality_recursive',
             'coarse_commonInitialGuess','fine_commonInitialGuess']:
    c=pd.read_csv(OUT/name/'calls.csv');tables[name]=c
    assert len(c)==1170 and np.array_equal(c.frame,np.arange(1,1171))
    ref=c[['referenceX','referenceY','referencePsi']].to_numpy()
    pos=c[['x','y','psi']].to_numpy()
    initial=c[['predictedX','predictedY','predictedPsi']].to_numpy()
    accepted=c.accepted.astype(bool);directional=c.directionalAccepted.astype(bool)
    absent=~(accepted|directional)
    np.testing.assert_array_equal(pos[absent],initial[absent])
    np.testing.assert_allclose(np.hypot(pos[:,0]-ref[:,0],pos[:,1]-ref[:,1]),c.positionErrorM,atol=1e-8,rtol=0)
    if name.endswith('_recursive'):
        motion=pd.read_csv(OUT/name/'dead_reckoning.csv')[['x','y','psi']].to_numpy()
        delta=np.diff(motion[:,:2],axis=0);angle=motion[:-1,2]
        forward=delta[:,0]*np.cos(angle)+delta[:,1]*np.sin(angle)
        left=-delta[:,0]*np.sin(angle)+delta[:,1]*np.cos(angle)
        expected=ref+np.array([.5,-.4,np.deg2rad(2)])
        angle=pos[:-1,2]
        expected[1:,0]=pos[:-1,0]+forward*np.cos(angle)-left*np.sin(angle)
        expected[1:,1]=pos[:-1,1]+forward*np.sin(angle)+left*np.cos(angle)
        expected[1:,2]=angle+wrap(np.diff(motion[:,2]))
        np.testing.assert_allclose(initial[:,:2],expected[:,:2],rtol=0,atol=2e-8)
        assert np.max(np.abs(wrap(initial[:,2]-expected[:,2])))<1e-11
    else:
        np.testing.assert_array_equal(initial,baseline[['predictedX','predictedY','predictedPsi']])
    for population,mask in [('all_outputs',np.ones(len(c),bool)),('accepted',accepted)]:
        rows.append(dict(variant=name,population=population,**metrics(c[mask])))
    checks[name]=dict(accepted=int(accepted.sum()),directional=int(directional.sum()),
                     rejections=c.loc[absent,'reason'].value_counts().to_dict(),
                     rejectionFrames=c.loc[absent,'frame'].tolist(),
                     coveragePassed=True,causalPredictionPassed=True,rejectedFallbackPassed=True)
    w=pd.read_csv(OUT/name/'source_windows.csv')
    assert (w.scans<=3).all() and (w.spanSeconds<=.25+1e-12).all()
    assert np.isfinite(c.loc[accepted,['informationXX','informationYY','informationPsiPsi']]).all().all()
    for r in c[accepted].itertuples():
        information=np.array([[r.informationXX,r.informationXY,r.informationXPsi],
                              [r.informationXY,r.informationYY,r.informationYPsi],
                              [r.informationXPsi,r.informationYPsi,r.informationPsiPsi]])
        assert np.linalg.eigvalsh(information).min()>0
    checks[name]['acceptedInformationPositive']=True

coarse=tables['coarse_recursive'];fine=tables['fine_recursive']
for name in ['coarse_recursive','coarse_commonInitialGuess']:
    np.testing.assert_allclose(tables[name][['x','y','psi']],baseline[['x','y','psi']],rtol=0,atol=1e-7)
common=coarse.accepted.astype(bool)&fine.accepted.astype(bool)
for name,c in [('coarse_recursive',coarse),('fine_recursive',fine)]:
    rows.append(dict(variant=name,population='common_accepted',**metrics(c[common])))
summary=pd.DataFrame(rows);summary.to_csv(DEST/'metrics.csv',index=False)
(DEST/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
audit=pd.read_csv(OUT/'input_audit.csv')
assert len(audit)==1170 and audit.coarseMeanDifferenceM.max()<1e-8 and audit.coarseCovarianceDifference.max()<1e-8
input_summary=dict(frames=1170,maximumCoarseMeanDifferenceM=float(audit.coarseMeanDifferenceM.max()),
                   maximumCoarseCovarianceDifference=float(audit.coarseCovarianceDifference.max()),
                   finePointCounts=audit[['curbPoints','polePoints','trafficSignPoints']].sum().to_dict(),
                   finePerceptionMedianSeconds=float(audit.finePerceptionSeconds.median()),
                   fineConversionMedianSeconds=float(audit.fineConversionSeconds.median()),
                   timingScope='Four simultaneous MATLAB perception partitions, two computational threads each; not isolated benchmarking')
(DEST/'input_summary.json').write_text(json.dumps(input_summary,indent=2)+'\n')
selected=[28,91,92,94,214,425,600,615,*range(805,818),820,830,832,842,855,943,959,966,1047,1137]
pd.concat([c[c.frame.isin(selected)].assign(variant=name) for name,c in tables.items()],ignore_index=True).to_csv(DEST/'selected_frames.csv',index=False)
shutil.copy2(OUT/'adapter_tests.csv',DEST/'adapter_tests.csv')
# Supplemental audits were run in the MATLAB MCP session, separately from replay.
for name in ['code_analyzer.csv','frame92_coordinate_audit.csv']:
    if (OUT/name).exists():
        shutil.copy2(OUT/name,DEST/name)
diagnostics={}
for name,c in [('coarse',coarse),('fine',fine)]:
    components=pd.read_csv(OUT/(name+'_recursive')/'component_counts.csv').drop(columns='frame')
    diagnostics[name]=dict(medianComponents=components.median().to_dict(),
                           medianMatchedComponents=float(c.matches.median()),
                           medianSimilarity=float(c.similarity.median()))
diagnostics['fineVsCoarse']=dict(improvedFrames=int(sum(fine.positionErrorM<coarse.positionErrorM)),
    improvedByMoreThanTenCm=int(sum(fine.positionErrorM<coarse.positionErrorM-.1)),
    worsenedByMoreThanTenCm=int(sum(fine.positionErrorM>coarse.positionErrorM+.1)))
fixed=tables['fine_commonInitialGuess']
peak=int(fine.positionErrorM.idxmax())
diagnostics['finePeak']=dict(frame=int(fine.frame.iloc[peak]),
    recursiveErrorM=float(fine.positionErrorM.iloc[peak]),
    commonInitialGuessErrorM=float(fixed.positionErrorM.iloc[peak]),
    predictedPositionDifferenceM=float(np.hypot(fine.predictedX.iloc[peak]-fixed.predictedX.iloc[peak],
                                               fine.predictedY.iloc[peak]-fixed.predictedY.iloc[peak])),
    accepted=bool(fine.accepted.iloc[peak]),similarity=float(fine.similarity.iloc[peak]),
    observableRank=int(fine['rank'].iloc[peak]))
(DEST/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n')

fig,axs=plt.subplots(3,1,figsize=(12,10),constrained_layout=True)
for name,label in [('coarse_recursive','Coarse perception'),('fine_recursive','Fresh fine perception')]:
    c=tables[name];axs[0].plot(c.timeSeconds,c.positionErrorM,label=label,lw=1)
    axs[1].plot(c.timeSeconds,c.yawErrorDeg,label=label,lw=1)
axs[0].set(ylabel='XY discrepancy (m)',title='1170-frame matching-only replay: same map, solver, motion inputs and first initial pose')
axs[1].set(ylabel='Yaw discrepancy (deg)',xlabel='Time (s)')
for name,label in [('coarse_recursive','Coarse'),('fine_recursive','Fine')]:
    c=tables[name];e=np.sort(c.positionErrorM.to_numpy())
    axs[2].plot(e,np.arange(1,len(e)+1)/len(e),label=label,lw=1.5)
axs[2].set(xlabel='XY discrepancy (m)',ylabel='Empirical cumulative probability',title='All outputs, including predictions on rejected frames')
for ax in axs:ax.grid(alpha=.25);ax.legend()
fig.savefig(OUT/'fine_vs_coarse.png',dpi=160);fig.savefig(OUT/'fine_vs_coarse.pdf');plt.close(fig)
print(summary.to_string(index=False));print(json.dumps(input_summary,indent=2))
