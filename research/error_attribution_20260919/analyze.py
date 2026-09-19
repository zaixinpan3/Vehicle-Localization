"""Summarize actual offline controls; no pose alignment or error clipping."""
from pathlib import Path
import json
import shutil
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/error_attribution_20260919'
DEST = Path(__file__).resolve().parent
calls = pd.read_csv(ROOT / 'output/matching_refinement_20260919/validated/recursive/calls.csv')
controls = pd.read_csv(OUT / 'matching_controls.csv')
fine = pd.read_csv(OUT / 'fine_controls.csv')
inputs = pd.read_csv(OUT / 'aligned_inputs.csv')
pair = pd.read_csv(OUT / 'pair_details.csv')
frozen = pd.read_csv(OUT / 'frozen_pairs.csv')

def metric(a):
    a = np.asarray(a)
    return dict(samples=len(a), rmseM=float(np.sqrt(np.mean(a*a))),
                p95M=float(np.quantile(a, .95, method='hazen')), maximumM=float(a.max()))

assert len(calls) == 1170 and len(controls) == 1170*8
assert controls.groupby('variant').frame.nunique().eq(1170).all()
reproduced = controls[controls.variant == 'reproduce'].set_index('frame')
accepted = calls[calls.accepted == 1]
assert reproduced.loc[accepted.frame, 'differenceFromProductionM'].max() < 1e-5
assert (frozen.exitflag == 1).all()
summaries = []
for label, lo, hi in [('full',1,1170), ('first_peak',80,100), ('second_peak',800,842)]:
    for variant, group in controls[controls.frame.between(lo,hi)].groupby('variant'):
        summaries.append(dict(region=label, variant=variant, **metric(group.candidateErrorM),
                              accepted=int(group.accepted.sum())))
pd.DataFrame(summaries).to_csv(DEST / 'matching_control_summary.csv', index=False)
summaries = []
for label, lo, hi in [('selected_frames',1,1170), ('first_peak',80,100), ('second_peak',800,842)]:
    for variant, group in fine[fine.frame.between(lo,hi)].groupby('replacedClass'):
        summaries.append(dict(region=label, replacedClass=variant, **metric(group.candidateErrorM),
                              accepted=int(group.accepted.sum()),
                              productionRmseM=metric(group.productionErrorM)['rmseM']))
pd.DataFrame(summaries).to_csv(DEST / 'fine_control_summary.csv', index=False)
selection = controls[controls.frame.isin([30,85,90,92,94,200,425,600,816,820,830,842,959,966])]
selection.to_csv(DEST / 'selected_matching_controls.csv', index=False)
for name in ['pair_details.csv', 'frozen_pairs.csv', 'observer_controls.csv', 'raw_time_fields.json']:
    path=OUT/name
    if path.exists(): shutil.copy2(path, DEST/name)

error=calls.positionErrorM.to_numpy()
tail=error>.5
summary=dict(productionAllOutputs=metric(error), aboveHalfMeterCount=int(tail.sum()),
             aboveHalfMeterSquaredErrorShare=float(sum(error[tail]**2)/sum(error**2)),
             oracleControlsAreIndependentPerFrame=True, fineControlFrames=int(fine.frame.nunique()),
             fineControlReusesMapBuildingObservations=True,
             acceptedReplayMaxDifferenceM=float(reproduced.loc[accepted.frame, 'differenceFromProductionM'].max()))
motion_rows=[]
rate=np.abs(np.rad2deg(inputs.yawRate.to_numpy()))
for lo,hi in [(0,3),(3,10),(10,float('inf'))]:
    keep=(rate>=lo)&(rate<hi); e=error[:len(inputs)][keep]
    motion_rows.append(dict(minYawRateDegPerS=lo,maxYawRateDegPerS=None if np.isinf(hi) else hi,
                           **metric(e),squaredErrorShare=float(sum(e*e)/sum(error[:len(inputs)]**2))))
summary['turnRateGroups']=motion_rows
good=inputs.lidarValid.astype(bool)
for source in ['lidar','gnss']:
    e=np.hypot(inputs[f'{source}X']-inputs.referenceX,inputs[f'{source}Y']-inputs.referenceY)
    summary[source+'MeasurementCommonValid']=metric(e[good])
summary['regions']={}
for label,lo,hi in [('first_peak',80,100),('second_peak',800,842)]:
    s=calls[calls.frame.between(lo,hi)]
    dx=(s.x-s.referenceX).to_numpy();dy=(s.y-s.referenceY).to_numpy();a=s.referencePsi.to_numpy()
    longitudinal=dx*np.cos(a)+dy*np.sin(a);lateral=-dx*np.sin(a)+dy*np.cos(a)
    summary['regions'][label]=dict(**metric(s.positionErrorM),
        longitudinalRmseM=metric(longitudinal)['rmseM'],lateralRmseM=metric(lateral)['rmseM'],
        squaredErrorShare=float(sum(s.positionErrorM**2)/sum(error**2)))
(DEST/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

plt.rcParams.update({'font.size':10})
fig,axs=plt.subplots(3,1,figsize=(12,10),constrained_layout=True)
axs[0].plot(calls.timeSeconds,error,label='Current matching',lw=1)
for variant,label in [('exact_initial','Exact reference initialization (oracle)'),
                      ('exact_window_motion','Exact relative scan transport (oracle)')]:
    s=controls[controls.variant==variant]
    axs[0].plot(calls.timeSeconds,s.candidateErrorM,label=label,lw=.9,alpha=.8)
axs[0].set(ylabel='XY discrepancy (m)',title='Full sequence: correcting initialization or transport does not remove the main matching errors')
axs[0].legend(ncol=3,fontsize=8)
for variant,label in [('reproduce','Current'),('without_sign','Traffic sign omitted'),('without_pole','Pole omitted')]:
    s=controls[(controls.variant==variant)&controls.frame.between(800,842)]
    axs[1].plot(s.frame,s.candidateErrorM,label=label,lw=1.4)
axs[1].set(xlabel='Frame',ylabel='Candidate XY discrepancy (m)',title='Fixed production initial guesses: class effects change within the difficult interval')
axs[1].legend()
for scenario,label in [('both','GNSS + LiDAR'),('gnss_only','GNSS only'),('lidar_only','LiDAR only')]:
    s=pd.read_csv(ROOT/f'output/matching_refinement_20260919/observer_final/{scenario}_poses.csv')
    axs[2].plot(s.time,np.hypot(s.x-s.referenceX,s.y-s.referenceY),label=label,lw=1)
axs[2].set(xlabel='Time (s)',ylabel='XY discrepancy (m)',title='Observer outputs: fusion attenuates the biased matching measurements')
axs[2].legend()
for ax in axs: ax.grid(alpha=.25)
fig.savefig(OUT/'error_attribution.png',dpi=160)
fig.savefig(OUT/'error_attribution.pdf')
plt.close(fig)
print(json.dumps(summary,indent=2))
