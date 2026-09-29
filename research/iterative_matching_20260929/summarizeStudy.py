"""Summarize recorded route replays; reference poses are evaluation-only."""
from pathlib import Path
import csv
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

study = Path(__file__).resolve().parent
repo = study.parent.parent
rows = {}
for path in sorted(study.glob('*.csv')):
    if path.name == 'variant_summary.csv':
        continue
    for row in csv.DictReader(path.open()):
        if 'maximumErrorM' in row and 'variant' in row:
            rows[row['variant']] = {k: row[k] for k in (
                'variant', 'maximumFrame', 'maximumErrorM', 'rmseM', 'p95M', 'full', 'directional')}
variants = pd.DataFrame(rows.values())
for key in variants.columns[1:]:
    variants[key] = pd.to_numeric(variants[key])
variants.sort_values('maximumErrorM').to_csv(study / 'variant_summary.csv', index=False)
before = pd.read_csv(repo / 'research/pole_boundary_recovery_20260929/full_route.csv')
after = pd.read_csv(study / 'final_raw.csv')
assert len(before) == len(after) == 1170
cached = pd.read_csv(study / 'cleanSoft.csv')
assert np.max(np.abs(after[['x','y','psi']].to_numpy()-cached[['x','y','psi']].to_numpy())) < 1e-9
comparison = before[['frame', 'errorM']].rename(columns={'errorM':'beforeM'})
comparison['afterM'] = after.errorM
comparison.to_csv(study / 'route_comparison.csv', index=False)
fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
axes[0].plot(before.frame[1:], before.errorM[1:]*100, lw=.9, color='.55', label='Before')
axes[0].plot(after.frame[1:], after.errorM[1:]*100, lw=1, color='#1479ad', label='Deployed')
for table, color in [(before,'.35'),(after,'#1479ad')]:
    row = table.iloc[1:].loc[table.errorM.iloc[1:].idxmax()]
    axes[0].scatter(row.frame,row.errorM*100,c=color,s=25,zorder=4)
    axes[0].annotate(f'{int(row.frame)}: {row.errorM*100:.2f} cm',(row.frame,row.errorM*100),xytext=(8,8),textcoords='offset points')
axes[0].set(xlabel='Mississippi frame', ylabel='Horizontal error (cm)', title='Causal 1,170-frame replay; configured initialization omitted from this plot')
axes[0].legend()
labels = ['baseline', 'anchored_pole', 'independent_t10_s25', 'confidentSoft200', 'softDirection25', 'softNeighborhood2']
values = []
for label in labels:
    if label in rows:
        values.append((label,float(rows[label]['maximumErrorM'])*100))
if not values or values[0][0] != 'baseline':
    values.insert(0,('baseline',before.errorM.iloc[1:].max()*100))
axes[1].bar([x[0] for x in values],[x[1] for x in values],color='#1479ad')
axes[1].tick_params(axis='x',labelsize=8)
axes[1].set(ylabel='Route maximum (cm)', title='Selected development milestones (all evaluated on the same reused route)')
for ax in axes: ax.grid(axis='y',alpha=.2)
fig.savefig(study/'route_comparison.png',dpi=170)
fig.savefig(study/'route_comparison.pdf')
summary = {
    'recordedReplayLabels':len(rows),
    'framesPerReplay':1170,
    'maximumExcludesConfiguredInitialization':True,
    'initializationErrorM':float(after.errorM.iloc[0]),
    'beforeMaximumM':float(before.errorM.iloc[1:].max()),
    'afterMaximumM':float(after.errorM.iloc[1:].max()),
    'peakReductionFraction':float(1-after.errorM.iloc[1:].max()/before.errorM.iloc[1:].max()),
    'rawReplayAgreesWithCachedReplay':True,
    'beforeRmseM':float(np.sqrt(np.mean(before.errorM**2))),
    'afterRmseM':float(np.sqrt(np.mean(after.errorM**2))),
    'note':'Replay labels include cleanup controls and repeated profiles, not independent datasets. MATLAB prctile P95 is in final_summary.json.'
}
(study/'study_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
