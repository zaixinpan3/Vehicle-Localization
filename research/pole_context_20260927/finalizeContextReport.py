"""Refresh the study narrative from measured final and rejected-profile results."""
import json,csv
from pathlib import Path
P=Path(__file__).resolve().parent
s=json.loads((P/'summary.json').read_text());p=P/'README.md';t=p.read_text()
t=t.replace('The current rule was frozen in\n`profile_freeze.json` before evaluating the temporal suffix and Downtown in this\niteration. Both sequences influenced earlier algorithm iterations, so these are\nregression and transfer checks, not previously unseen independent test sets.', 'The first profile was frozen before this iteration\'s temporal and Downtown\nchecks. Distance-stratified failures prompted two further measured revisions;\n`profile_freeze.json` identifies the installed one. These datasets and previous\nevaluations were reused, so no independent held-out accuracy is claimed.')
a=t.index('## Installed rule');b=t.index('## Full-sequence and transfer results')
installed='''## Installed rule

Existing continuous-height shaft support, density, width, isolation, trimming,
and actual per-owner support checks remain in force. A hypothesis must additionally
have at least 30 retained support points **or** satisfy the extra tight/upright
geometry condition. Each selected owner must have at least 15 retained points
**or** satisfy that condition. The earlier hard minimum of 10 owner points and
0.30 m owner height still applies.

For sensor XY range up to 15 m, the extra geometry limits are 4 degrees tilt and
0.10 m residual RMS. Between 15 and 20 m they interpolate linearly to 6 degrees
and 0.12 m, then remain capped at those values. The bounded relaxation allows
less precise axes from sparse distant sampling; it is an empirically checked
heuristic, not a calibrated sensor uncertainty model. Input coordinates must
retain the sensor/vehicle-frame convention already used by the perception ROI.
No original density, continuity, width or hard owner-support check is relaxed.

This treats weakly supported shafts and peripheral owners more cautiously without
eliminating every sparse pole. Dense support retains the preceding acceptance
envelope. Counts and geometry already exist; the change adds scalar comparisons
and range interpolation. Mixed pillars still qualify from a concentrated,
vertically continuous subset. All online detection remains on 0.6 m pillars.

The initial fixed 4-degree/0.10 m profile reduced Mississippi extras by 29.76%,
but the aggregate concealed a 20-30 m coverage drop from 76.17% to 51.78%. It
was rejected. An inverse-square relaxation of point thresholds restored far
coverage but reduced extras by only 2.36%; it was also rejected. Their full
results remain in `fixed_count_*` and `count_scaled_*` artifacts. The installed
profile recovers part of the distant coverage while retaining a measurable
reduction in extras. Its remaining range loss is reported below.

Setting both sparse point thresholds to zero reproduces the preceding decisions.
The paired timing baseline uses this setting and checks selected pillar identities
against frozen previous records on every timed frame. Both timed variants share
diagnostic support extents.

Development ablation (`ablation_development.csv`):

| Variant | Extra pillars | Fine point coverage | Positive pillar recall |
|---|---:|---:|---:|
'''
labels={'baseline':'Previous','sparse_shaft_only':'Fixed sparse shaft gate only','weak_owner_only':'Fixed weak owner gate only','fixed_count_combined':'Both fixed gates, rejected','blanket_owner_20':'Blanket 20-point minimum, rejected','count_scaled_combined':'Count scaling, rejected','range_geometry_combined':'Installed bounded geometry relaxation'}
for r in csv.DictReader((P/'ablation_development.csv').open()):
 installed+=f"| {labels[r['variant']]} | {r['extra']} | {100*float(r['coverage']):.2f}% | {100*float(r['recall']):.2f}% |\n"
t=t[:a]+installed+'\n'+t[b:]
a=t.index('## Full-sequence and transfer results');b=t.index('## Validation and timing')
quality='''## Full-sequence and transfer results

Raw reference point XY is projected into exact 0.6 m owners, without dilation,
nearest-neighbor tolerance, old coarse ground truth or ground-filter denominator
exclusions. Rates pool counts across frames. These are not per-frame means or
physical-object recall. `full_frames.csv` concatenates disjoint runs of frames
1:780 and 781:1170. Downtown uses `round(linspace(1,539,24))`.

| Dataset | Extra pillars, previous -> current | Extra fraction, previous -> current | Fine point coverage, previous -> current | Positive pillar recall, previous -> current |
|---|---:|---:|---:|---:|
'''
percent=lambda v:f'{100*v:.2f}%'
labels={'Mississippi_full':'Mississippi, 1170 frames','Mississippi_development':'Development prefix, 780 frames','Mississippi_temporal':'Temporal suffix, 390 frames','Downtown':'Downtown, 24 frames'}
for name,r in s['results'].items():
 old,new=r['baseline'],r['current'];quality+=f"| {labels[name]} | {old['extraPillars']} -> {new['extraPillars']} (-{percent(r['extraCountReduction'])}) | {percent(old['extraFraction'])} -> {percent(new['extraFraction'])} | {percent(old['pointCoverage'])} -> {percent(new['pointCoverage'])} | {percent(old['pillarRecall'])} -> {percent(new['pillarRecall'])} |\n"
for name in ['Mississippi_full','Downtown']:
 r=s['results'][name]['current'];quality+=f"\n{labels[name]}: {r['coveredPoints']}/{r['finePoints']} fine points covered; {r['matchedPillars']}/{r['targetPillars']} reference pillars matched; {r['selectedPillars']} selected pillars. Point miss rate: {percent(r['pointMissRate'])}.\n"
quality+='''
Extra fraction is `FP / selected`, not the background false-positive rate
`FP / (FP + TN)`. Counts refer to frame/pillar occurrences, not unique physical
poles. The primary point-containment measure exceeds 80% in both pooled datasets;
Downtown positive-pillar recall remains below 80%. Remaining extra fractions are
high, and no claim of universal per-frame 80% coverage is made.

Distance stratification on Mississippi is critical: final point coverage is
97.78% at 0-10 m, 89.58% at 10-20 m and 66.47% at 20-30 m, compared with preceding
97.86%, 91.45% and 76.17%. The 20-30 m denominator is 4674 points; the final change
loses 453 previously covered points there. Positive pillars with fewer than 10
reference points have 37/116 recall; those with at least 100 have 570/579.
This iteration improves false selection with an explicit remaining sparse/far
tradeoff; it does not solve the full precision/recall problem.

![Measured quality tradeoff](quality_comparison.png)

'''
t=t[:a]+quality+t[b:]
a=t.index('## Validation and timing');b=t.index('## Reproduction and artifact scope')
validation=f'''## Validation and timing

All {s['tests'].get('Passed',0)} selected MATLAB tests pass with zero failures or incomplete cases
in the final combined result. Seven new controls cover tight sparse support,
diffuse/leaning sparse rejection, dense tilt allowance, peripheral ownership,
distant sparse tolerance and persistent wide-surface rejection. Existing mixed
pillar, boundary, native/MATLAB parity, original fine and mapping assertions remain.
The first distant fixture crossed a second pillar unintentionally; its translation
was corrected. The affected 18-test suite was rerun and replaced in the full-run
result; `fixture_initial_tests.csv` preserves the initial failure. The clean full
reproduction command remains `validateContextAlignment`.

All 1194 evaluated frames retain identical ground/filter source counts and non-pole
Gaussian component fields, except the global mixture-weight normalization that
changes when pole components change. Code Analyzer output and exact results are
in `code_analysis.csv` and `tests.csv`.

Paired runtime uses 24 preloaded frames per dataset and three alternating warmed
repetitions, or 72 measurements per variant/dataset. I/O and configuration are
excluded. Median milliseconds, disabled-gate preceding profile -> current:
'''
for name,r in s['timing'].items():validation+=f"\n- {name}: {r['1']['medianMs']:.3f} -> {r['2']['medianMs']:.3f} ms.\n"
validation+='''
These workstation measurements do not establish hard real-time operation. Earlier
studies' absolute timings are not used as the baseline for this paired comparison.
See `paired_runtime.csv`, `summary.json` and `residual_groups.csv`.

'''
t=t[:a]+validation+t[b:]
t=t.replace('exportExpandedContext;\nexportContextContinuity;', 'exportExpandedContext;\nexportContextContinuity;\nexportDevelopmentRanges;')
p.write_text(t)
