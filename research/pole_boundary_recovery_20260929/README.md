# Split pole recovery on the 0.6 m Mississippi lattice

The production change restores the missed pole in frame 856 without lowering
the learned classifier's global threshold. Its 18 original fine-reference
points occupy pillars 7944 and 7945 (11 and 7 points). The existing shared
shaft hypothesis was valid, but per-owner scores 0.23665 and 0.02399 rejected
both owners at the 0.87472 operating point.

## Implementation

`recoverSplitPoleShaft` adds a narrowly scoped physical acceptance path to
`classifyPillarPoleSupport`, enabled only by the Mississippi profile. It uses
statistics already measured for the existing hypotheses, with no second grid,
new learned model, reference lookup, frame identifier, world-coordinate gate,
or temporal input. The original model artifact and score threshold remain.

A hypothesis must be narrow, vertically continuous, approximately upright and
isolated, with a stable transverse center in four populated height quarters.
At least two owners of that same hypothesis must independently contribute
five accepted returns spanning one meter, at least 20% of its accepted points,
and lie within 0.1 m of the axis. Each participating owner must also contain
other-height clutter: the shaft accounts for 30--80% of its points and at most
80% of its whole-pillar height. This last condition specifically protects a
subset obscured by whole-pillar clutter; it is not a general neighbor dilation.
The complete numeric settings are in `recovery_config.json` and the production
configuration. Qualifying owners receive the existing acceptance threshold as
their evidence score. These scores are ranks, not calibrated probabilities.

All final means and covariances still use the original whole 0.6 m off-ground
pillar. No point mask, sub-pillar Gaussian or fine-perception output is used
online. Downtown and fine perception do not enable this recovery.

## Investigation and limits

Broad geometric relaxation was rejected: one initial concentrated/continuous
rule increased reference-empty selections from 36 to 617. Requiring two owners
alone still introduced empty selections. The additional clutter-subset condition
separates the diagnosed failure from otherwise complete, densely populated
columns. `exploreGeometry.py` and `exploreSplit.py` preserve the intermediate
sweeps; these alternatives were not deployed.

A separate owner-independent shaft classifier was also investigated. Four
combinations of any/all-positive hypothesis labels and tree depth 3/5 were
trained only on frames 1--780, with five contiguous folds and 20-frame purges.
The operating point required at least ten added owners, at most 10% incremental
reference-empty owners and at most 8% pooled empty owners. Only the any-positive,
depth-5 model yielded an operating point; it missed frame 856 and introduced
one empty suffix owner. No additional model is deployed. `exploreAxis.py`
contains this rejected experiment; model pickles remain under ignored output.

**The final physical rule was developed using this already inspected route,
including frame 856 and full-route candidate audits. Neither the suffix nor
the prefix is an independent validation of that rule.** Only three frames gain
owners, so zero newly empty owners on this route is limited regression evidence,
not a general false-positive guarantee. Fine-reference labels are the original
0.3 m detector output, not manually exhaustive physical ground truth.

## Detection results

| Partition | Selected, before / after | Empty, before / after | Point coverage, before / after |
|---|---:|---:|---:|
| All 1,170 frames | 1,536 / 1,542 | 36 / 36 | 78.91% / 79.09% |
| 1--780 | 1,065 / 1,067 | 1 / 1 | 84.97% / 85.01% |
| 781--1170 | 471 / 475 | 35 / 35 | 68.05% / 68.47% |

All-route reference-empty fraction changes from 2.3438% to 2.3346%; the suffix
changes from 7.4310% to 7.3684%. Six added pillars in frames 685, 820 and 856
cover 340 additional reference points, with zero added reference-empty pillars.
Frame 856 selects exactly two pole pillars and covers all 18 reference points.
`verifyRecovery` recomputes every raw scan and asserts exact owner-ID agreement
with `expected.csv`, then measures point coverage directly from raw coordinates.

## Recursive matching result

Frame 856 changes from **41.05 cm to 7.22 cm** position error. Its source window
now contains one confirmed pole and the matcher accepts the refinement. The
remaining maximum after initialization is frame 854 at **39.64 cm**; the first
frame retains its configured 64.03 cm initial error. Route RMSE changes from
13.2903 cm to 13.2391 cm and MATLAB `prctile` P95 from 24.4939 cm to 24.2411 cm.
Full/directional update counts remain 1,152/10. This is an actual complete
causal replay with production perception, not the previous reference-assisted
pole intervention. The nearby frame-854/855 localization issue remains.

## Validation and reproduction

Run from the repository root with MATLAB R2026a:

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/pole_boundary_recovery_20260929');
verifyRecovery;
replayRecoveryRoute;
finishRecovery;
assertSuccess(runtests({'tests/pillarPoleDistributionTest.m', ...
    'tests/mississippiCurbRecoveryTest.m','tests/semanticPillarPrecisionTest.m'}));
validateRecovery;
checkRecovery;
```

The tests cover the actual frame-856 regression, unchanged whole-pillar moments
and non-pole masks, unsupported owner rejection, hypothesis isolation, broken
vertical support, elongated cross sections, row permutation and empty inputs.
Existing wall, disconnected blob, minority shaft, crop and fine-mode checks
also remain. MATLAB/native results agree on all classes in the three affected
frames. Paired timing uses 15 frames, three alternating-order repeats; warmed
medians are 150.59 ms before and 147.26 ms after. Concurrent replay and normal
runtime variation preclude a speedup claim; no substantial added cost is seen.

`replayRecoveryRoute` recomputes perception and recursively matches all 1,170
frames with the configured map, unchanged matching settings and source-window
rules. Frozen independent wheel/gyro/observer motion inputs and the same initial
pose isolate the perception change; existing reference tilt projection remains,
with no intermediate reference XY/yaw resets. `full_route.csv`,
`replay_summary.json` and `validation.json` contain the final matching results.

The updated niri viewer uses all 65,536 raw points at uniform size 4, recolors
original fine feature points in place, and shows the nearby 0.6 m floor grid.
Orange denotes pole, magenta traffic sign and cyan curb. No feature scatter
is superimposed. Frame-856 curb and sign selections remain unchanged; the
existing seven reference-empty curb pillars are not fixed by this pole change.
