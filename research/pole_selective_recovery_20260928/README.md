# Mississippi pole discrimination: specialist model and minority-shaft protection

The deployed Mississippi 0.6 m pole discriminator now selects the missed shaft
in frame 857. It uses a Mississippi-specific distribution model and a strict
geometric protection for continuous minority shafts amid other-height clutter.
All source statistics and output moments still use the existing whole pillars.
There is no finer XY lattice, online fine-label lookup, frame-specific rule,
map feedback into detection, or additional online learned model.

## Measured results

False selection means a selected pillar contains zero original fine-reference
pole points. These detector references are imperfect, not independent physical
truth. The following raw replay counts cover the existing 1,170-frame route.

| Measure | Previous | Deployed |
|---|---:|---:|
| Selected pole pillars | 1,533 | 1,536 |
| Reference-empty pillars | 64 | 36 |
| Reference-empty fraction | 4.17% | 2.34% |
| Covered reference points | 151,658 / 194,300 | 153,325 / 194,300 |
| Point coverage | 78.05% | 78.91% |
| Reused suffix empty fraction, 781:1170 | 39/505 = 7.72% | 35/471 = 7.43% |
| Reused suffix point coverage | 70.46% | 68.05% |
| Frame 857 pole coverage | 0/13 | 13/13 |
| Frame 857 recursive position error | 47.39 cm | 7.80 cm |
| Frame 827 recursive position error | 9.38 cm | 8.14 cm |
| Largest error after frame 1 | 47.39 cm, frame 857 | 41.05 cm, frame 856 |
| Full-route position RMSE | 13.46 cm | 13.29 cm |

The tradeoff is explicit: suffix coverage decreases by 2.41 percentage points.
This is consistent with prioritizing precision and recovering the inspected
shaft, but does not establish universally improved recall. The full-route
aggregate includes the training prefix and is more optimistic than the reused
suffix. Neither is an independent-drive generalization benchmark. Initial
frame 1 still has the configured 64.03 cm error without a successful match.

Paired full-perception timing on 14 frames with three alternating-order repeats,
discarding each frame's first repeat, gives medians **141.21 ms before** and
**140.60 ms after**. The sub-millisecond difference is noise, not a speedup
claim; no material additional latency was observed in this limited check.
Map-matching parameters, map, motion propagation, curb and sign decisions are
unchanged. Frame 857 has one actual temporally confirmed/matched pole.

## Implemented decision

`pillarPoleDistributionConfig` chooses
`mississippiPolePillarDistributionModel.json` only for Mississippi/Missisipi.
Downtown keeps the original shared model and threshold; offline 0.3 m perception
is unchanged. The new model uses the existing 143-feature schema of shaft
height/continuity, radial concentration, context, owner support and joint XYZ
moments. Its operating threshold is 0.8747229048074372; scores from different
models are not directly comparable probabilities.

Six Mississippi-only candidates were fitted on frames 1:780 with five contiguous
folds and a 20-frame purge. Each uses 250 histogram-boosting trees, learning
rate 0.05, L2=2 and seed 928. The selected depth-5/minimum-leaf-10 model uses
point-support weighting power 1. Selection maximizes prefix out-of-fold covered
reference points with at most 7.5% reference-empty selections. It obtains
96,924 covered points and 74/992 empty selections (7.46%) in that selection
protocol, versus the previous shared calibration's 97,395 and 74/1,002.
The lower OOF coverage is disclosed; suffix results were inspected only after
freezing the specialist. The suffix has already been used for development,
including identification of frame 857, so it is not a fresh holdout.

`protectMinorityPoleShaft` addresses a regression found by the existing synthetic
minority-shaft test: the specialist alone rejected a 60-point continuous shaft
amid 800 other-height clutter points. The protection requires all of:

- Support height at least 2.5 m and contiguous support at least 90% of height.
- Radial RMS at most 0.04 m, isolation at least 0.95, and at least 90% of
  supported-height structural context within 0.15 m of the axis.
- Owner support fraction at most 0.2 and owner support height at least 2 m.
- At least four core points in each height quarter and adjacent-quarter center
  displacement at most 0.08 m.

These distribution tests accept a supported subset despite unrelated owner
returns. They promote only qualifying hypotheses to the learned acceptance
threshold; the established whole-pillar moments are retained. They do not
activate for any recorded candidate in this route, and a second raw replay
confirms unchanged route selections. They do restore the continuous-minority
regression, while an added disconnected-minority-blobs-with-clutter test stays
rejected. This synthetic protection is not claimed to have broad field
validation beyond the executed checks.

## Explored alternatives

Simple score-plus-height/radius/isolation rescue and a concentration/quarter-axis
variant were selected using prefix OOF scores. The strict initial incremental
5% budget yielded no eligible rule; relaxing to incremental 10% and pooled 8%
yielded only small gains and failed to recover frame 857. They are not deployed.
A shared/specialist consensus also failed to preserve additional old detections
under the 7.5% OOF budget; its selected floor accepts no extra rows. Runtime
therefore evaluates only the specialist, avoiding doubled classifier work.

## Validation and reproduction

- `trainSpecialist.py` trains/freezes/exports the specialist and expected owner
  IDs. The three `select*.py` scripts preserve unsuccessful alternative searches.
- `validateSpecialist` compares all 13,767 candidate scores between Python and
  MATLAB (maximum difference 4.44e-16), checks identical point routing and
  curb/sign pillar IDs on 14 frames, and measures paired full-perception timing.
  It explicitly pins the previous model/threshold for reproducibility.
- `replaySpecialist` recomputes all raw frames and recursively matches them with
  the current map and frozen independent wheel/gyro/observer odometry. It uses
  the existing INS-tilt projection convention, with no reference XY/yaw resets.
  Every raw pole owner set matches the frozen prediction exactly.
- `verifyDeployedPoles` repeats raw 1,170-frame pole inference after adding the
  geometric protection. Every owner set and metric matches the first replay.
  Thus the protection leaves the cached matching trajectory's inputs unchanged;
  the expensive matcher is not redundantly replayed a second time.
- 32 tests pass across `pillarPoleDistributionTest`, `mississippiCurbRecoveryTest`
  and `semanticPillarPrecisionTest`, including new sparse-frame-857 and
  disconnected-minority regressions. Existing original fine-mask regressions
  and native/MATLAB agreement checks pass. Eight MATLAB files have zero factory
  Code Analyzer findings. Python compilation and scoped whitespace checks pass.
- `summarizeStudy.py` independently checks both raw replays, aggregate counts,
  trajectory errors against recorded poses, deployed model identity, tests and
  viewer metadata. First replay startup encountered CSV boolean text; comparison
  now uses numeric score/threshold. An audit briefly used pandas Series.empty
  instead of the named column; bracket indexing fixes that audit. The initial
  specialist test failure is retained in this report, not represented as a pass.

Use `uv run --offline --with numpy --with pandas --with scikit-learn python`
for training scripts. In MATLAB, add the repository root, call
`setupVehicleLocalization`, then add this study directory before invoking its
functions. Local inputs include `output/pole_geometry_20260927/`, frozen fine
references, raw Mississippi scans, and the prior map/motion cache. Large
feature tables, model pickles and detailed MAT results remain under `output/`.
The exported production JSON and compact study evidence are committed.

The niri viewer was refreshed to `Mississippi 857 | improved pole classifier`,
retaining the camera and all 65,536 source points. It shows one selected pole
pillar covering all 13 stored reference points; curb/sign highlights remain
unchanged. Screenshots and metadata are included. Remaining curb errors, the
map/trust-gate issue, and the new maximum at frame 856 are not claimed resolved.
