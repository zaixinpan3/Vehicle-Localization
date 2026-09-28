# Recovering curb proposals suppressed by road adjacency

The user identified poor curb coverage in Mississippi frame 900 after the
earlier acceptance-model change. This study finds a different bottleneck:
most missed points never reach that model. The deployed recovery reconsiders
omitted raw geometric proposals using whole-pillar statistics and connected
boundary support. It does not change ground segmentation, the original fine
reference, the 0.6 m grid, or the pole/sign models. Downtown is not retuned or
replayed.

## Frame-900 diagnosis

All 99 original fine curb points survive coarse ground routing. They project
into 35 coarse cells. The old candidate stage covers only 54 points; the
existing forest retains 51, in 19 selected cells, two of which contain no
reference curb point.

The missing points divide into three groups:

- 43 points occupy 13 cells along approximately X=1.0:8.2 m, Y=7.0 m. All
  have a raw curb response, but road adjacency removes the entire segment.
  They lie 4--5 cells from the initial road mask, exceeding its 2-cell radius.
  This is distance to an estimated road mask, not physical distance to a road.
- Two points occupy cells with only one or two ground returns, below the
  three-point minimum needed for initial coarse curb evidence.
- Three points reach the proposal set but fail the final forest acceptance.

Disabling road adjacency globally produces 511 candidates and covers 90/99
points, but the unchanged final model still covers only 53. Disabling other
individual topology stages also fails to solve the problem. The complete
ablation and per-cell evidence are in `frame900_ablations.csv` and
`frame900_cells.csv`. Lowering the final threshold alone cannot recover the
45 points outside its input proposals.

## Implemented recovery

The original forest decisions are retained. A separate pool contains only
cells that had raw geometric curb evidence, were omitted by the original
proposal mask, and have base energy at least 0.25. The additional classifier
uses the existing 112 pillar/neighbor raster descriptors; no extra raw-point
fit, finer grid, source-point labels or frame identity is used online.

The model is a histogram boosted classifier with 180 iterations, maximum
depth 6, at most 31 leaves, minimum leaf size 25, learning rate 0.05, L2=2,
balanced classes and seed 928. It admits individually strong scores at
0.9845521548207087. Additional cells need an 8-connected weak-score component:

- Every member has score at least 0.8.
- At least two members, and at least 40% of all members, have score at least 0.9.
- Transverse RMS width is at most 0.15 m and second-moment anisotropy at least 0.9.
- The length proxy `2*sqrt(3*lambda_max)+0.6` is at least 4.2 m, where
  `lambda_max` is the population covariance eigenvalue of cell centers.
  This proxy is not measured arc length.

Geometry descriptors are quantized at 1e-8 in both implementations. The rule
uses a single component evaluation: recovered cells do not seed recursive
dilation. Cells without raw evidence cannot be filled. Accepted cells use
the existing energy-to-probability mapping and complete-pillar moments.
The exported runtime model is approximately 454 KiB. It ranks agreement with
the fine reference and is not a calibrated physical-class probability.

## Validation and selection limits

Model fitting uses frames 1:780 only, with five contiguous temporal folds and
a 20-frame purge around each held-out block. The initial single-cell threshold
uses at most 5% reference-empty selections among prefix out-of-fold predictions.
The raster model covers 2,225 extra prefix points; adding 45 raw distribution
descriptors reduces this to 1,564, while extra trees cover 1,537. None of these
strict individual gates recovers the inspected segment in frame 900.

Chain parameters were then tuned with the inspected frame and reused suffix
labels. The final constraints were pooled prefix out-of-fold reference-empty
fraction <=5.3%, reused suffix <=9.6%, frame-900 coverage at least 80 points
with at most its original two reference-empty cells, and frame-500 coverage
at least 58 points. Within these constraints, prefix covered points were
maximized; equivalent rules prefer more required anchors. This is explicitly
**development and validation on reused data**, not an untouched test. Frame
900 is excluded from model fitting but is used to select the chain policy.

The selected recovery adds 753 prefix out-of-fold cells, 57 reference-empty,
covering 2,410 additional points. Together with the existing forest's prefix
out-of-fold counts, the reference-empty fraction is 1,081/21,243 (5.09%).
Several alternatives either increased frame-900 errors or exceeded the
suffix limit; their results are preserved alongside the selected rule.

| Evaluation | Before | After |
|---|---:|---:|
| Frame 900 covered reference points | 51/99 (51.52%) | 83/99 (83.84%) |
| Frame 900 reference-empty / selected cells | 2/19 (10.53%) | 2/28 (7.14%) |
| Frame 500 covered reference points | 58/64 | 58/64 |
| Suffix 781:1170 covered reference points | 31,210/42,139 (74.06%) | 32,775/42,139 (77.78%) |
| Suffix reference-empty / selected cells | 924/10,045 (9.20%) | 1,005/10,534 (9.54%) |
| All 1,170 frames covered points, including fitted prefix | 112,504/133,836 (84.06%) | 118,451/133,836 (88.50%) |
| All frames reference-empty / selected cells | 1,060/36,051 (2.94%) | 1,302/38,030 (3.42%) |

The inspected frame gains nine reference-bearing cells and 32 covered points,
with no additional reference-empty cell. Sixteen points still remain uncovered:
11 in the unrecovered part of the segment, two in sparse cells and three
rejected by the unchanged forest. The suffix gains 1,565 points and 81
reference-empty cells. Thus coverage improves at a small additional reference
disagreement cost; this is not a reduction in absolute errors throughout the
recording. Aggregate fractions do not guarantee a limit in each frame.

The user also confirmed that frame 900's unique coarse pole at approximately
(4.6,-3.8) m is real despite its absence from the original fine reference.
That separate pillar-level review is preserved in
`../pole_reference_audit_20260928/`. The old point labels remain unchanged.
Consequently, all “reference-empty” fractions above measure detector
disagreement, not established physical false-detection rates. No unreviewed
curb cell was relabeled merely because the new method selected it.

## Reproduction

Large input/reference caches and descriptor/prediction tables remain in local
`data/` and `output/`. The capture joins fixed fine labels only after measuring
current-frame descriptors. It exports 377,445 omitted raw proposals, of which
3,096 contain reference points. Raw descriptors were explored offline; only
the 112 raster descriptors enter the deployed recovery model.

```bash
matlab -batch "setupVehicleLocalization; addpath('research/curb_proposal_recovery_20260928'); diagnoseCurbProposals(900); captureCurbRecoveryCandidates"
uv run --with scikit-learn==1.9.1 --with pandas python research/curb_proposal_recovery_20260928/trainRecoveryGate.py
uv run --with pandas --with scipy python research/curb_proposal_recovery_20260928/probeRecoveryChains.py
uv run --with pandas --with scipy python research/curb_proposal_recovery_20260928/evaluateRecoveryChainTradeoff.py
uv run --with scikit-learn==1.9.1 --with pandas python research/curb_proposal_recovery_20260928/exportRecoveryGate.py
matlab -batch "setupVehicleLocalization; addpath('research/curb_proposal_recovery_20260928'); validateCurbProposalRecovery; replayCurbProposalRecovery; benchmarkCurbProposalRecovery"
```

`replay.csv` records every raw frame against frozen references. The replay
asserts exact agreement with the exported core-plus-recovery cell sets,
unchanged preprocessing, and unchanged pole/sign IDs. Focused tests retain
fine-reference equality on frames 300/500/900 and verify both backends,
single-channel inference, empty inputs, original selection preservation,
raw-evidence-only additions and unchanged full-pillar moments. The benchmark
uses six frames and five repetitions in alternating old/new order; model load
and disk reads are excluded. `summary.json` and `runtime.csv` contain final
measurements. `frame900/` preserves a comparison export of the live niri view.

Final validation passed **12/12 tests**, with no failed or incomplete cases,
and all **1,170** raw-frame comparisons. Factory Code Analyzer found no
issues in the checked files. The paired 60-run all-channel benchmark measured
a median of **127.5325 ms before and 140.157 ms after** (9.90% higher).
Mean times were 125.0724 and 136.8627 ms; 95th percentiles were 158.0613 and
164.7169 ms. The median paired difference was 12.836 ms. These are warm
in-process measurements on this machine, not a real-time deployment guarantee.

The initial diagnostic ablation needed the pre-adjacency mask restored when
that entire stage was disabled. The first raw replay also exposed a harness
CSV boolean-import mismatch; it was corrected before complete replay. These
were research-harness issues, not reported as passed full runs.
The result summarizer also needed a null coverage value for frame 900's sign
channel, which has zero reference points inside the ROI; no denominator was
invented and no reference points were removed.
