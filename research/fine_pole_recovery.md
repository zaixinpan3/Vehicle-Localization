# Fine pole recovery for Mississippi frame 260

Date: 2026-09-09. Baseline revision:
`31514822b52931627243e933217a474f5e849cbf`.

The default fine detector now accepts original point 63010 (row 34, column 985),
XYZ `[-17.912437, -4.260993, 1.158267]` m, and 16 additional nearby points.
Frame 260 changes from 119 to 136 accepted pole points. Every previously
accepted pole point remains accepted, and other semantic masks are unchanged.
This change does not classify every return along the complete pole: individual
support and distance tests still reject insufficiently supported returns.

## Parameter decision

Enable the existing geometrically constrained fine recovery by default through
`fine.poleRecoveryEnabled=true`. No point index, frame number, coordinate region,
or dataset-specific exception is embedded in the detector. The coarse algorithm
and its whole-pillar representation are unchanged.

Recovery requires a coarse pole candidate with at least 12 nonground returns,
at least 3 m metric height, no more than 2 degrees whole-pillar regression tilt,
and no more than 0.10 m transverse standard deviation. It must also pass the
fine structural candidate test with three consecutive occupied support bins.
The original candidate test remains at four. Fine support ratios and robust
per-point line-distance validation still apply. Original and added candidates
are fitted separately, preserving previously accepted points.

An alternative global reduction of the core threshold from four to three was
probed and rejected: Mississippi frame 100 gained 87 points but lost 64 original
pole points. The constrained recovery kept every original point on the evaluated
frames. This is a parameter-selection decision using the existing current
algorithm; it does not restore an obsolete implementation.

## Executed checks

The 30 frozen frames from `tests/reference/perceptionMasks.json` produced only
these additions, with no removed pole points or changes to other masks:

| Dataset | Frame | Original pole points | Added pole points |
| --- | ---: | ---: | ---: |
| Mississippi | 100 | 432 | 12 |
| Mississippi | 260 | 119 | 17 |
| Mississippi | 800 | 33 | 119 |
| Downtown | 100 | 270 | 13 |

The other 26 frames were unchanged. Local 3D and XY scatter plots of all four
additions were inspected; the additions exhibit narrow vertical structure.
In particular, frame 800's larger addition follows a dense narrow vertical
column. This geometric inspection is not independently labeled ground truth
and does not establish a measured false-positive rate.

An additional 15 frames were checked after selecting the recovery configuration:
Mississippi 50, 175, 250, 350, 425, 525, 650, 725, 825, 950; Downtown 75, 150,
250, 350, 450. Current recovery-on/off comparisons had identical fine masks and
coarse candidates on all 15. These are additional regression checks, not a new
labeled benchmark or proof of generalization.

Full suite: **327 passed, zero failed, two synthesis tests filtered** because
YALMIP/SDP dependencies were unavailable. Factory Code Analyzer: **zero findings
in seven changed/new MATLAB files**. The default-setting regression explicitly
requires point 63010, exactly 17 additions within 0.35 m XY of that point, and
preservation of the original frame-260 pole mask. Thirty-frame exact regression
expectations combine the unchanged original fixture with the explicit 161-point
correction in `tests/reference/finePoleRecovery.json`; the old fixture was not
regenerated. This replaces a broad historical pole F1 tolerance with exact
expected corrections and preservation checks.

The visible niri MATLAB preview was recomputed with the default configuration,
zoomed to the reported pole and annotated `63010: pole`. Pole points are cyan.
All scatter-layer original-index mappings were refreshed and checked against
source XYZ; datatips continue printing original point indices in the desktop
Command Window. The desktop screenshot confirms 136 pole, 797 curb, 83 road
marking and 52 traffic-sign points. Display timing includes a separate desktop
session and is not a real-time benchmark.

## Reproduction

```matlab
setupVehicleLocalization;
frame = loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',260);
cfg = perceptionConfig();
cfg.executionMode = "offline";
result = perceiveFrame(frame,cfg);
assert(result.featureMasks.pole(63010));
assert(nnz(result.featureMasks.pole)==136);
runtests('tests');
```

Set `cfg.fine.poleRecoveryEnabled=false` to measure the contribution of this
current detector option. CSV/JSON exports are in
`research/results/fine_pole_recovery_20260909/`. Original recorded clouds and
local probe/plot artifacts remain outside Git. No full trajectory replay or
new real-time qualification was performed.
