# Coarse perception on a 0.6 m, 100 x 100 pillar lattice

Date: 2026-09-24. Baseline revision: `796c737344e6c29c3e7aa8869aab2630dc0ebbb1`
(0.3 m, 334 x 334 pillars for both products). Data:
`data/raw/MissisipiPointClouds.mat`, all 1170 scans.

The user asked for the online coarse perception to run on 0.6 m x 0.6 m
pillars, 100 x 100 cells in total, with the parameters adjusted so that the
perception result stays close to the previous one. The offline product that
builds the map keeps its 0.3 m lattice.

## Design

- `pillarGridConfig(executionMode)`: the lattice belongs to the execution
  mode. `coarseProbabilityCloud` (default) is 100 x 100 pillars of 0.6 m
  covering [-29.9, 30.1) m; `offline` keeps 334 x 334 pillars of 0.3 m
  covering [-50, 50.2) m. Both keep `latticeOffset = [0.1 0.1]`, so every
  coarse cell boundary -29.9 + 0.6 k coincides with an offline boundary
  -50 + 0.3 j and each coarse pillar is exactly the union of four offline
  pillars. The 60 m window follows from the requested 100 cells.
- `perceptionConfig(dataset, executionMode)` builds the lattice and passes
  its spacing to `groundSegmentationConfig`, `groundFeatureConfig`,
  `structuralPillarConfig` and `coarseSemanticProbabilityCloudConfig`. The
  stage configs state both tuned values side by side through
  `latticeTunedValue(spacing, value03, value06)`; a spacing other than 0.3 or
  0.6 m is rejected. `perceiveFrame` raises `perception:LatticeModeMismatch`
  when `cfg.executionMode` is switched after construction, and every script
  and test now asks for the mode at construction time. Mapping
  (`buildFeatureMap`, `collectFeatureObservations`) requires the offline
  configuration explicitly.
- The coarse output grid is two whole pillars (1.2 m) on the 0.6 m lattice and
  three (0.9 m) on the 0.3 m lattice, so output cells never cut pillars.
- The offline configuration is unchanged: `groundFeatureConfig(0.3)`,
  `groundSegmentationConfig(0.3)`, `structuralPillarConfig(0.3)`,
  `coarseSemanticProbabilityCloudConfig(pillarGridConfig("offline"))` and
  `perceptionConfig("Mississippi","offline")` are `isequaln` to the
  `796c737` functions (apart from the new `voxel.executionMode` tag), so all
  recorded fine references remain valid.

## Method

`captureCoarseSequence` stores, per scan, the candidate pillar IDs of every
channel, road and curb cells, pole and sign cells with their evidence, the
Gaussian components and the timing. `compareCoarseSequences` projects the
baseline 0.3 m cells by their centres into the 0.6 m lattice (restricted to
the 60 m window) and reports precision, recall and F1 both on exact cells and
with a one-cell (0.6 m, 8-neighbour) tolerance, nearest-centre distances,
component counts and per-class mixture weights. `curbRangeAgreement` splits the
curb agreement by range, `curbBandThickness` counts curb cells per x-column and
road side, `analyzeCurbFeatureShift` reports feature quantiles on baseline curb
and road cells for both lattices, and `sweepPoleGates` tabulates the whole-pillar
pole statistics of every candidate pillar against the baseline poles. Parameters
were tuned on every tenth scan (117 frames) and the final configuration was
scored on all 1170 scans.

Three parameter audits (ground segmentation and road, curb energy and
refinement, structural branch) classified every cell-count and density
parameter before tuning. The energy stage was kept cell-identical: windows,
seed counts and component sizes stay in cells while the metric lengths that
describe cell-scale structures (linearity window, band length and width
limits) double. Gap, span and search radii of the refinement stages are
physical lengths and were halved in cells. Metric feature targets were
re-derived from the measured shift on the same cells (frames 28, 94, 260, 384,
458, 600, 700, 855, 1000, 1137):

| Feature (baseline curb / road quantiles) | 0.3 m | 0.6 m | 0.6 m target |
|---|---|---|---|
| Height step, curb median / road P90 | 0.072 / 0.024 m | 0.095 / 0.059 m | min 0.055, target 0.105, sigma 0.035, max 0.35 m (was 0.04 / 0.08 / 0.03 / 0.30) |
| Residual slope, curb P75 / road P90 | 0.121 / 0.025 rad | 0.094 / 0.046 rad | 6.0 deg, sigma 2.0 deg (was 7.0 / 3.0) |
| Curvature, curb P90 / road P90 | 1.19 / 0.33 | 0.47 / 0.14 | 0.40, sigma 0.14 (was 1.0 / 0.35) |
| Roughness, curb median / road P90 | 0.010 / 0.009 m | 0.032 / 0.012 m | 0.055, sigma 0.025 m (was 0.04 / 0.02) |
| Linearity, curb median | 0.41 | 0.13 (before rescaling) | metric window and width limits doubled |

Minimum height-step gates of the refinement stages were raised by the
measured curb ratio (about 1.3) and roughness gates by 1.5. Curb cells need
three ground points on the 0.6 m lattice (one on 0.3 m): with one point the
far-range cells (20--30 m) produced curbs the baseline never had. The
road-facing shoulder recovery is disabled on the coarse lattice because it
added a second 0.6 m row to bands that the 0.3 m product carries in 0.74 m.
The road roughness limit is 0.04 m (0.12 m) so that cells straddling a curb no
longer let the road flood the verge. Poles keep every metric gate, and the
shape-score neighbourhood stays at two cells so that a pole split over a cell
boundary keeps its point contrast. The radial-scatter gate alone does not
survive the wider pillar: baseline poles have a radial standard deviation of
0.134 m (median), 0.163 m (P75) and 0.188 m (P90) inside 0.6 m pillars, while
other pillars with 12 points and 1.5 m height start at 0.172 m (P10), so any
limit either loses a third of the poles or admits most clutter. The user's
observation that a pole pillar has one XY location of high return density
whose returns span a large height led to `computePillarDensityCore`: the
densest XY spot of the pillar (0.1 m histogram peak refined by one mean shift
inside 0.15 m) and, for the returns within 0.15 m of it, their share of the
pillar (`coreFraction`) and z range (`coreHeight`). On the same pillars the
share is 0.68 (median) for baseline poles and 0.31 for the rest
(`probePoleCoreStatistic.m`). The 0.6 m core gate requires
`coreFraction >= 0.55` and `coreHeight >= 1.5 m`, relaxes the radial limit to
0.25 m, the point score to 0.60 and the context fraction to 0.52; the 0.3 m
lattice keeps its former gates with the core thresholds at zero. Six real runs
(`cand7*_vs_baseline.csv`) chose these values by keeping the pole count and
mixture weight at the baseline level.

## Results on all 1170 scans (`final2_vs_baseline.csv`)

Agreement with the 0.3 m coarse product inside the 60 m window. Tolerant
values allow one 0.6 m cell.

| Channel | Baseline cells | 0.6 m cells | Precision | Recall | F1 | Precision (tol.) | Recall (tol.) | F1 (tol.) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Curb | 121281 | 99361 | 0.744 | 0.609 | 0.670 | 0.935 | 0.878 | 0.905 |
| Pole | 9129 | 8040 | 0.523 | 0.460 | 0.489 | 0.571 | 0.503 | 0.534 |
| Pole, radial-std gate only (`final_vs_baseline.csv`) | 9129 | 9982 | 0.416 | 0.455 | 0.435 | 0.453 | 0.482 | 0.467 |
| Traffic sign | 4503 | 4625 | 0.972 | 0.998 | 0.985 | 0.974 | 0.999 | 0.987 |
| Road | 273497 | 273759 | 0.749 | 0.749 | 0.749 | 0.796 | 0.877 | 0.835 |

Curb agreement by range (exact / tolerant recall): 0--10 m 0.635 / 0.900,
10--20 m 0.632 / 0.905, 20--30 m 0.213 / 0.459; the far band holds 5.7% of the
baseline curb cells. Curb bands are 1.75 cells per column and side (1.05 m)
against 2.46 cells (0.74 m) on the 0.3 m lattice. Mean nearest-centre distance
from a 0.6 m curb cell to a baseline curb cell is 0.44 m. Per scan the cloud
carries 41 curb, 5.6 pole and 2.5 sign components (baseline 67, 6.7 and 2.6 at
0.9 m); the class mixture weights are 0.793 / 0.153 / 0.054 (baseline
0.801 / 0.146 / 0.053). Median coarse perception time per scan is 30.0 ms
against 54.0 ms (1.80x), same session, native kernels; the density core costs
about 2 ms of that.

Pole agreement remains the weak channel. A 0.6 m pillar merges the shaft with
brackets, signs and foliage that the 0.3 m lattice kept in neighbouring
pillars, and the fixed one-cell context ring grows from 0.9 m to 1.8 m.
Sweeping the whole-pillar gates alone (radial std, context fraction, point
score) never exceeded F1 0.47; the density core lifts exact F1 from 0.435 to
0.489 and tolerant F1 from 0.467 to 0.534 at the same pole mass.

## Downstream: production localization chain

`runMncavCoarseLocalizationExperiment` reran the full chain (fresh coarse
perception, recursive matching, source regeneration, observer scenarios) for
the 0.6 m lattice with the radial-std pole gate
(`output/mncav_coarse_localization_20260924c`) and with the final density-core
pole gate (`..._20260924d`) against `..._20260924b` (0.3 m lattice, same map,
matcher and gains).

Recursive LiDAR-only matching stage (odometry seed, no GNSS, no observer):

| Chain | Accepted | RMSE (cm) | Median | P95 | Max | > 30 cm | Yaw RMSE (deg) | Total ms/scan (median) | Perception ms | Registration ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.3 m (b) | 1169 | 12.65 | 9.13 | 22.97 | 39.34 | 18 | 0.444 | 75.8 | 58.4 | 16.9 |
| 0.6 m, radial-std pole gate (c) | 1165 | 12.84 | 9.30 | 25.22 | 40.23 | 22 | 0.539 | 49.9 | 30.8 | 18.9 |
| 0.6 m, density-core pole gate (d) | 1164 | 12.64 | 9.17 | 24.39 | 38.13 | 20 | 0.524 | 53.4 | 34.4 | 19.0 |

GNSS-aided matching inside the fused run: 11.22 cm RMSE (b), 11.47 cm (c),
10.89 cm (d). Run d was timed while the full test suite ran on the same
machine, so its per-scan times (and its 46 calls above 100 ms) are inflated;
the isolated perception measurement is 30.0 ms per scan.

Observer scenarios (`compareLatticeProduction.m`, `scenario_comparison.csv`,
`matching_stage_comparison.csv`; errors against INSPVA, RMSE after the common
2 s initialization transient):

| Scenario | b RMSE (cm) | c | d | b P95 | d P95 | b max | d max | b > 30 cm | d > 30 cm | b yaw (deg) | d yaw |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| GNSS + LiDAR | 5.42 | 5.01 | 4.85 | 9.90 | 9.10 | 17.45 | 16.68 | 0 | 0 | 0.389 | 0.473 |
| LiDAR only | 14.24 | 12.92 | 13.04 | 26.70 | 22.84 | 38.10 | 34.07 | 39 | 19 | 0.421 | 0.495 |
| GNSS only | 5.78 | 5.78 | 5.78 | 11.08 | 11.08 | 14.87 | 14.87 | 0 | 0 | 1.330 | 1.330 |
| GNSS outage 40--60 s | 6.17 | 5.98 | 5.77 | 12.38 | 11.21 | 18.71 | 20.21 | 0 | 0 | 0.391 | 0.479 |
| LiDAR outage 40--60 s | 5.51 | 5.08 | 4.98 | 9.91 | 9.18 | 17.45 | 16.68 | 0 | 0 | 0.380 | 0.446 |
| Both outage 40--60 s | 28.85 | 21.94 | 24.27 | 67.43 | 63.03 | 128.23 | 95.34 | 164 | 165 | 0.373 | 0.445 |
| Alternating 1 s | 15.40 | 13.59 | 13.62 | 26.80 | 24.61 | 40.45 | 34.15 | 26 | 20 | 0.415 | 0.484 |

With the density-core pole gate the recursive matching RMSE equals the 0.3 m
chain (12.64 against 12.65 cm) and the fused position accuracy improves
(5.42 to 4.85 cm fused, 14.24 to 13.04 cm LiDAR only), while heading RMSE
rises by about 0.1 deg in every LiDAR-bearing scenario, consistent with the
wider curb Gaussians. The both-outage scenario differs through the
dead-reckoning bias learned before the outage, as in the pyramid study, not
through matching.

## Validation

- Full suite `runtests('tests')`: 630 tests, 621 passed, 1 filtered
  (YALMIP synthesis check), 8 failed before the threshold update. Two of
  those were `coarseSemanticProbabilityCloudTest` regression thresholds that
  scored the 0.3 m coarse product against the 0.3 m fine point masks; they now
  hold the measured 0.6 m agreement (source curb precision/recall
  0.532/0.742, NDT 0.614/0.829 on frames 260/300/326; dispersed recall
  0.775/0.837) with a small margin, and the suite passes 5/5. The other six,
  `currentPerceptionReferenceTest` ground-point expectations for Mississippi
  frames 200, 600, 775, 800, 1050 and 1150, fail identically on a detached
  worktree of `796c737` (25/31 there as well), so they predate this change
  and are left untouched.
- After adding the density core: `wholePillarPerceptionTest` (13/13, with
  two new cases: the core statistic on a shaft sharing its pillar with
  clutter, and the same shared pillar accepted as a pole while the clutter
  alone is not), `pillarPerceptionTest`, `poleCompactSupportTest`,
  `poleBoundaryRecoveryTest`, `structuralSemanticPerceptionTest`,
  `pillarEfficiencyTest` (75/75 together). A second full-suite run gave
  632 tests, 614 passed; besides the six pre-existing reference failures,
  the lateral-observer suites failed or were filtered because another
  session was editing `localization/lateralObserver` in the working tree at
  that moment (files timestamped 19:07, after run d had finished at 19:05);
  those edits are not part of this change.
- Offline configuration equality with `796c737` (see Design) and the
  unchanged pass status of every recorded fine curb, pole and sign test; the
  offline pole gates keep the density-core thresholds at zero.
- `checkcode -config=factory`: no findings in the 48 changed or new MATLAB
  files; `git diff --check` clean.
- Downstream: the production chain rerun above and its `call_path_audit.json`.

## Limits

- Agreement is measured against the previous coarse product, not against
  labelled ground truth; the baseline's own pole precision against the fine
  product was moderate.
- The 60 m window drops features beyond 30 m that the 100 m lattice retained;
  their influence on matching is included in the downstream numbers above.
- Parameters were tuned on every tenth scan of the same drive that the final
  numbers are reported on; the 0.6 m values are therefore in-sample for this
  route.
- Curb bands are about 0.3 m wider than before, which widens the across-curb
  scatter of the curb Gaussians; the downstream matching numbers include that
  effect.

## Artifacts

Drivers (copies of `output/coarse_lattice_20260924/`): `captureCoarseSequence.m`,
`compareCoarseSequences.m`, `curbBandThickness.m`, `curbRangeAgreement.m`,
`analyzeCurbFeatureShift.m`, `sweepPoleGates.m`, `poleGateFailures.m`,
`diagnoseLatticeFrame.m`, `diagnoseCurbEnergy.m`. Results: `final_vs_baseline.csv`
(per scan), `cand*_vs_baseline.csv` (tuning steps on every tenth scan),
`frame0260_masks.png` and `frame0855_masks.png` (both lattices side by side).
Raw sequences and the localization run stay under `output/`.
