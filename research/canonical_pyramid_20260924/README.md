# Canonical map pyramid in production registration

Date: 2026-09-24. Implements Route C of
[the mode-ambiguity study](../alias_hypotheses_20260924/README.md) in the
production registration driver and reruns the full production chain.
Previous production revision: `db1aaea`.

## Implementation

- `localization/canonicalizeSemanticCloud.m` (moved from the study folder,
  rewritten so that only merged groups are recomputed): greedy grouping of
  same-class point components (`pole`, `trafficSign`) closer than a radius,
  in descending weight order; each group becomes one Gaussian by moment
  matching (weighted mean, within plus between scatter, summed weights).
  Weights are normalized by their maximum first, so a common scale of the
  stored prior cannot change the grouping. XYZ moments are kept only when
  every member carries height and are forced to marginalize exactly to the
  planar moments; the source `heightEvidence` sidecar follows the same rule.
  Line classes are never merged.
- `config/distributionRegistrationConfig.m`: required `cfg.pyramid` group,
  `mapMergeRadius = 1.5 m`, `sourceMergeRadius = 0.5 m`, `trustRadius = 0.15 m`,
  `pointClasses = ["pole","trafficSign"]`.
- `localization/registerSemanticProbabilityCloud.m`: one path. The solver
  first runs on the canonical clouds without aid; its pose (or the caller's
  prediction if the coarse solve is not accepted) seeds the solve on the
  original clouds, where position aid and additional seeds act as before.
  When the fine level had no evidence beyond planar geometry, no position aid
  and no active relative-height association, a refinement that moves more
  than the trust radius from the coarse pose is replaced by the coarse
  solution, reported with `reason = "coarseRetainedByTrustRadius"` and its
  correspondences translated to original component indices. Every result
  carries `pyramid` diagnostics (coarse pose, acceptance, similarity,
  component count, fine seed, refinement shift, retention flag, radii). A
  configuration without the group is rejected
  (`VehicleLocalization:MissingPyramidConfiguration`).

Tests: `tests/canonicalPyramidTest.m` (moment preservation, identity at zero
radius, weight-scale invariance, XYZ marginal consistency, height-evidence
sidecar, recovery of the true basin of a split landmark that traps a
single-level solve, trust-radius retention and its lift by position aid,
rejected configuration). Three existing tests assumed that a planar solve
stays on the alias its seed favours; they now assert the contract's
behaviour: the repeated-structure fixture of `positionAidedRegistrationTest`
repeats at 2 m (beyond the merge radius) with the seed near the wrong copy,
and `relativeHeightAssociationTest` checks that planar-only registration
reports the canonical midpoint of two aliases while height association
selects the correct one. 266 of 267 tests pass in 20 observer and
localization suites (`tests.csv`); the incomplete test is the YALMIP-gated
synthesis check, filtered as before. Code Analyzer: no findings in the seven
changed MATLAB files.

## Production replay

`runMncavCoarseLocalizationExperiment('output/mncav_coarse_localization_20260924b')`,
fresh coarse perception on all 1,170 scans, recursive matching, source
regeneration and the seven observer scenarios with GNSS-aided rematching.
Compared with the previous production chain
(`comparePyramidProduction.m`, `scenario_comparison.csv`,
`matching_stage_comparison.csv`; errors against INSPVA, RMSE after the
common 2 s initialization transient):

| Scenario | Before (cm) | After (cm) | Before P95 | After P95 | Before max | After max | Before >30 cm | After >30 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| GNSS + LiDAR | 5.53 | **5.42** | 9.83 | 9.90 | 17.47 | 17.45 | 0 | 0 |
| LiDAR only | 18.39 | **14.24** | 37.33 | 26.70 | 70.56 | 38.10 | 101 | 39 |
| GNSS only | 5.78 | 5.78 | 11.08 | 11.08 | 14.87 | 14.87 | 0 | 0 |
| GNSS outage 40–60 s | 6.31 | 6.17 | 12.86 | 12.38 | 19.68 | 18.71 | 0 | 0 |
| LiDAR outage 40–60 s | 5.61 | 5.51 | 9.83 | 9.91 | 17.47 | 17.45 | 0 | 0 |
| Both outage 40–60 s | 23.79 | 28.85 | 56.63 | 67.43 | 103.07 | 128.23 | 159 | 164 |
| Alternating 1 s | 16.57 | 15.40 | 31.08 | 26.80 | 41.70 | 40.45 | 74 | 26 |

Full-run fused RMSE including the transient: 6.05 → 5.96 cm. Heading RMSE is
unchanged (0.39 deg fused, 0.42 deg LiDAR-only).

The both-outage scenario contains no LiDAR between 40 and 60 s, and the
matches after 60 s are identical in the two chains frame by frame (22.4,
12.4, 12.7 cm ...). Its difference is the dead-reckoning drift during the
outage: 99 cm versus 125 cm at 59.9 s, from a learned lateral velocity bias of
0.2603 versus 0.2730 m/s at the outage start (LiDAR matching in the preceding
10 s was 11.9 versus 10.1 cm RMSE). A 20 s coast multiplies that 1.3 cm/s
difference into 25 cm; this measures the sensitivity of the bias learner at
one instant, not the matcher.

Recursive LiDAR-only matching stage (`matching/report.mat`):

| | Accepted | RMSE | P95 | max | >30 cm | >50 cm | frame 851 | total ms/scan (median) | registration ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | 1169 | 16.84 | 34.72 | 68.48 | 77 | 14 | 68.5 | 66.9 | 8.8 |
| after | 1169 | **12.65** | 22.97 | 39.34 | 18 | 0 | **4.9** | 75.8 | 16.9 |

The stage result equals the frozen-seed, reference-seeded and closed-loop
values of the study (12.65 cm): the answer no longer depends on the seed.
Inside the fused run, matching RMSE is 11.22 cm (before 11.25) at 21.9 ms
median per call including GNSS hypothesis selection (before 11.2 ms); the
coarse solution was retained in no fused-run frame. Per scan, the whole
online chain moves from 66.9 to 75.8 ms median, within the 100 ms frame.

## Limits

- Radii (1.5 / 0.5 / 0.15 m) were chosen from this map's alias structure and
  this drive's refinement-shift distribution; one drive, same-drive INSPVA
  map.
- The canonical level lowers the per-frame precision ceiling from 10.9 to
  13.0 cm; the gated refinement recovers part of it (12.65 cm).
- The map builder still publishes tile duplicates and split sub-components;
  canonicalization at publication remains open.
- Registration time roughly doubles (8.8 → 16.9 ms median in the recursive
  stage).

## Reproduction

```matlab
addpath(pwd); setupVehicleLocalization;
runtests({'tests/canonicalPyramidTest.m','tests/positionAidedRegistrationTest.m','tests/relativeHeightAssociationTest.m'});
runMncavCoarseLocalizationExperiment('output/mncav_coarse_localization_20260924b');
addpath('research/canonical_pyramid_20260924'); comparePyramidProduction;
```

MATLAB R2026a; deterministic.
