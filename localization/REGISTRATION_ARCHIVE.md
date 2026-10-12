# Registration localization branch: archived on 2026-10-12

The current localization pipeline uses `overlapGradient` exclusively. The
user retired the alternative route that optimizes a LiDAR pose and injects
the resulting registration measurement into the seven-state observer.
This is a scope decision, not a claim that the remaining channel has no
failure cases or that the mathematical registration model is invalid.

## Current entry points

- `runMncavFullObserverExperiment`: overlap-only scenarios, with fresh coarse
  horizons built directly from scans and measured motion. No registration
  replay or optimized LiDAR pose is required. The default output is
  `output/mncav_overlap_localization/observer`.
- `replayDowntownGnssFreeLocalization`: overlap-only tracking.
- `runMncavVdbLocalization`: overlap-only tracking, plus GNSS-only and
  dead-reckoning ablations. GNSS bootstrap remains declared.
- `activeLocalizationChannel` rejects every other channel before replay
  inputs are opened. `runMncavCoarseLocalizationExperiment` is a retired
  entry and reports `VehicleLocalization:ArchivedRegistration`. The older
  `runMncavFullLocalizationExperiment`, `runMncavZeroDelayExperiment` and
  `runCarlaLocalization` dataset entries likewise refuse execution.

`MatchingFolder` on the Mississippi entry is only an optional historical
acquisition/motion-cache reader. Its saved matching poses and acceptance
flags do not supply current observer corrections. New runs need no such folder.
`RematchWithGnss` is removed; current GNSS availability changes the observer's
GNSS correction, and never selects a registration hypothesis.

## Historical implementation and recovery

The complete pre-retirement implementation is recoverable at Git commit
`2d6dcf52e94c391a9538e9c9be59751d85c08cae`, including
`scripts/runMncavCoarseLocalizationExperiment.m`,
`scripts/runMncavFullObserverExperiment.m`,
`scripts/replayDowntownGnssFreeLocalization.m`, and
`scripts/carla/runMncavVdbLocalization.m`. Inspect with `git show` or use an
isolated worktree at that commit for an explicitly requested historical run.
The CARLA GNSS-mask extension was an existing uncommitted change at retirement
and is not part of that historical commit.

Low-level `localizeLidarFrame`, `registerSemanticProbabilityCloud`,
`replayMississippiLocalization`, pose-graph helpers and registration tests
remain as frozen standalone diagnostics and compatibility checks. They are
outside the current production localization workflow; no new accuracy study
should select them as a live alternative. Existing research comparisons and
scores retain their original channel labels and dates.

`registrationSupport`, `prepareSemanticRegistrationGeometry` and the geometry
configuration remain shared dependencies of overlap, source-window construction,
mapping and calibration. Their names do not mean the active pipeline optimizes
a pose. The observer's generic recorded-pose/residual interfaces are retained
for synthetic validation and historical analysis, while all current dataset
entry points supply only `data.lidarOverlap` for LiDAR tracking.

The MnCAV vehicle parameters, theta=4 design, scale ladder, measurement
weights, reference definitions and initialization conventions are unchanged.
Archiving the alternative does not resolve the overlap excursions previously
observed on CARLA outer_lane2.
