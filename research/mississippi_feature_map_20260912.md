# Mississippi map from saved perception observations

On 2026-09-12, the existing mapping pipeline processed all 1,170 frames from
`output/mississippi_perception_video_20260912/feature_observations.mat`.
Perception was not rerun. Mapping implementation and parameters were unchanged
from commit `ec718e9a166808c73d989d31925be7d59f968cd8`.

## Method

The saved `featureData` contains GNSS/INS-registered global XYZ observations,
frame identifiers, calibration, and pose provenance. These coordinates were
passed directly to `buildSlidingWindowMap`; no second pose transformation was
applied. The frame identifiers were checked against 1:1170 and the feature names
and observation totals were asserted before construction.

The run used `featureMapBuildConfig`, the recorded calibration, and all saved
feature classes. The existing 30-frame windows advance by 20 frames; overlapping
observations are ingested once into canonical spatial tiles. The repeated-
observation model uses one observation block per frame, 0.10 m XY representatives,
0.25 m reference cells, 8 m tiles, a 2 m context halo, and up to three components
per tile. Publication requires at least two effective observed blocks and the
existing repeatability decision (posterior greater than 0.5 under equal costs),
together with the existing support and mass checks.

## Results

Construction took 183.858663 seconds, excluding file export, tests, and plotting.
The input contained 399,086 feature-point observations, which are repeated
observations rather than distinct physical objects.

| Class | Input observations | Representatives | Tiles | Candidate components | Published components |
| --- | ---: | ---: | ---: | ---: | ---: |
| curb | 133,836 | 128,239 | 383 | 1,072 | 884 |
| pole | 194,300 | 23,168 | 204 | 404 | 256 |
| trafficSign | 70,950 | 23,475 | 111 | 257 | 191 |

The exported map contains 1,331 published Gaussian components. These counts are
mixture components, not counts of individual curbs, poles, or signs. Total
published reference mass is 1339.8149361016444.

The final fits reached the configured 80-iteration limit in 138/383 curb tiles,
11/204 pole tiles, and 15/111 traffic-sign tiles. These fits are retained by the
current implementation; this run does not claim that every fit converged.
Model-selection fits also include iteration-limited cases, recorded separately
in `validation.json`. No convergence thresholds were changed for this run.

## Validation and limitations

- `tests/temporalStabilityGmmMapTest.m`: 30 passed, 0 failed, 0 incomplete.
- `mappingSupport.validateRepeatedObservationMap` passed the schema, covariance,
  ownership, support, and mass checks.
- At 100 deterministic published-component locations (170 valid class queries),
  full Gaussian reconstruction from `temporalMapToProbabilityCloud` agreed with
  `queryTemporalStabilityGmmMap` within its declared approximation error bounds.
  Maximum intensity error was 3.10081682368323e-10; maximum valid score error was
  1.9957221370546797e-9.
- The exported overview was visually inspected. It compares registered input
  observations with published 2-sigma XY Gaussian footprints against the same
  GNSS/INS trajectory. The 0.25 m display reduction affects only the left plot.

These checks establish construction and export consistency, not independent
semantic or localization accuracy. The current stability model assesses repeated
geometric consistency; it does not model visibility or count every missing
detection as negative evidence. Persistent, spatially consistent false detections
can survive publication. Pose errors in the recorded registration also affect
the map. No registration refinement or localization evaluation was run here.

## Local artifacts and reproduction

All generated files are under `output/mississippi_mapping_20260912/`:

- `probability_cloud_map.mat`: full map, variable `probabilityCloudMap`.
- `probability_cloud.mat`: compact published Gaussian export, variable `cloud`.
- `mapping_configuration.mat`: exact configuration and input path.
- `map_overview.png` and `map_overview.fig`: observation/map overview.
- `validation.json`, `map_layer_summary.csv`, `tests.json`, and `tests.mat`.
- `mapping_manifest.json`: input/output byte sizes and SHA-256 hashes.
- `build_from_saved_observations.m`, `plot_map.m`, and `build_log.txt`.

Executed construction command from the repository root:

```bash
matlab -batch "run('output/mississippi_mapping_20260912/build_from_saved_observations.m')" -logfile output/mississippi_mapping_20260912/build_log.txt
```

The full map SHA-256 is
`74abe950f618abef4e0991181743b0ba74b1d689a15185db2f0c63c52b8ab104`.
The source observation SHA-256 is
`df99b125366cc2d6573d28b3b5cd97f3d0bae43d866fd0f8ce565e916e4a903b`.
Generated binaries and recorded datasets remain local; the source commit records
this experimental report. Export copies of the run scripts, manifest, metrics,
and overview support the corresponding internal activity record.
