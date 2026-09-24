# Lateral velocity at the output point: production adoption

Date: 2026-09-24. Adopts the lever-arm finding of
[the LiDAR-only study](../lidar_only_20260923/README.md) into the production
chain. Code revision before the change: `6ed951f`.

## Reference-point audit

Every runtime signal was assigned to the body point it actually describes:

| Signal | Point | How it reaches the output point |
|---|---|---|
| Pose state `[X, Y, psi]`, map, LiDAR match | INSPVA output point | definition; LiDAR stored axes are translated by `mississippiLidarFrameCalibration.json` |
| BESTPOS position | antenna | `mncavBestposOutputPoint.json` (1.72 m ahead, 0.26 m left), already applied |
| Four-wheel longitudinal speed | centreline, any longitudinal position | identical along the centreline for planar rigid motion |
| Gyro yaw rate | any point | identical |
| Lateral-observer lateral velocity | IMU location (its master state integrates the IMU) | **was applied unchanged at the output point: the defect fixed here** |
| IMU accelerations | IMU location | still applied unchanged (see limits) |

Regressing the lateral-velocity error on the measured yaw rate on the
separate 12-11-24 drive gives an effective forward offset of **2.360 m** between
the lateral observer's point and the INSPVA output point (seconds 1–40; vy
RMSE 0.247 → 0.054 m/s; held-out remainder 0.095 → 0.059 m/s). The evaluation
drive, held out entirely, fits 2.32 m independently. The IMU lateral
acceleration shows the same lever arm against `rDot` (2.56 m on 12-11-24,
2.15 m on the evaluation drive), which supports the reading that the IMU sits
about 2.3 m ahead of the INSPVA output point. The number remains an empirical
effective offset: it also absorbs any yaw-rate-proportional error of the
bicycle model, whose cornering stiffnesses are unidentified priors, and no
installation survey exists.

## Implementation

Single path, no alternate branch:

- `lateralObserverConfig` gains a required `outputPoint` group.
  The reference profile exports at the observer point (`forwardOffsetM = 0`).
  The MnCAV profile loads `config/mncavMotionOutputPoint.json`, written by
  `scripts/calibrateMncavMotionOutputPoint.m` from the 12-11-24 drive only.
- `runLateralVelocityObserver` rejects a configuration without the group
  (`VehicleLocalization:MissingOutputPointConfiguration`). It exports
  `lateralVelocity = observerPointLateralVelocity - forwardOffsetM * yawRate`
  and forms the side-slip interface command from that output-point velocity,
  so `sideSlipAngle` and `sideSlipAngleRate` are consistent with it. The
  certified master and hidden states are untouched.
- The production entry points (`runMncavCoarseLocalizationExperiment`,
  `runMncavFullObserverExperiment`, `benchmarkLocalizationPipeline`,
  `runMncavWheelSpeedExperiment`, `runMississippiMapMatchingExperiment`) run
  the stored MnCAV gains with the current `lateralObserverConfig("mncav")`
  rather than the configuration frozen inside the saved design. The saved and
  current configurations were verified identical apart from the function
  handle that `isequaln` cannot compare. `tests/reference/lateralObserverDesign.mat`
  received the zero-offset group; its gains are byte-identical.

Consumers change nothing: the global observer and the source-window odometry
read `lateral.lateralVelocity` and `sideSlipAngleRate` as before and now
receive output-point values. Observer gains, the bias learner, GNSS aiding
and the matcher are unchanged.

## Production replay

`runMncavCoarseLocalizationExperiment('output/mncav_coarse_localization_20260924')`
reran the whole chain: fresh whole-pillar coarse perception on all 1,170 scans,
recursive LiDAR-only matching, source-window regeneration and the seven
observer scenarios with closed-loop GNSS-aided rematching. Errors are against
INSPVA over 1,169 frames; "after 2 s" excludes the common 64 cm
initialization transient (`compareOutputPointTransport.m`,
`scenario_comparison.csv`).

| Scenario | Before RMSE (cm) | After RMSE (cm) | Before after 2 s | After after 2 s | Before P95 | After P95 | Before max | After max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| GNSS + LiDAR | 8.22 | **6.05** | 7.88 | 5.53 | 16.49 | 9.83 | 23.58 | 17.47 |
| GNSS only | 9.19 | 6.33 | 8.87 | 5.78 | 19.16 | 11.08 | 26.02 | 14.87 |
| LiDAR only | 24.30 | **18.78** | 24.09 | 18.39 | 51.89 | 37.33 | 75.93 | 70.56 |
| GNSS outage 40–60 s | 8.72 | 6.76 | 8.41 | 6.31 | 16.69 | 12.86 | 23.58 | 19.68 |
| LiDAR outage 40–60 s | 8.26 | 6.12 | 7.93 | 5.61 | 16.49 | 9.83 | 23.58 | 17.47 |
| Both outage 40–60 s | 39.52 | 23.72 | 39.80 | 23.79 | 112.98 | 56.63 | 137.96 | 103.07 |
| Alternating 1 s | 19.26 | 16.63 | 19.25 | 16.57 | 35.01 | 31.08 | 57.54 | 41.70 |

Heading RMSE is unchanged (0.39 deg fused, 0.51 deg LiDAR-only, 1.33 deg GNSS
only). The share of fused outputs within 10 cm rises from 82.0% to 94.8%.

Supporting measurements (`matching_and_odometry.csv`):

- Lateral velocity RMSE against the reference over the evaluation drive:
  0.313 → 0.257 m/s. The remainder is mostly the drive-specific constant
  offset of about −0.25 m/s that the online bias learner handles inside the
  observer.
- Half-second source-window odometry drift: 15.2 → 13.4 cm RMS.
- LiDAR matching inside the fused run: 1,167 → 1,168 accepted frames,
  RMSE 11.51 → 11.25 cm, maximum 33.35 → 36.79 cm (one frame above 30 cm in
  both).
- The recursive LiDAR-only matching stage, seeded by one-frame odometry,
  changes little (accepted RMSE 17.19 → 16.84 cm, P95 36.1 → 34.8 cm) because
  the lever-arm error over 0.1 s is at most about 7 cm. The gain comes from
  the observer's velocity state, which no longer lags in turns.

The LiDAR-only result is 18.8 cm, above the 16.5 cm reached in the study with
additional gain and height settings. Those settings were not adopted here.

## Validation

223 of 224 tests passed in 17 observer and localization suites
(`tests.csv`); the one incomplete test is the YALMIP-gated synthesis check,
filtered as before. New tests cover the rigid-body transport, the zero-offset
identity, the rejected obsolete configuration and the MnCAV calibration
loading. Code Analyzer reported no findings in the ten changed MATLAB files.
MATLAB R2026a.

## Limits

- The IMU accelerations still enter the global observer at the IMU location.
  Their lever-arm terms, `rDot*d` in `ay` (up to about 0.6 m/s² in transients)
  and `r²*d` in `ax` (up to about 0.2 m/s²), are not compensated because a
  clean yaw acceleration is unavailable without differentiating the gyro.
- The crawl-mode kinematic constraint inside the lateral observer is written
  at the bicycle-model CG; it acts only below 6 m/s.
- The constant lateral offset of the evaluation drive is not calibrated and is
  left to the online bias learner.
- One evaluation drive and one calibration drive; same-drive INSPVA map.

## Reproduction

```matlab
addpath(pwd); setupVehicleLocalization;
calibrateMncavMotionOutputPoint;                                        % config/mncavMotionOutputPoint.json
runMncavCoarseLocalizationExperiment('output/mncav_coarse_localization_20260924');
addpath('research/output_point_transport_20260924'); compareOutputPointTransport;
```

The "before" side of the comparison is the 2026-09-23 production evaluation
under `output/localization_evaluation_20260923`.
