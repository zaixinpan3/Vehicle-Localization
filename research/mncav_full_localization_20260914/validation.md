# Complete MnCAV recorded localization experiment

Date: 2026-09-14. Sequence: `raw_data_2024-06-07-12-09-31_0`, all 1,170
front LiDAR scans. The experiment now executes the actual lateral observer,
fresh map matching, and the continuous global observer. It is a completed
**offline cascade experiment**, not a successful real-time validation or an
independent-map accuracy benchmark. Position accuracy did not improve over
matching alone at common scan times; heading accuracy improved.

## Inputs and execution order

1. Use `lateralObserverConfig("mncav")` and solve
   `designLateralObserverGains`. The nominal MnCAV model uses the central
   stock Pacifica Hybrid parameters and explicitly unidentified inertia/tire
   priors. All 18 speed/acceleration design-grid points pass the existing
   recovered-gain checks. The speed grid is nine points from 5 to 30 m/s;
   acceleration endpoints are -3 and 3 m/s^2. This certifies the design-grid
   conditions, not interpolation or the complete hybrid runtime.
2. Prepare recorded speed, steering and corrected acceleration/gyro inputs
   with `prepareMncavObserverReplay`. Constant sensor-axis/offset corrections
   come from the prior adjacent-drive calibration export. Verify its vehicle
   values equal the central nominal model, then run the actual hybrid lateral
   observer over the entire 100 Hz sequence, including low-speed/stationary
   branches. No reference lateral velocity or sideslip is supplied.
3. Solve `designImprovedObserverGains` for the new `mncav` profile. Initial
   synthesis used the previous information-shaping scale 5. Once real
   matching information was inspected, the final experiment explicitly
   selects scale 0.001 and re-synthesizes. It retains theta=2, |q|<=0.4 rad/s,
   150 ms assumed delay, rate .05, and the existing information-weight sector.
   The four-vertex continuous matrix check passes with margin .0860993.
4. Process all raw scans with fresh coarse perception and recursive D2D
   matching against the frozen 2026-09-12 map. Prediction uses recorded speed,
   corrected gyro, actual estimated lateral velocity, and the last accepted
   match. Rejected matches retain that motion prediction. One initial
   reference pose is biased by [.5 m,-.4 m,2 degrees]; later reference XY/yaw
   are used only for scoring, not recursive prediction. Recorded roll/pitch
   remain known tilt inputs. Global estimates do not feed back into matching.
5. Reconstruct accepted pose/information samples offline and run the global
   observer with the **actual lateral observer's saved outputs** aligned to
   its input interval. Passing `LateralInputs` avoids resetting the already
   executed lateral estimator; it is not an exact-lateral bypass. The global
   initialization propagates the same initial seed using motion alone to
   t=.15 s, then declares constant estimated prehistory.

The raw sensor stream uses the existing receiver/ROS time bridge. Receiver
elapsed duration is 116.89954 s versus ROS duration 107.15884 s. The lateral
input grid ends at 116.89 s. Matching holds its last supplied motion for the
remaining 9.543 ms, explicitly reported, with a hard 20 ms edge allowance.

## Map and calibration provenance

The frozen source is
`output/mississippi_mapping_20260912/probability_cloud_map.mat`, containing
1,331 published Gaussian components (884 curb, 256 pole, 191 traffic sign).
Its documented SHA-256 is
`74abe950f618abef4e0991181743b0ba74b1d689a15185db2f0c63c52b8ab104`.
The mapping configuration's identity rotation/zero translation matches the
online calibration. No new map is built and no second global transformation
is applied. Full frame processing uses blocks of 50 and eight MATLAB threads.

Mapping and queries use the **same drive, including the query observations**.
The GNSS/INS pose also constructed the map. Results therefore measure within-
sequence consistency and implementation behavior, not independent absolute
accuracy or GNSS/INS-device-loss performance. Mapping provenance is detailed
in `research/mississippi_feature_map_20260912.md`.

## Gains and actual operating conditions

`gains.json` stores all eighteen 2-by-2 lateral gains, speed/acceleration grids,
and global K/N/P/Q/R/g and verification. At 8 m/s and zero longitudinal
acceleration, the interpolated lateral gain is approximately

    L = [-0.5890, -0.0068;
          0.0847,  0.0019].

The global normalized K chains are [3;3;1.5] and its yaw coefficient is 1.
With theta=2 the actual pose-injection matrix T*K is

    [6  0  0;
     12 0  0;
     12 0  0;
     0  6  0;
     0 12  0;
     0 12  0;
     0  0  2].

The complete physical auxiliary gain is preserved in `audit.json`.
Actual lateral estimation reaches maximum |vy|=.4765 m/s. The hidden dynamic
model is evaluated at 10,787 of 11,690 samples; the stationary detector is
active at 70 samples. Other samples use the existing hybrid logic. Recorded
corrected longitudinal acceleration spans about -1.2513 to 1.5022 m/s^2;
no evaluated dynamic sample leaves the acceleration design grid. The
maximum supplied q=r+betaDot is .3549753 rad/s, and global integration-stage
checks report no course-rate excursion. Estimated velocity/acceleration
component maxima are 15.4903 m/s and 2.9857 m/s^2. Estimated-state checks do
not establish the theorem's true-state or sensor-disturbance assumptions.

## Real matching information and reconstruction limits

Of 1,170 scans, 1,098 are accepted full poses (93.846%); 72 are rejected for
inconsistent classes; no directional-only pose is emitted. All predictions
on rejection remain in the all-frame matching metrics. The continuous input
uses only the 1,098 full-pose measurements and their **unchanged original
3-by-3 matcher information matrices**, including cross terms.

The minimum information eigenvalue is 1.2896876. With the historical gain-
shaping lambda=5, its minimum normalized weight would be about .205 and the
old .998520 lower sector is not satisfied. Lambda is a selectable observer
normalization parameter, not measured information. The final explicit
lambda=.001 yields minimum W eigenvalue .9992252, above the retained sector,
without multiplying, flooring or replacing a measured information matrix.
This makes injection nearly full strength, including weak accepted geometry;
it is a design choice informed by this sequence, not a calibrated noise
covariance, a parameter optimum, or held-out tuning. False accepted matches
can still have substantial influence.

The strict default recorded adapter fails with
`VehicleLocalization:ReconstructionGap`: the maximum accepted-pose interval
is .882857 s, exceeding .12 s. The first reconstructed measurement also fails
the historical lambda=5 information threshold. These mismatches are retained
in the audit and are not described as a successful unchanged-default run.

For this offline experiment, `MaximumOfflineGap=1` explicitly permits linear
interpolation of accepted matching poses and information across these gaps.
No rejected prediction, reference pose, or fabricated information is inserted
as a new LiDAR measurement. The longest gap is approximately
[82.69938,83.58224] receiver seconds. Even with zero processing time, 479 of
11,675 global samples require a future captured scan; maximum lookahead past
the nominal current time is .732240 s. The declared .15 s delay is therefore
**an offline DDE experiment setting, not an achieved causal latency**.
An online missing/delayed-measurement interface and its stability conditions
remain necessary for a real-time deployment claim. Merely relaxing the gap
check does not supply such a proof.

## Measured results

The global observer processes 11,675 samples on [.15,116.89] seconds with
maximum integration step 5 ms and no pose time alignment chosen to minimize
error. At the 1,167 scan timestamps common to both estimators, both are
scored against the exact same scan-associated reference:

| Common scan metric | Matching plus prediction on rejection | Full cascade |
|---|---:|---:|
| Position RMSE (m) | 0.2883 | 0.3589 |
| Heading RMSE (deg) | 0.6220 | 0.5547 |

Full 100 Hz observer position RMSE is .338912 m, median .196721 m,
P95 .605824 m, and maximum 2.170376 m. Heading RMSE is .551535 degrees and
maximum absolute error 2.091922 degrees. The 10 Hz trace is an export of the
full-resolution results, not the basis of the metrics. After the first five
seconds, position RMSE is .343454 m, so startup alone does not explain the
position deficit. A separate acausal interpolation of matching at current
100 Hz times has position/yaw RMSE .251214 m/.584391 degrees; it has a
different reference/time basis from raw scan metrics and is not an online
localization output.

All 1,170 matching/prediction frames have position RMSE .288295 m, P95
.570872 m, maximum 1.584695 m; yaw RMSE .621195 degrees. Accepted-only
position RMSE is .273854 m and is explicitly not the whole-sequence score.
The lateral-aided motion-only baseline has position RMSE 19.0496 m and final
error 30.4665 m. Map corrections are necessary on this drive.

The largest global position error occurs at 109.38 s. Around it, frames
1090--1096 are all accepted but have reference discrepancies around
1.46--1.58 m, followed by a rapid correction in frames 1097--1100. This
supports investigating accepted matching bias/jumps and observer transient
response. It does not uniquely isolate model residual, timing, and gain
contributions. A certificate that tolerates bounded disturbances does not
promise better RMSE than raw measurements. No such improvement is claimed.

## Timing, checks and artifacts

Fresh lateral synthesis takes 6.015 s and global synthesis .212 s in the final
orchestration run. Lateral and global execution take 1.920 s and 12.504 s,
respectively, excluding matching. Fresh map matching has computation median
87.241 ms, P95 100.126 ms, P99 112.707 ms and maximum 748.364 ms. All calls
are retained, including startup; 64 exceed 100 ms and four exceed 150 ms.
Disk I/O and offline preparation are separate. These are measured durations,
not a hard real-time or end-to-end deadline guarantee.

104 unique tests pass: lateral 15, MnCAV source 3, recorded motion 5,
continuous global runtime 46, MnCAV certificate 7, geometric registration 16,
and information 12. An initial geometric suite setup lacked the root path;
those 16 tests were rerun successfully after explicitly adding it. The initial
incomplete result is retained locally. Three changed/new scripts have zero
factory Code Analyzer findings. The final trajectory/error figure was
visually inspected. Actual matching and full-cascade execution additionally
exercise the new motion-input and orchestration paths on the recorded data.

All original outputs are local under
`output/mncav_full_localization_20260914/`: full MAT states/designs, 100 Hz
trajectory, lateral output, accepted/rejected calls, original information,
figures, gain export, summaries, tests and audits. Compact public results are
in this report directory; original bags, maps and generated binaries are not
committed. The saved `matching/report.mat` was generated freshly in this
experiment, then reused by the final orchestration run to avoid repeating
all 1,170 scans. Gains and lateral/global outputs were recomputed there.

From the repository root, with YALMIP and SeDuMi on the MATLAB path:

```matlab
addpath(pwd);
setupVehicleLocalization;
maxNumCompThreads(8);
% Fresh full run, including every map-matching call:
report = runMncavFullLocalizationExperiment;
audit = auditMncavFullLocalizationExperiment;
% Reproduce the final observer run using this experiment's saved matches:
report = runMncavFullLocalizationExperiment( ...
    "output/mncav_full_localization_20260914", ...
    MatchingFolder="output/mncav_full_localization_20260914/matching");
```

The completed result establishes that the implemented stages run together on
this recorded drive. It also exposes three remaining engineering issues:
causal handling of missing pose measurements, rejection/downweighting of
incorrect accepted matches, and gain/model choices that currently worsen
position RMSE. Full physical cascade stability, calibrated loaded-vehicle
parameters and independent-drive localization accuracy remain unestablished.
