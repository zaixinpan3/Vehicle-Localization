# Four-wheel-only localization input

Date: September 16, 2026.

The current localization replay now derives longitudinal velocity exclusively
from the four recorded wheel rates. The user's source requirement supersedes
the previous experiment's decision to retain the reported vehicle speed on
accuracy grounds. There is no vehicle-speed/Twist acquisition or fallback in
the current motion preparation, recursive matching, calibration or complete
observer experiment. Earlier outputs and reports remain historical evidence.

## Implemented source contract

`scripts/prepareWheelMotionInputs.m` is the common adapter. It reads native
`wheel_speed_report.csv` packets with `front_left`, `front_right`, `rear_left`
and `rear_right` in rad/s, plus steering and corrected IMU. The existing
`estimateWheelLongitudinalSpeed` applies per-wheel conversion, turning
compensation, consistency rejection and acceleration prediction. Its effective
radii, acceleration coefficients and 0.01 s correction time constant are
unchanged from the separate-drive calibration. This recorded forward-driving
adapter clamps negative estimated speed to zero; the underlying estimator
continues to support signed speed.

The native wheel estimator is causal in its wheel packets. Upstream IMU and
steering reconstruction is still offline linear interpolation. Missing wheels
raise `VehicleLocalization:MissingWheelSpeed`; invalid or expired wheel aiding
raises `VehicleLocalization:WheelSpeedCoverage`. The adapter does not shorten
the run when wheel packets end early. A declared stationary initial speed
covers only the first three samples, before the first packet at 0.020248 s;
these remain explicitly invalid for wheel aiding. All later samples have recent
valid wheels. The allowed wheel age is 0.15 s.

`extractVehicleReplaySensors.py` now exports all four wheel rates and omits
`/vehicle/twist` and the steering report's vehicle-speed field. Fresh exports
contain 5,845 evaluation packets and 3,894 calibration packets, with timestamps
and four angular-rate values exactly matching the prior raw exports. Original
recordings and historical exports were not deleted. Existing replay sensor
folders received labeled copies of the wheel file for callers that still use
those folder paths; their old speed files are unused by the current adapter.

`prepareMncavObserverReplay` supplies the same wheel-derived Vx to the lateral
and global observers. `replayMississippiLocalization` also uses that motion
and the actual lateral observer by default; externally supplied motion must
declare `longitudinalVelocitySource="four_wheel"`. Recursive experiment reuse
checks reject matching results without that source provenance. A two-frame
functional replay exposed an existing metadata assumption about an ODOM-only
pose-table column. The matcher now reports either the current interpolated
INSPVA source or the legacy nearest-ODOM source correctly. Zero time offset for
the former describes interpolation to the scan timestamp, not zero reference
uncertainty.

The complete observer initializes directly from the first accepted LiDAR pose,
wheel/lateral velocity and rotated IMU acceleration. It no longer loads an old
vehicle-speed experiment's initial state or trajectory. On this recording that
state is numerically identical to the previous shared initialization. Both
GNSS XY and LiDAR pose injection remain enabled; gains and vehicle parameters
are unchanged. The historical interface audit moved under
`research/mncav_interface_audit_20260916/audit_historical_interfaces.py`; its
fixed-input steering comparison explicitly reads its frozen historical state.
That audit is not a current speed-input route.

## Executed results

The full replay evaluates all 11,690 samples on the original 100 Hz time grid,
using the existing precomputed zero-delay LiDAR poses and GNSS/INS XY packets.
The primary both-source result is:

| Position metric | Current four-wheel-only input |
| --- | ---: |
| Whole-trajectory RMSE | 11.8711 cm |
| Whole-trajectory median | 6.9269 cm |
| Whole-trajectory P95 | 23.7246 cm |
| Whole-trajectory maximum | 45.4715 cm |
| RMSE on the same 1,083 accepted LiDAR frames | 9.9304 cm |

Vx discrepancy against evaluation reference velocity is 0.0747469 m/s RMSE,
with +0.0545028 m/s mean error. Reference velocity is used for separate-drive
calibration and evaluation only; it is not a runtime velocity input. The
previous vehicle-speed result was 11.7403 cm whole-trajectory RMSE. This source
migration satisfies the required architecture; it does not demonstrate an
accuracy improvement over that historical result.

| Injection scenario | Whole-trajectory position RMSE (cm) |
| --- | ---: |
| GNSS and LiDAR | 11.8711 |
| LiDAR only, with motion observer | 23.5445 |
| GNSS only, with motion observer | 30.2563 |
| GNSS missing during 40–60 s | 11.9622 |
| LiDAR missing during 40–60 s | 12.6683 |
| Both missing during 40–60 s | 73.1758 |
| Alternating sources | 21.8293 |

The source-only ablations share the same initial LiDAR/motion state; the GNSS-only
case is not a GNSS-only cold start. LiDAR-only in this table means the observer
with GNSS disabled, not raw map matching. The same-frame metric and the uniform
whole-trajectory metric use different timestamp populations and must not be
interchanged.

## Validation and limits

- All 126 MATLAB tests pass, including 13 wheel-estimator tests and eight new
  source-adapter tests. The new cases verify missing-wheel rejection, no
  fallback despite an available old speed file, immunity to poisoned old
  speed fields, wheel-to-output dependence, missing columns, and initial/expired
  coverage behavior.
- Repeated full replay is identical. Changing future GNSS/LiDAR packets after
  60 s leaves the earlier state prefix identical. Halving the integration step
  changes position by at most 1.69524e-7 m. These checks do not establish causal
  upstream IMU/steering preprocessing.
- The recorded analyzer scope has zero findings; additional changed MATLAB
  scripts also passed static checks. Python compilation and Git whitespace
  checks pass.
- Independent Python/HDF5 verification recomputes all 21 scenario/population
  metric rows within 4.67e-15, proves production Vx is exactly the saved
  wheel-derived signal after the forward clamp, and checks wheel provenance.
  The optional selection runner and production replay agree within 6.34e-8
  in pose components; their Python/MATLAB clock bridges differ by roundoff.
  Within the native-frame runner, adding output timestamps preserves the
  uniform-grid states bit-for-bit.
- The recursive matching smoke test accepts both selected frames using the
  wheel-only default and INSPVA map. This is a two-frame functional check;
  the full observer experiment uses frozen matching measurements and does not
  represent a newly rematched full sequence.

The existing reference-assisted map and per-frame matching seeds, shared
GNSS/INS receiver reference, unresolved physical output-point transform,
common-mode wheel scale/slip limitations and offline upstream preprocessing
remain. Positive sampled translation margins and the observed course-rate
envelope do not verify all theorem hypotheses. No new unconditional ISS proof,
independent absolute-accuracy claim or fully online end-to-end claim is made.

## Reproduction and artifacts

From the repository root, with the original local recordings and existing map,
matching and gain artifacts available:

```bash
uv run --offline --with rosbags python scripts/extractVehicleReplaySensors.py \
  --output-dir output/mncav_wheel_only_20260916/sensors
uv run --offline --with rosbags python scripts/extractVehicleReplaySensors.py \
  --bag data/raw/Missisipi/raw_data_2024-06-07-12-11-24_0.bag \
  --output-dir output/mncav_wheel_only_20260916/calibration_sensors
uv run --offline --with numpy --with pandas --with scipy \
  python scripts/calibrateMncavWheelSpeed.py
```

```matlab
setupVehicleLocalization;
report = runMncavFullObserverExperiment;
validation = validateFullLocalizationObserver;
wheelReport = runMncavWheelSpeedExperiment;
smoke = replayMississippiLocalization( ...
    'output/mississippi_mapping_inspva_20260915/probability_cloud_map.mat', ...
    'output/mncav_wheel_only_20260916/sensors', ...
    'output/mncav_wheel_only_20260916/matching_smoke', ...
    'recursive', [1,2], FrameBlockSize=2);
```

```bash
uv run --offline --with numpy --with pandas --with h5py \
  python research/mncav_wheel_only_20260916/verify_results.py
```

The report directory contains compact metric, validation and provenance
exports. Native CSV data, MAT state histories, plots and the smoke replay stay
under `output/mncav_wheel_only_20260916/`, with hashes in
`artifact_manifest.json`. They are excluded from public Git along with raw
recordings and generated binaries. Signed technical records and labeled source
and output copies are archived separately after the project commit and push.
