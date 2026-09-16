# Four-wheel longitudinal velocity estimation for MnCAV

## Material Passport

Date: September 16, 2026. Status: VERIFIED by recorded replays, 51 passing
MATLAB tests and independent numerical checks. Scope: implement and evaluate
an explicit wheel-derived longitudinal-velocity input. The candidate works,
but does **not** improve the current evaluation drive; the established
localization runner and its default input are therefore retained.

## Recorded signals and the existing input

Both available Mississippi drives contain `/vehicle/wheel_speed_report`
with front-left, front-right, rear-left and rear-right angular velocities.
The original evaluation bag's embedded `dbw_fca_msgs/WheelSpeedReport`
definition specifies **rad/s**, not m/s. The existing read-only exports
contain 5,845 evaluation packets and 3,894 calibration packets, with no
nonfinite wheel values and no dropped timestamps in this replay export.
Receiver-clock median intervals are 0.019998 and 0.019942 s. The largest
evaluation interval is 0.118169 s; the observer explicitly permits 0.15 s of
inertial propagation before declaring wheel aiding expired.

The legacy `/vehicle/twist.linear.x` is the vehicle CAN speed forwarded
through `/vehicle/steering_report.speed`. All 11,693 evaluation records and
7,792 calibration records have identical stamps and exactly identical speed
values. The [Dataspeed upstream release source](https://github.com/DataspeedInc-release/dbw_fca_ros-release/blob/538a04413ec94fe531d1de3971409899202ebf0e/dbw_fca_can/src/DbwNode.cpp#L314-L334)
decodes `VEH_VEL` from the steering CAN report and assigns it to the Twist
longitudinal component. This establishes the software interface and agrees
with the recordings; it does not establish the proprietary ECU's internal
wheel-fusion algorithm or the exact driver build used during collection.
The existing input is not a GNSS-position-difference velocity.

Stock geometry comes from the [manufacturer specifications, pages 2--3](https://www.stellantisfleet.com/content/dam/fca-fleet/na/fleet/en_us/chrysler/2021/Pacifica/specifications/2021_CH_PacificaHybrid_Specifications.pdf):
wheelbase 3.089 m, front/rear tracks 1.734/1.735 m and steering ratio 16.2.
The existing independently fitted 1.5-degree steering-wheel zero is retained.
These are stock geometry and empirical corrections, not surveyed MnCAV
installation geometry or measured loaded tire radii.

## Implemented estimator

The implementation is `estimateWheelLongitudinalSpeed`, configured by
`wheelSpeedObserverConfig` and `mncavWheelSpeedCalibration.json`. Its runtime
inputs are four native wheel rates, steering angle, corrected IMU yaw rate
and longitudinal acceleration. No GNSS, INSPVA velocity, LiDAR pose or
legacy Twist speed enters this function.

For wheel i at lateral position y_i, rolling speed is approximately

    R_i omega_i = cos(delta_i) (Vx - r y_i)
                + sin(delta_i) (Vy + r x_i).

The two rear wheels have delta_i = 0, so their corrected longitudinal
candidates are R_i omega_i + r y_i. Their average cancels the ideal
left/right turning contribution. Front angles follow Ackermann geometry
from the measured steering angle and front track. With small lateral tire
slip, projection supplies each wheel candidate:

    v_i = R_i omega_i cos(delta_i) + r y_i + tau_i a_effective.

The front projection has residual sin(delta_i) times lateral velocity at the
wheel in wheel coordinates. It is an approximation when tires sideslip; this
is one reason rear-only and four-wheel variants are compared. For short
prediction intervals, a_effective = ax + lr r^2 uses the kinematic
approximation Vy = lr r. It is not an exact acceleration identity under
arbitrary sideslip. Existing global/lateral observer equations and gains
are unchanged.

The four candidates pass a 0.15 m/s median-consistency gate. Unavailable or
outlying wheels are excluded; at least two consistent wheels are needed.
Two remaining wheels differing by more than 0.30 m/s are not averaged.
Accepted wheels update an acceleration-predicted scalar velocity with an
exponential correction. Stationary wheel/gyro evidence suppresses drift.
The estimator reports source age, accepted count, rejected wheels and a
validity flag; it never labels indefinite propagation as valid wheel aiding.
It preserves signed reverse speed, although the existing forward-driving
localization experiment clips its input at zero.

This handles one inconsistent wheel. Four wheels with the same slip or scale
error can remain mutually consistent. In straight steady motion, four wheel
rates alone do not identify a common rolling-radius scale and absolute
speed separately. The function explicitly reports that common-mode slip is
not observable. A common-factor adaptation based on trusted GNSS/LiDAR could
be a subsequent design, but was not implemented, tested or claimed here.

## Calibration and selection

`calibrateMncavWheelSpeed.py` fits only the separate 12:11:24 drive, using
receiver times strictly between 1 and 40 s with reference speed above 3 m/s
(3,899 reconstructed samples). Bounded soft-L1 regression fits one radius
and one acceleration coefficient per wheel, with loss scale 0.025 m/s.
The effective radii in FL/FR/RL/RR order are
0.3617443/0.3617199/0.3621151/0.3619289 m; acceleration coefficients are
0.02325/0.01872/0.05058/0.05330 s. These coefficients absorb equivalent
input lag and model errors; they are not measured delivery delays.

Twelve variants compare robust four-wheel fusion, four-wheel mean and
rear-only fusion, each at time constants 0.01/0.04/0.08/0.15 s. Selection
uses only the calibration drive's 3,790 later samples (40 s onward).
Robust four-wheel fusion with 0.01 s is selected. This is the best of these
specified candidates on that selection interval, not a globally optimal
estimator. Calibration selection is development evidence, not an untouched
test set. Preliminary radius-only, affine and gyro-differential diagnostics
also examined the evaluation drive; it must not be described as a blinded
holdout. The committed coefficients and candidate selection nevertheless
use only the separate drive, and no evaluation-reference correction is
injected at runtime.

## Measured results and decision

| Velocity comparison | Legacy CAN speed RMSE (m/s) | Selected wheel fusion RMSE (m/s) |
|---|---:|---:|
| Calibration drive, later selection interval | 0.033251 | 0.024271 |
| Evaluation drive, all 11,690 output samples | **0.058186** | 0.074747 |

Calibration-selection RMSE decreases 27.0%, but its peak error grows from
0.1190 to 0.1714 m/s. The evaluation wheel estimate has mean error
+0.05450 m/s, versus +0.02983 m/s for legacy CAN speed. Squared mean error
accounts for 53.17% of the wheel estimator's evaluation MSE. The evaluation
peak does improve, from 0.28245 to 0.23337 m/s, but overall RMSE worsens.
This supports a transfer/calibration-bias concern, not a proven diagnosis
of tire pressure, true tire radius, wheel slip or installation error.

The selected velocity replaces only Vx in the frozen 12:09:31 full-observer
inputs. The lateral observer is recomputed, while original GNSS/LiDAR
packets, global gains, timestamps, map, initial pose and scoring references
are preserved. A declared zero initial speed bridges the first three output
samples before the first wheel packet at 0.020248 s; no future wheel packet
is copied backward. All later outputs have valid recent wheel aiding.

| Full localization metric | Existing CAN input | Wheel-derived candidate |
|---|---:|---:|
| Whole-trajectory position RMSE (cm) | **11.7403** | 11.8711 |
| Whole-trajectory median (cm) | **6.7395** | 6.9269 |
| Whole-trajectory P95 (cm) | **23.4138** | 23.7246 |
| Whole-trajectory maximum (cm) | 46.1107 | **45.4715** |
| Same 1,083 accepted LiDAR frames, RMSE (cm) | **9.7613** | 9.9304 |

The native-frame audit adds outputs at existing integration boundaries and
preserves every uniform state bit-for-bit. Raw map matching remains
14.8629 cm RMSE on those same accepted frames. No selective interval removal
is used to claim improvement. The new wheel candidate remains below raw
matching at accepted frames, but is slightly worse than the established
complete observer, so it is **not promoted to the default localization
input**. It is available through the reusable estimator and the executable
comparison runner.

## Reproduction and checks

```bash
uv run --offline --with numpy --with pandas --with scipy python scripts/calibrateMncavWheelSpeed.py
```

Executed through MATLAB MCP:

```matlab
setupVehicleLocalization;
report = runMncavWheelSpeedExperiment;
results = runtests({'tests/wheelLongitudinalSpeedTest.m', ...
    'tests/fullLocalizationObserverTest.m', 'tests/lateralObserverTest.m', ...
    'tests/mncavVehicleConfigTest.m'});
```

All **51 tests pass**, including 13 new physical/interface cases: straight
and accelerating motion, turning geometry, one slipping or unavailable
wheel, conflicting remaining wheels, missing packets and expiry,
stationary drift suppression, initial-state declaration, reverse motion,
future-input isolation and malformed timestamps. Four MATLAB files have
zero factory Code Analyzer findings. The first test run exposed two uses of
an unavailable `range` function; replacing them with base-MATLAB max-minus-min
resolved both. Its initial results are preserved in the operational output.
The tests do not demonstrate accurate estimation during common-mode slip.

```bash
uv run --offline --with numpy --with pandas --with h5py --with matplotlib python research/mncav_wheel_speed_20260916/verify_results.py
python -m py_compile scripts/calibrateMncavWheelSpeed.py research/mncav_wheel_speed_20260916/verify_results.py
```

The independent verifier checks stored states against exported velocity,
position and native-frame scores, unchanged source packets, calibration-only
selection, source identity and preserved original uniform outputs. Maximum
metric discrepancy is 5.55e-17. Its first source-packet equality check
incorrectly treated matching invalid NaNs as unequal; explicit equal-NaN
comparison fixes the verifier without changing measurements. The plotted
comparison is `output/mncav_wheel_speed_20260916/comparison.png`.

The wheel runtime is causal relative to supplied motion streams. Existing
motion preparation still uses offline linear reconstruction and INSPVA-based
timestamp bridging. The evaluation reference is planar INSPVA velocity;
physical frame/lever-arm and shared-receiver limitations persist. The map
and matching seeds remain reference assisted. No new ISS theorem,
independent absolute-accuracy evidence, fully online upstream preparation,
or universal best-Vx claim is made.
