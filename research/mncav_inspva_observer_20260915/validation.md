# MnCAV observer replay with the INSPVA-built map

Date: September 15, 2026. Status: executed recorded-data experiment and
implementation validation. The source drive is Mississippi
`raw_data_2024-06-07-12-09-31_0` (June 7, 2024). This report compares the frozen
outputs of the [INSPVA map rebuild](../inspva_mapping_rebuild_20260915/validation.md)
with the existing motion-aided seven-state observer and a new offline gap
reconstruction. All three reference-relative matching initializations are
retained; no matching calls or gain search are rerun.

## Result

Adding measured motion to the observer improves position RMSE at the same
accepted LiDAR timestamps. The initial continuous replay barely improves
full-trajectory RMSE because linear interpolation through a 3.10-second
measurement gap contributes a large, artificial trajectory error. Using
measured motion to reconstruct long gaps reduces this contribution while
preserving every accepted LiDAR pose and its geometric information exactly.

The primary zero-seed comparison uses all 1,084 full accepted measurements.
Both methods use the same timestamps and the same INSPVA frame reference.

| Metric | Original LiDAR measurements | Observer, linear gaps | Observer, motion-informed gaps |
|---|---:|---:|---:|
| Position RMSE (cm) | 14.8634 | 11.8210 | **11.7369** |
| Position median (cm) | **7.8867** | 9.6757 | 9.6345 |
| Position P95 (cm) | 33.0450 | 20.6526 | 20.6700 |
| Position maximum (cm) | 57.5065 | 46.3459 | 32.9050 |
| Heading RMSE (degrees) | 0.4244 | 0.3165 | 0.3140 |

The final accepted-frame position RMSE decreases by about 21.0%, but the
median increases. The fraction within 5 cm changes from 25.28% to 12.36%,
and within 10 cm from 61.62% to 53.51%. This is an improvement in RMSE and
larger errors, not improvement at every frame or a demonstrated few-centimeter
typical error. No accuracy threshold is guaranteed by the stability certificate.

Full-trajectory comparisons use the original 11,690 uniform 100 Hz samples,
covering 0--116.89 seconds, without overweighting inserted LiDAR knots:

| Continuous output | Position RMSE (cm) | Median (cm) | Maximum (cm) |
|---|---:|---:|---:|
| LiDAR poses with linear interpolation | 26.5834 | 8.0256 | 194.7598 |
| Observer using that linear input | 26.4000 | 9.9438 | 196.2178 |
| Motion-informed interpolation alone | 13.6733 | 8.0692 | 57.1469 |
| Observer using motion-informed interpolation | **12.2595** | 9.8788 | 33.4049 |

The 26.5834 cm continuous LiDAR baseline includes interpolation errors in
missing measurements. It is not the 14.8634 cm raw accepted-measurement
baseline. Motion-informed interpolation already uses vehicle motion; it must
not be described as pure LiDAR. The observer adds a further 10.3% RMSE
reduction relative to that motion-informed input on the same uniform grid.

The same fixed design is checked with both nonzero matching initializations:

| Per-frame seed | Full measurements | Raw LiDAR RMSE (cm) | Final observer RMSE at those frames (cm) | Final uniform RMSE (cm) |
|---|---:|---:|---:|---:|
| Zero reference offset | 1,084 | 14.8634 | 11.7369 | 12.2595 |
| +[0.5 m, -0.4 m, 2 degrees] | 1,066 | 17.7102 | 13.8795 | 14.8124 |
| Opposite offset | 1,069 | 20.7377 | 15.7732 | 16.4381 |

All modes retain their 1,170 evaluation frames. The zero mode has 4 directional
and 82 rejected matches; these 86 frames supply no new full-pose knot. Final
results for them and for all native frames are retained in
[motion_gap_metrics.csv](motion_gap_metrics.csv). The continuous observer
requires full geometric information; this experiment does not implement a new
directional-only injection. No rejected candidate is promoted into a measurement.

## Motion sources, vehicle and gains

The original 100 Hz speed, steering and corrected IMU signals come from
`output/mncav_zero_delay_20260914/experiment.mat`. Only the saved motion arrays,
their original uniform indices and lateral replay/configuration are loaded.
The legacy position/GNSS channels are not fed to this observer. The established
IMU sign/offset corrections were determined from the other recorded drive
(12:11:24); they are not refitted to the present reference.

The lateral gains are synthesized again using `lateralObserverConfig("mncav")`,
and the actual hybrid lateral observer is executed from the recorded motion.
Its lateral output reproduces the previous motion preparation exactly. The
nominal MnCAV Pacifica Hybrid model is:

| Parameter | Value |
|---|---:|
| Mass | 2,273 kg |
| Front CG distance | 1.374605 m |
| Rear CG distance | 1.714395 m |
| Yaw inertia | 5,874.607330 kg m² |
| Front axle cornering stiffness | 108,238.095238 N/rad |
| Rear axle cornering stiffness | 80,817.777778 N/rad |
| Steering ratio used in motion preparation | 16.2 |

The yaw inertia and cornering stiffnesses remain nominal priors, not identified
loaded-vehicle values. Lateral synthesis uses nine speeds from 5 to 30 m/s and
longitudinal acceleration vertices -3 and +3 m/s². The exported gain array is
2 x 2 x 9 x 2. The H2 bound is 0.4061147774 and the maximum certificate margin
is -4.0273954e-6. The hybrid runtime handles stationary/crawl/out-of-range
operation; the LPV certificate does not by itself certify all hybrid behavior.

The existing [motion-aided design](../mncav_motion_aided_20260914/design.md) uses
the seven states `[X,Vx,Ax,Y,Vy,Ay,psi]` with direct signed velocity and
acceleration correction. Its fixed physical gains are
`[kp,kv,ka,kpsi] = [4,4,12,4] /s`. The structured certificate has translation
dissipation margin 1.83614988, translation decay bound 0.91807494 /s and
heading decay bound 1 /s, conditional on the declared input/disturbance bounds.
Geometric information uses scale 0.001 and a declared minimum pose weight
0.25. The measured maximum course-angle rate is 0.3549753123 rad/s, within
the existing 0.4 rad/s envelope. No rate clipping is used.

This is a fixed-gain transfer to newly rebuilt-map measurements. The existing
gains were previously chosen using the first 60 seconds of this already
inspected drive; this experiment is not independent gain validation. The
after-60-second metrics are reported as a time partition, not a newly unseen
holdout. No gains, sensor offsets or vehicle parameters are selected from the
current error scores.

## Gap diagnosis and implemented reconstruction

In the initial zero-mode replay, accepted frames 803 and 834 bracket
80.200052--83.298762 seconds. The 309 intervening uniform samples account for
75.54% of the LiDAR-interpolation squared error and 79.75% of the observer
squared error. Even linearly connecting the two *reference* endpoints departs
from the intervening reference path by 2.0191 m. This reference-chord experiment
is diagnostic only: its positions never enter the reconstruction or observer.

Thus this large peak is mainly a mismatch between straight, constant-velocity
interpolation and the measured trajectory through a long gap. The observer
is continuously pulled toward that inaccurate interpolation. This finding
does not show that the vehicle dynamic model cannot turn or change speed.
The existing observer tests cover constant acceleration and turning; the new
reconstruction test uses an analytically specified accelerating turn.

The implementation in
[reconstructMotionAidedLidarGaps.m](../../localization/reconstructMotionAidedLidarGaps.m)
uses the original accepted endpoint poses `(p_i, psi_i)` and `(p_j, psi_j)`.
For `lambda=(t-t_i)/(t_j-t_i)`, it computes

\[
\begin{aligned}
\psi_m(t)&=\psi_i+\int_{t_i}^{t}r(s)\,ds,\\
\psi_b(t)&=\psi_m(t)+\lambda[\psi_j-\psi_m(t_j)],\\
d(t)&=\int_{t_i}^{t}R(\psi_b(s))[v_x(s),\hat v_y(s)]^\mathsf{T}\,ds,\\
p_b(t)&=p_i+d(t)+\lambda[p_j-p_i-d(t_j)].
\end{aligned}
\]

Trapezoidal integration uses the existing motion/frame union grid. Only
intervals longer than 0.25 seconds change; shorter intervals remain the original
linear reconstruction. This timestamp criterion was fixed for the diagnostic
and then used for the implementation; it is not an error-based rejection rule.
The three modes modify respectively 11, 13 and 16 intervals. All original
accepted poses and geometric information remain bit-for-bit identical at
their original integration knots. Generated interior poses are continuous-input
reconstruction knots, not additional LiDAR detections. The observer still uses
piecewise-linear input between these knots and its original RK4 dynamics.

Both endpoint constraints use the later accepted match. The method is explicitly
offline and compatible with the requested precomputed replay; setting processing
delay to zero does not make this interpolation causal in real time. Information
matrices retain their original geometric interpretation; neither interpolated
values nor reused motion inputs are claimed to be independent measurements or
new calibrated covariances. The change addresses input reconstruction, with the
observer gains and their conditional certificate unchanged.

## Lateral ablation and remaining limits

An ablation replaces `vy`, `beta` and `betaDot` with zero at fixed gains. With
the original linear gaps, accepted-frame RMSE changes from 11.8210 to 11.6254 cm
and uniform RMSE from 26.4000 to 25.4668 cm. With motion reconstruction in both
cases, the full lateral path gives 12.2595 cm uniform RMSE versus 12.2753 cm
with zero lateral output; accepted-frame RMSE is 11.7369 versus 11.6032 cm.
Consequently this drive does not establish a consistent, substantial benefit
from the lateral observer by itself. The final reported configuration retains
the requested actual lateral observer; the ablation selects no parameters.

Motion fusion improves the large-error tail while increasing the median and
reducing the fraction of very small errors. Sensor bias, timing and nominal
model mismatch remain possible contributors to this tradeoff; this experiment
does not identify one as its unique cause.

The map and query features are from the same drive. Every frozen per-frame
matching diagnostic was initialized relative to INSPVA; these measurements
are not demonstrated autonomous tracking output. INSPVA is also the provisional
evaluation reference, not independent physical ground truth. Its native rate
is about 50 Hz, interpolated onto the 100 Hz evaluation grid. The native-frame
comparison uses the exact frame reference retained with the matching experiment.
The observer and motion reconstruction receive neither reference positions nor
reference-based state resets. These downstream safeguards do not remove the
upstream correlation or reference-seeded initialization. The original failing
LiDAR-only recursive run is not replaced by a claim of successful online
closed-loop localization.

## Reproduction and verification

From the repository root, prepare evaluation data:

```sh
uv run --offline --with numpy --with pyproj python scripts/prepareInspvaObserverReference.py
```

Then run in MATLAB:

```matlab
setupVehicleLocalization;
runInspvaMapObserverComparison();
analyzeInspvaObserverReplay();
runInspvaMotionGapComparison();
```

Required frozen artifacts are the September 14 motion MAT and the completed
September 15 INSPVA-map matching folder. The scripts synthesize lateral gains,
execute the actual observers, save all comparison populations and export
figures. There is no randomized trial or random seed in this deterministic replay.

Validation performed:

- 36 existing MATLAB tests pass: motion-aided observer (14), lateral observer
  (15), and exact frame-aligned replay (7).
- Five new gap tests pass: analytical accelerating turn, noisy endpoint and
  information preservation, unchanged frequent measurements, heading lift
  through pi, and rejection of delayed or incomplete-anchor inputs.
- All five changed/new MATLAB files have zero factory Code Analyzer findings.
- Exact accepted-pose/information injection is verified for every mode. The
  lateral replay repeats exactly, and each zero-mode observer repeats exactly.
  Halving the RK4 maximum step from 5 to 2.5 ms changes position by at most
  8.4021e-9 m for both input reconstructions.
- Independent Python standard-library checks reproduce five statistics in
  each of 48 available CSV metric rows within 1e-9, covering every native
  population for all three modes and every zero-mode uniform population.
  All 3,510 frame/mode/acceptance pairs agree. The original matching-error
  round trip differs by at most 4.6796e-9 m because positions were serialized.
- A supplemental runner initially supplied CSV-round-tripped times to an
  exact-knot API; its input assertion stopped execution. The runner now takes
  anchor times from the saved MAT frame indices and checks CSV agreement
  separately (maximum discrepancy 4.9738e-13 s), without fitting any clock.
- Plot exports were inspected and legend layout corrected. MATLAB emitted
  a vector-export performance advisory; numeric outputs completed normally.

The independent check is reproducible with
`python research/mncav_inspva_observer_20260915/analyze_results.py`.
[Operational hashes](operational_artifact_manifest.json) identify retained
source/output MAT and CSV artifacts. Full operational arrays and plot binaries
remain under `output/mncav_inspva_observer_20260915/`; compact error tables,
configuration, diagnostics and validation records accompany this report.

The requested offline observer comparison is complete. It supports reduced
RMSE conditional on these frozen matches and this reference, with clear median,
lateral-model and online-generalization limitations.
