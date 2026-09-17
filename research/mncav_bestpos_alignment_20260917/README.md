# Correct BESTPOS output-point mismatch and position correction bandwidth

## Result and scope

The requested MnCAV replay now has **7.8474 cm position RMSE over all 1,169
frames**, down from 39.7953 cm, with median 4.6151 cm, P95 17.1913 cm and maximum
26.8705 cm. On the identical 1,083 accepted LiDAR frames, fusion RMSE is
**6.8231 cm versus raw map matching 14.8629 cm**. Both the reference-point
correction and the GNSS position gain change are needed to explain this result.

This remains the approximately 10 Hz synchronized BESTPOS/LiDAR observer.
Longitudinal velocity is exclusively four-wheel derived. The seven-state
observer and upstream lateral observer remain active. Raw LiDAR poses, motion
inputs, lateral estimates, initial state, frame times and evaluation reference
are identical to the preceding experiment. No measurement-pose transport,
state reset, ODOM input, future observer state, or evaluation-reference attitude
is introduced into the runtime. Real-sample bracket interpolation still has
up to 0.09996725 s of offline synchronization wait; LiDAR processing delay is zero.

These are discrepancies against the recorded INSPVA reference in an existing
same-drive, reference-assisted map/matching experiment. They are not independent
absolute ground-truth accuracy or a complete sampled-system ISS certificate.
BESTPOS types 54/55/56 in this recording remain INS assisted.

## Root cause: different position output points

The earlier pipeline directly fused projected BESTPOS XY with a map located at
the recorded INSPVA output point. Their discrepancy rotates approximately with
vehicle heading and persists at standstill. A common projection and timestamp
therefore do not make these measurements interchangeable.

The [NovAtel SPAN configuration documentation](https://docs.novatel.com/Tools/Content/Manage/ConfigurationSPAN.htm)
allows INS logs to report positions at several configured locations. The
[official driver configuration](https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_driver/config/std_init_commands.yaml)
also distinguishes antenna and INS output locations. The available INSCONFIG
snapshot does not establish a surveyed physical transform. Accordingly, this
implementation calls the estimated quantity a **relative output-point offset**,
not a verified antenna-to-CG installation lever arm. No physical installation
claim is inferred from its numerical value.

`calibrateMncavBestposOutputPoint.py` reads only the separate
`raw_data_2024-06-07-12-11-24_0.bag`. It uses receiver GPS timestamps, identical
EPSG:32615 projection, and grid-convergence-corrected INSPVA yaw. Calibration
uses seconds 1--40, valid BESTPOS and INS status 3, giving 360 sample pairs.
The coordinatewise median of the forward/left relative positions is frozen:

\[
\hat\ell = [1.72485903887133,\quad 0.2644114700440067]^T\ {\rm m}.
\]

The earlier exploratory diagnostic included 390 pairs before the INS-status
gate and yielded [1.72535743, 0.26428146] m; it is not the deployed calibration.
The production script never reads the 12-09-31 evaluation drive.

| Independent calibration drive population | Samples | Raw point discrepancy RMSE | Corrected discrepancy RMSE |
|---|---:|---:|---:|
| Training, 1--40 s, valid INS | 360 | 175.405 cm | 4.263 cm |
| Held-out remainder after 40 s | 379 | 175.966 cm | 4.558 cm |

Applied unchanged to the evaluation drive, this calibration gives 6.1517 cm
output-point discrepancy RMSE versus 172.1798 cm before correction. That last
number is a scoring diagnostic using evaluation-reference yaw; runtime instead
uses its own estimated yaw, as verified independently and by a perturbed-heading
test. No offset is fitted on the evaluation drive.

## Measurement model and uncertainty

The raw receiver model is \(y_G=p+R(\psi)\ell+n_G\). The position injected into
the observer is

\[
\tilde y_G=y_G-R(\hat\psi)\hat\ell.
\]

`correctGnssOutputPoint` performs this operation on the current packet. GNSS
course history also uses corrected positions, with the current predicted
attitude; after heading correction, the position injection uses the updated
attitude. This prevents a rotating reference-point offset from masquerading
as the observer point's course. No GNSS position or yaw is reconstructed
between localization frames.

Reported position covariance is augmented by a calibration residual term and
a declared heading uncertainty term:

\[
\tilde\Sigma_G=\Sigma_G+R\Sigma_\ell R^T+
 \sigma_\psi^2(RJ\hat\ell)(RJ\hat\ell)^T,
\qquad J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
\]

The per-axis training residual uses robust MAD scale with a preselected 5 cm
floor; the resulting \(\Sigma_\ell=0.05^2 I\). The declared heading standard
deviation is one degree. These are weighting allowances, not a propagated
filter covariance or a statistical coverage guarantee. BESTPOS and the INS
reference have unknown shared errors/correlations. Generic
`fullObserverConfig` keeps zero offset; `mncavFullObserverConfig` explicitly
loads this dataset-specific calibration. Legacy saved configurations retain
coincident-point behavior when this field is absent.

The remaining geometry disturbance obeys the planar bound
\(\|d_G\|\leq\|n_G\|+\|\ell-\hat\ell\|+
2\|\hat\ell\||\sin((\psi-\hat\psi)/2)|\).
Thus heading error still couples into position. The previous continuous
comparison matrices alone do not prove ISS of this complete sampled system.

## Correct source-dependent response bandwidth

With output points aligned, old gains gave 12.3236 cm full-frame RMSE and
8.7873 cm accepted-frame RMSE. The 86 frames without a valid LiDAR match had
33.0457 cm RMSE and contributed 52.8975% of squared position error. The old
GNSS position gain was 1/s versus LiDAR's 4/s. In the simple single-source
error equation \(\dot e_p=\delta v-kW e_p\), a persistent velocity error has
steady position error \((kW)^{-1}\delta v\); the slower correction retains
more motion-model error during LiDAR gaps.

The new MnCAV configuration sets GNSS position gain to the existing LiDAR
position gain, **4/s**, giving equal nominal 0.25 s time constants before
information weighting. This is one structural gain choice, not a parameter
sweep minimizing evaluation-reference error. It was chosen after inspecting
the development replay; this is not a blind held-out validation of the gain.
All other gains stay fixed: global [4, 4, 12, 4] and GNSS heading 0.5/s. The
continuous translation comparison matrices are rechecked and remain positive;
`sampledSystemCertified` and `allTheoremHypothesesVerified` remain false.
A synthetic GNSS-only test with 0.2 m/s persistent motion bias verifies the
expected reduction in steady position error from 0.20 m to 0.05 m.

| Controlled change | All-frame RMSE (cm) | Accepted-frame RMSE (cm) |
|---|---:|---:|
| Original geometry, GNSS gain 1 | 39.7953 | 30.4149 |
| Covariance allowance only, GNSS gain 1 | 39.1949 | 29.7593 |
| Point correction only, GNSS gain 1 | 12.1983 | 8.7424 |
| Point correction and covariance, GNSS gain 1 | 12.3236 | 8.7873 |
| Point correction and covariance, GNSS gain 4 | **7.8474** | **6.8231** |

The geometry ablation explains most of the original regression. Adding
uncertainty alone cannot correct a biased measurement model. The small cost
of covariance allowance is retained instead of selecting the best evaluation
number by removing uncertainty.

## Matched-frame comparison and missing channels

| Method, same 1,083 accepted frames | RMSE (cm) | Median (cm) | P95 (cm) | Maximum (cm) |
|---|---:|---:|---:|---:|
| Current aligned BESTPOS fusion, 10 Hz | **6.8231** | **4.4253** | **14.2208** | **22.3164** |
| Previous unaligned BESTPOS fusion, 10 Hz | 30.4149 | 27.8112 | 44.6966 | 126.9795 |
| Historical ODOM fusion, transported 100 Hz | 9.9304 | 6.6913 | 19.9976 | 40.3271 |
| Unmodified LiDAR matching | 14.8629 | 7.8758 | 33.0655 | 57.5065 |

Raw matching has no accepted pose on the other 86 frames, so no all-frame raw
LiDAR RMSE is invented. Current fusion there has 15.8375 cm RMSE; the worst
full-frame error, 26.8705 cm at 80.5715 s, occurs without accepted LiDAR.
The full-frame heading RMSE is 0.5402 degrees (previously 0.3381); accepted-frame
heading RMSE is 0.3467 degrees versus raw LiDAR 0.4245. Position improvement
does not imply improvement in every orientation metric.

| Scenario, all 1,169 frames | Position RMSE (cm) |
|---|---:|
| Both available according to recorded validity | 7.8474 |
| LiDAR channel only | 25.5782 |
| Corrected BESTPOS channel only | 8.8041 |
| GNSS withdrawn at 40--60 s | 8.0542 |
| LiDAR withdrawn at 40--60 s | 7.5925 |
| Both withdrawn at 40--60 s | 74.0815 |
| Alternating channels each second | 13.5621 |

Ablations share the initial LiDAR-based state, as before; GNSS-only is not a
cold-start test. Simultaneous long outages still drift. Slightly lower error
with LiDAR withdrawn over this particular interval shows that more channels
are not guaranteed to improve every realization. Per-window metrics remain
in `metrics.csv`; no failed match or high-error localization frame is removed.

## Validation and reproduction

All **152 MATLAB tests pass**, including ten output-point/bandwidth tests.
The new tests cover rotated-point signs, covariance rotation, invalid
calibration, a turning vehicle, GNSS-only correction, GNSS absence, legacy
configuration, estimated-attitude dependence, continuous gain feasibility and
steady motion-bias rejection. Code Analyzer reports zero findings. Replay and
future-packet prefix invariance are exact. One backward-Euler update per
frame is retained; its position equation residual is 4.0163e-9 m at absolute
UTM coordinates, below the 1e-7 m roundoff allowance. The earlier integration
step setting has no effect; this is not a refinement test of the new runtime.

The independent Python audit reproduces 21 scenario/population metric rows to
4.45e-15, verifies unchanged sources/lateral estimates/other gains, exclusively
wheel-derived Vx, the independent calibration split, corrected-position
identity, covariance identity (1.14e-13 maximum information residual), and
zero virtual pose updates or state resets. No random input is introduced.

```bash
uv run --offline --with numpy --with pandas --with pyproj python scripts/prepareMncavBestpos.py
uv run --offline --with rosbags --with numpy --with pandas --with pyproj python scripts/calibrateMncavBestposOutputPoint.py
```

```matlab
setupVehicleLocalization;
report = runMncavFullObserverExperiment;
validation = validateFullLocalizationObserver;
```

```bash
uv run --offline --with numpy --with pandas --with h5py python research/mncav_bestpos_alignment_20260917/verify_results.py
```

Numerical artifacts, source CSV exports and figures are in
`output/mncav_bestpos_alignment_20260917`; compact audited tables and their
artifact manifest are retained beside this report. Historical baseline outputs
are preserved. Recorded datasets and generated MAT/image binaries are not
committed. This development result meets the same-frame position comparison
target, while independent-map generalization, a verified physical transform,
long simultaneous-outage accuracy and the complete sampled ISS proof remain
open.
