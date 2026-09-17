# Synchronous localization with recorded BESTPOS

Date: September 17, 2026.

The default global localization observer now updates once per native LiDAR
frame, approximately 10 Hz, using aligned BESTPOS, LiDAR, wheel-derived motion
and lateral estimates. It performs one backward-Euler state update per frame.
It does not transport GNSS/LiDAR anchors between frames, generate inter-frame
pose measurements, or produce a 100 Hz localization trajectory. The wheel and
lateral estimators retain their upstream processing and calibration; only
their aligned frame values enter the global observer.

The user's later instruction explicitly replaces ODOM with BESTPOS. This
source migration is implemented despite degraded agreement with the existing
reference. The result is not an accuracy improvement: the observed position
discrepancy points to an unresolved measurement/reference output-point issue.

## Sources and timing

- BESTPOS has 1,170 original packets and a 0.1 s median GPS-time interval.
  LiDAR has approximately the same 10 Hz rate. The current shared evaluation
  covers 1,169 native LiDAR frames within the previous wheel/motion coverage,
  including 1,083 accepted LiDAR poses and 86 unavailable full poses. The final
  original frame outside that coverage remains excluded explicitly, as in the
  preceding experiment; no new high-error interval is removed.
- `prepareMncavBestpos.py` converts recorded latitude/longitude directly to
  EPSG:32615, the existing map projection. It uses BESTPOS GPS week/seconds
  directly, with INSPVA timestamp pairs only to locate the first LiDAR epoch.
  It reads no ODOM and uses no INSPVA position, velocity, fitted offset or
  inferred lever arm. Reported east/north standard deviations are projected
  through a numerical UTM Jacobian; their unreported cross covariance is
  assumed zero. These information matrices are tuning inputs, not proof of
  independent errors across sensors or evaluation reference.
- `prepareMncavObserverReplay(...,IncludeOdom=false)` supplies current wheel
  motion without reading the old ODOM position/reference channel. Older
  experimental callers retain their historical optional exporter behavior.
- `synchronizeLocalizationInputs` retains native LiDAR poses exactly. It
  linearly interpolates real BESTPOS endpoints only at the low-rate frame
  timestamps, without a motion model. Invalid endpoints and gaps over 0.15 s
  withdraw GNSS for that frame. It records both original endpoint times.
- The maximum BESTPOS alignment wait is **0.09996725 s**; the maximum motion
  endpoint wait is **0.00999224 s**. This is offline bracket synchronization.
  Zero LiDAR processing delay is preserved, but the complete alignment does
  not have zero latency. The prior ODOM control needed at most 0.02008013 s.
- The compatibility field `data.highRate` now contains the approximately
  10 Hz global input grid. Wheel speed still originates exclusively from
  four-wheel fusion. Upstream IMU/steering reconstruction remains offline.

**BESTPOS is not pure GNSS in this recording.** Position types are 56
(`INS_RTKFIXED`, 437 packets), 55 (`INS_RTKFLOAT`, 683), and 54 (`INS_PSRDIFF`,
50). All have solution status 0. NovAtel explicitly allows combined GNSS/INS
positions in BESTPOS on SPAN systems. The earlier wording that called this
source raw/pure GNSS is corrected here. [NovAtel BESTPOS documentation](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm).
The prior source audit found no BESTGNSSPOS named topic or raw message ID 1429
in this bag; see [the existing provenance audit](../mncav_reference_roles_20260915/findings.md).

## Discrete update and design scope

The seven-state structure and continuous gain values remain unchanged:
`[kp,kv,ka,kpsi]=[4,4,12,4]`, GNSS position/heading gains `1/0.5`.
There is no accuracy-based gain search in this change. For the actual frame
interval `h=t(k)-t(k-1)`, let

\[
K_p=k_GW_G+k_pW_{L,p},\qquad
b=k_GW_Gy_G+k_pW_{L,p}y_{L,p}.
\]

Unavailable channels contribute zero. The position update is

\[
\hat p_k=(I+hK_p)^{-1}
  (\hat p_{k-1}+h\hat v_k+h b).
\]

The velocity/acceleration update solves

\[
\begin{bmatrix}
(1+hk_v)I&-hI\\
-hq_k^2I&(1+hk_a)I-2hq_kJ
\end{bmatrix}
\begin{bmatrix}\hat v_k\\\hat a_k\end{bmatrix}
=
\begin{bmatrix}
\hat v_{k-1}+hk_vR(\hat\psi_k)[V_x,\hat V_y]^T\\
\hat a_{k-1}+hk_aR(\hat\psi_k)[a_x,a_y]^T
\end{bmatrix}.
\]

Gyro prediction uses the two frame-end rates' average. With qualified LiDAR
yaw, the implicit heading correction has coefficient
`h*kpsi*w/(1+h*kpsi*w)` on the wrapped innovation. Without qualified LiDAR
yaw, admitted GNSS displacement heading uses an implicit sine correction.
The existing two-second motion/displacement heading reconstruction and slow
LiDAR lateral-bias correction operate on frame histories; neither generates
inter-frame absolute-position measurements. Both sources missing leaves only
the state dynamics, with no absolute-position guarantee.

`maximumIntegrationStep` does not affect this mode: there are zero integration
substeps and exactly 1,168 state advances for 1,169 outputs. This is a changed
discretization, not merely downsampling the old output. `designFullObserverGains`
still verifies the continuous matrices as context. The complete sampled,
nonlinear cascade is explicitly marked `sampledSystemCertified=false`; the
continuous certificate is not silently transferred to it. Positive sampled
matrix margins are diagnostics, not a new ISS proof.

Saved old configurations without `timing`, or an explicit
`timing="historical_transport"`, retain the previous runtime for reproducible
historical controls. The current default is `timing="synchronous"`.

## Executed results

Evaluation uses the existing INSPVA reference on identical native LiDAR frame
timestamps. The reference-assisted map, per-frame matching seeds and shared
receiver information remain limitations. All seven ablations share the same
initial LiDAR/motion state; GNSS-only is not a GNSS-only cold start.

| Current BESTPOS + LiDAR metric | Value |
| --- | ---: |
| All 1,169 frames: position RMSE | **39.7953 cm** |
| Median | 28.7819 cm |
| P95 | 58.1510 cm |
| Maximum | 184.8523 cm |
| Heading RMSE | 0.338106 degrees |
| Same 1,083 accepted LiDAR frames: position RMSE | **30.4149 cm** |

| Same 1,083 accepted frames | Position RMSE (cm) |
| --- | ---: |
| Current 10 Hz synchronous, BESTPOS | 30.4149 |
| Development control: 10 Hz synchronous, ODOM | 8.8282 |
| Historical 100 Hz transported observer, ODOM | 9.9304 |
| Raw LiDAR matching | 14.8629 |

The development ODOM control completed before the user selected BESTPOS and
is retained in `output/mncav_synchronous_20260917`. Its whole-frame RMSE is
11.8241 cm. The new timing/discretization was not inherently worse in that
control. Replacing ODOM with BESTPOS substantially changes the position
measurement. The current vs. historical rows change both timing and source;
they are not a single-factor sampling-rate comparison. The all-frame metric
is not interchangeable with the prior uniform 100 Hz whole-trajectory metric.

| Current source/outage scenario | All-frame position RMSE (cm) |
| --- | ---: |
| BESTPOS and LiDAR | 39.7953 |
| LiDAR only with motion observer | 25.5782 |
| BESTPOS only with motion observer | 185.3751 |
| BESTPOS absent 40–60 s | 38.2895 |
| LiDAR absent 40–60 s | 80.8733 |
| Both absent 40–60 s | 89.4793 |
| Alternating channels | 56.6981 |

## BESTPOS/reference discrepancy

Independent diagnostic comparison at native BESTPOS times gives 1.72156 m
position RMSE against INSPVA on 1,169 covered packets. The first of 1,170
BESTPOS packets predates INSPVA coverage by 0.02 s and is excluded only from
this diagnostic, without extrapolating a reference or removing localization
outputs. Rotating their differences into the evaluation body frame gives:

- Median longitudinal/lateral difference: **+1.70417 / +0.27010 m**.
- 5th–95th percentiles: longitudinal **1.60895–1.78850 m**, lateral
  **0.21771–0.30877 m**.
- Thirteen covered stationary samples have median difference
  **+1.79505 / +0.25269 m**.

The persistent body-frame offset, including standstill, is consistent with
different output reference points. It is not a verified physical lever-arm
calibration or proof of its sole cause. The earlier INSCONFIG audit has no
usable translation entries, so no compensation is applied. These reference
differences are diagnostic only; no fitted offset is inserted into runtime.
This explains why source/frame consistency must be resolved before the new
BESTPOS configuration can be claimed to meet the previous accuracy target.

## Validation and reproduction

All **142 MATLAB tests pass**, including 16 new synchronization/discretization
cases. They cover exact straight motion, turn-step convergence, high-gain
implicit contraction, future-frame isolation, independently missing channels,
invalid payloads, mismatched clocks, real-measurement preservation, endpoint
waiting, rejected gaps/extrapolation and required aligned lateral estimates.
Recorded replay is exactly repeatable; future synchronized measurement
mutation leaves the prior state prefix unchanged. This does not assert
causality of the upstream bracket interpolation. Zero analyzer findings remain.

The implicit position equation residual is 2.65e-9 m at absolute UTM
coordinates near 5e6 m. The initial 1e-9 absolute threshold was too tight for
those coordinates; the final 1e-7 threshold admits coordinate roundoff. An
initial BESTPOS export used textual booleans incompatible with MATLAB's
logical conversion; numeric 0/1 fixes the interface. Two alignment warnings
were corrected. An initial diagnostic verifier rejected an out-of-coverage
first BESTPOS packet; explicit diagnostic coverage replaces endpoint clamping.
These initial issues did not require measurement or reference fitting.

Independent Python/HDF5 verification recomputes all 21 scenario/population
rows within 4.89e-15, verifies matching clocks, unchanged LiDAR measurements,
BESTPOS projection/interpolation, wheel-only Vx and zero virtual-pose updates.
The primary replay and full tests run via MATLAB MCP:

```bash
uv run --offline --with numpy --with pandas --with pyproj python scripts/prepareMncavBestpos.py
```

```matlab
setupVehicleLocalization;
report = runMncavFullObserverExperiment;
validation = validateFullLocalizationObserver;
```

```bash
uv run --offline --with numpy --with pandas --with h5py --with pyproj \
  python research/mncav_synchronous_bestpos_20260917/verify_results.py
```

Required original wheel, IMU, steering, matching, map and gain artifacts are
the same as the previous wheel-only experiment. Original data and prior
results remain unchanged. Generated MAT/CSV/plot files stay in
`output/mncav_synchronous_bestpos_20260917` outside public Git; compact results
and verification code accompany this report. Git commit/push and the shared
archive closure preserve the implementation and this negative source-transfer
finding without claiming that the localization accuracy goal was achieved.
