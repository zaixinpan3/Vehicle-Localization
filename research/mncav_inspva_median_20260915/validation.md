# Why the INSPVA-map observer increases median error

## Material Passport

Date: September 15, 2026. Mode: executed diagnostic experiments and validation.
Verification status: VERIFIED for the repeated production trajectory, numerical
channel decomposition and exported descriptive statistics. Scope: the frozen
zero-seed INSPVA-map replay, all 1,084 full accepted frames and the original
11,690 uniform samples. Sources: the completed
[fusion experiment](../mncav_inspva_observer_20260915/validation.md), original
sensor logs, the separate 12:11:24 drive and primary NovAtel frame documentation.
Twenty-two observer controls are retained. Reference substitutions are labeled
diagnostic oracles, not candidate localization results. No production algorithm,
calibration, map, matching outputs or default gains are modified.

## Finding

The median increase is primarily associated with a persistent lateral-velocity
inconsistency relative to the current INSPVA reference, combined with a position
correction gain that converts that inconsistency into several centimeters of
position displacement. Simultaneously, filtering substantially reduces the
small population of large LiDAR errors. Their squared-error improvement is
large enough to reduce RMSE despite making a majority of individual frames worse.

This is more specific than a generic statement that smoothing trades accuracy
for smoothness. With the position-input channel held fixed, replacing only the
lateral-velocity input with the INS-implied value reduces the median from
9.6345 to 6.2730 cm. Replacing longitudinal velocity or acceleration alone has
much smaller effects. However, this identifies a problematic signal relationship
relative to INSPVA, not which physical sensor or frame definition is correct.

## Paired distribution: where the median change comes from

The original raw LiDAR median is 7.8867 cm and observer median is 9.6345 cm.
At exactly the same accepted frames, 58.5793% become worse and 41.3284% become
better; one frame is equal. The median of paired changes is +0.9974 cm,
whereas the difference of medians is +1.7479 cm. These are distinct statistics.

| Bin defined by original LiDAR error | Frames | Raw median (cm) | Observer median (cm) | Frames becoming worse |
|---|---:|---:|---:|---:|
| Below 5 cm | 274 | 3.2960 | 7.3987 | 88.32% |
| 5--10 cm | 394 | 7.0880 | 9.4596 | 74.62% |
| 10--20 cm | 274 | 13.673 | 11.705 | 34.31% |
| At least 20 cm | 142 | 29.485 | 13.422 | 3.52% |

The last 142 frames account for 65.81% of the raw squared error. Their
squared-error sum decreases from 15.7605 to 3.7184 m². The two initially
accurate bins together increase their squared-error sum by 3.6418 m². Overall,
the squared-error sum decreases from 23.9479 to 14.9327 m², yielding the reported
14.8634 -> 11.7369 cm RMSE. The bins are descriptive and selected by the raw
outcome; they are not an independent causal experiment or a future gating rule.

This effect is not confined to rejected frames or long-gap recovery. Excluding
accepted endpoints around every gap longer than 0.25 s and one second after
each right endpoint leaves 974 accepted frames. Their medians are still
7.7311 -> 9.4643 cm. At 611 moving, approximately straight frames
(`vx>=5 m/s`, `abs(r)<0.03 rad/s`), the change is 7.6606 -> 9.9023 cm.
At 385 moving turning frames it is 9.6809 -> 10.069 cm. The median increase
is therefore not evidence that the dynamic model simply cannot handle turns.

## Mechanism in the implemented observer

The implemented position dynamics are

\[
\dot{\hat p}=\hat v+k_pW_p(p_L-\hat p).
\]

Let `p_R` be the evaluation reference trajectory, `e=hat(p)-p_R`,
`n_L=p_L-p_R` and `Delta v=hat(v)-dot(p_R)`. Then

\[
\dot e=-k_pW_pe+k_pW_p n_L+\Delta v.
\]

With frozen inputs and weights, split the two forced vector responses:

\[
\dot e_L=-k_pW_pe_L+k_pW_p n_L,\qquad
\dot e_M=-k_pW_pe_M+\Delta v,\qquad e=e_L+e_M.
\]

The initial LiDAR position discrepancy is assigned to `e_L`; `e_M(0)=0`.
The position-input response includes the already fixed offline motion-gap
reconstruction. The motion response includes heading/model/input errors and
any inconsistency between navigation velocity and the derivative of reference
position. The two responses are not statistically independent, and their
scalar error medians cannot be added.

| Diagnostic contribution, accepted frames | Magnitude median (cm) | Mean forward component (cm) | Mean left component (cm) |
|---|---:|---:|---:|
| Position-input response `e_L` | 5.7402 | -0.3580 | -1.9439 |
| Motion-state response `e_M` | 6.0935 | +1.0448 | **-4.5700** |
| Vector sum / actual output | 9.6345 | +0.6868 | **-6.5139** |

Positive lateral means vehicle left, so the persistent additional displacement
is toward the right relative to INSPVA. The reconstructed sum differs from the
original position trajectory by at most 7.1780e-6 m. An independent check using
the clean-position-input oracle agrees with `e_L` within 5.6401e-9 m and with
`e_M` within 7.1764e-6 m. This rules out numerical decomposition error at the
centimeter scale.

The XY weight eigenvalues range from 0.998132 to 0.999949; their medians are
0.999902 and 0.999932. Thus the current effect is not caused by these frames
being assigned very small geometric-information weights. With `kp=4/s`, the
position correction time constant is approximately 0.25 s. In a locally steady
case, a persistent velocity discrepancy creates approximately

\[
e_M\simeq(k_pW_p)^{-1}\Delta v.
\]

On the uniform straight-moving subset, the estimated global velocity has a
mean leftward-component discrepancy of -0.2298 m/s relative to the reported INS
velocity. Dividing by 4/s gives about -5.74 cm, consistent in scale with the
observed rightward displacement. This estimate is local and approximate; the
full decomposition above handles changing orientation and inputs.

Zero LiDAR processing delay does not eliminate this response time or velocity
inconsistency. Conversely, a 0.25 s position time constant does not imply a
blind `speed*0.25 s` trajectory lag: correct velocity prediction cancels that
motion. The residual velocity discrepancy is what drives the extra offset.

## Controlled substitutions and gain sensitivity

Each run uses identical frozen position-input reconstruction and geometric
information. Even when lateral velocity is substituted, gap reconstruction is
held fixed so the control isolates the downstream observer channel. Every
control uses all original evaluation times. INS-implied planar body velocities
are computed from the logged east/north velocity and azimuth; acceleration
controls differentiate 0.21-second moving-mean velocity. These reference-fed
controls diagnose consistency and cannot be claimed as operational performance.

| Control | Reference injected downstream? | Accepted-frame median (cm) | RMSE (cm) |
|---|---|---:|---:|
| Actual configuration | No | 9.6345 | 11.7369 |
| Reference longitudinal velocity only | Yes | 9.5826 | 11.618 |
| Reference lateral velocity only | Yes | **6.2730** | 8.6597 |
| Both reference body velocities | Yes | 5.9238 | 8.4974 |
| Reference body acceleration only | Yes | 9.4938 | 11.770 |
| Reference heading input only | Yes | 10.218 | 12.545 |
| Reference heading and yaw rate | Yes | 10.248 | 12.546 |
| Acceleration derived from measured velocity | No | 9.5009 | 11.762 |

Reference heading alone worsens this discrepancy, partly consistent with the
actual heading error compensating some velocity-frame mismatch. That is an
interaction, not evidence that inaccurate heading is desirable or that the
INS heading is known wrong. The diagnostic velocity replacement does not
identify true tire sideslip or demonstrate a new estimator accuracy.

Position-gain sensitivity holds the other three gains at `[4,12,4]`:

| `kp` (/s) | Accepted median (cm) | Accepted RMSE (cm) | Maximum (cm) |
|---:|---:|---:|---:|
| 2 | 14.627 | 16.043 | 37.224 |
| 4, current | 9.6345 | 11.7369 | 32.905 |
| 8 | 8.0841 | 11.088 | 38.252 |
| 12 | 7.9608 | 11.339 | 43.395 |
| 16 | 7.8964 | 11.604 | 46.256 |
| 24 | 7.8527 | 12.047 | 48.353 |

Increasing position correction suppresses the persistent motion contribution,
while following more measurement noise and increasing the maximum relative to
the current observer. The `kp=24` median is only about 0.034 cm below raw LiDAR;
this small same-drive difference is not evidence of a generalizable advantage.
Velocity gains 12/24 and acceleration gains 6/24 do not produce a comparable
median reduction. All tested controls satisfy their reported course-rate
envelope, and all gain cases pass the existing structured design check.

The current gains were chosen earlier with a different map and an RMSE objective
with RMSE/P95/maximum/heading constraints. Median was not a selection criterion.
Transfer of that setting to cleaner map measurements is not automatically
optimal. The present sensitivity analysis changes no default and promotes no
full-drive fitted setting as independently validated.

## What is known about the lateral mismatch

On 6,303 uniform straight-moving samples, the lateral observer output averages
0.00680 m/s while INS-implied planar lateral velocity averages 0.26985 m/s.
The difference remains -0.25635 m/s after restricting to INS good-status samples.
Shifting the INS lateral series by -0.5 to +0.5 s leaves the matched-subset mean
discrepancy between -0.2627 and -0.2623 m/s. A simple small timing shift therefore
does not account for this persistent component; this is not proof that every
sensor latency is zero.

The physical interpretation is unresolved. A lateral observer's CG velocity and
an INS output-point velocity are not interchangeable without reference-point
alignment. In a planar rigid body, a longitudinal point displacement `ell_x`
contributes `r*ell_x` to lateral velocity. A small yaw-frame offset `delta`
contributes approximately `delta*vx`. NovAtel documents separate body, vehicle
and user frames, configurable rotations, and configurable output translations.
[SETINSROTATION](https://docs.novatel.com/OEM7/Content/SPAN_Commands/SETINSROTATION.htm),
[reference-frame definitions](https://docs.novatel.com/OEM7/Content/SPAN_Operation/Definition_Reference_Frames.htm).

A descriptive fit of `insVy-estimatedVy = delta*vx + ell_x*r` on moving samples
up to 60 s gives `delta=1.2683 degrees`, `ell_x=-1.8030 m`; on later samples,
lateral discrepancy RMSE changes from 0.24218 to 0.08578 m/s. However, adding
an intercept changes these coefficients to 2.4181 degrees, -1.5423 m and
-0.25736 m/s. This ambiguity demonstrates why the fitted numbers must not be
treated as measured extrinsics or a proven physical cause. No fit is applied
to the observer, and this within-drive temporal check uses an already inspected
recording rather than an independent test drive.

There is also material evidence against declaring a universal fixed yaw offset:

| Native INS near-straight subset (`vx>5`, `abs(r)<0.03`) | Samples | Median course minus reported heading |
|---|---:|---:|
| Evaluation drive, 12:09:31 | 3,128 | +1.28557 degrees |
| Separate calibration drive, 12:11:24 | 1,192 | +0.00407 degrees |

The separate export lacks the INS-status field, so these subsets do not have
matched quality filtering. Both use their recorded INS velocities and attitude;
neither is an independent heading/sideslip measurement. Real sideslip, changing
INS attitude error, output configuration/reference-point differences and model
mismatch remain confounded. The finding supports checking frame and point
consistency before accepting new tire parameters or a fixed bias correction.

## Validation, scope and next design decision

`analyzeInspvaObserverMedian` reproduces the saved actual seven-state trajectory
bit-for-bit, runs all 22 controls, exports paired/grouped distributions and
performs the forced-response decomposition. Factory Code Analyzer reports zero
findings. Independent Python checks reproduce the primary RMSE and median,
all four bin medians/counts/worsening fractions, and the squared-error sums.
The component-versus-oracle checks and all control rate audits are retained in
[validation_checks.json](validation_checks.json). No new production code is
introduced, so existing runtime unit tests are not rerun merely for this audit.

Reproduce from the repository root:

```matlab
setupVehicleLocalization;
analyzeInspvaObserverMedian();
```

```sh
uv run --offline --with numpy --with matplotlib python research/mncav_inspva_median_20260915/analyze_signals.py
```

The full operational MAT, motion arrays and plots are retained under
`output/mncav_inspva_median_20260915/`; compact diagnostic outputs accompany
this report and their originals are identified by the artifact manifest.

Priority: establish a common position reference point and motion/attitude frame,
then validate the lateral signal on independent data; subsequently select pose
gain using both typical-error and tail-error objectives. An explicit motion-bias
state may be appropriate once its physical/reference interpretation is clear.
Simply increasing velocity-channel confidence or assuming more signals must
improve each sample does not address the demonstrated inconsistency.

The same-drive map/query and per-frame reference-seeded matching limitations
remain. INSPVA is the comparison reference, not independently established truth.
This audit explains the implemented observer's discrepancy and identifies a
specific validation gap; it does not certify the correct physical lateral speed.

## Statistical interpretation audit

Coverage: 11/11 fallacy checks. No p-values, independence-based confidence
intervals or significance claims are used for this temporally correlated drive.

| Check | Disposition |
|---|---|
| Simpson reversal | Speed/turn/gap groups reported; stationary median can improve while moving medians worsen. |
| Ecological inference | Paired frame results reported; RMSE is not described as every-frame improvement. |
| Berkson selection | Full-accepted population fixed; limits of acceptance selection retained. |
| Collider conditioning | Raw-error bins are descriptive, not causal evidence. |
| Base rates | Counts and full-population worsening fractions included. |
| Regression to mean | Improvement in high-error bins alone is not proof; controlled channel substitutions and decomposition support the mechanism. |
| Survivorship | The same 1,084 accepted frames are compared; full-trajectory results retained separately. |
| Look-elsewhere | All 22 controls exported, including worsened results. |
| Forking paths | Exploratory gain/geometry probes disclosed; no production setting selected. |
| Correlation versus causation | Software-channel interventions are distinguished from unresolved physical causes. |
| Reverse causality | The direction in the implemented differential equation is explicit; physical frame/bias identification is not inferred from correlation alone. |
