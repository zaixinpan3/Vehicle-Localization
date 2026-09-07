# Why reliable D2D positions do not stabilize the current observer

Date: 2026-09-07. Follow-up to the
[full Mississippi replay](mississippi_full_sequence_replay.md).

## Finding

The observed failure is primarily a feedback-design failure, not absence of
LiDAR pose information. On this recorded interval, supplying perfect pose
values at the same accepted LiDAR timestamps still causes divergence. Removing
delivery delay or reducing the integration step does not remove it either.
The existing information-to-weight map and short correction pulses do not give
the seven-state system sufficient stabilizing feedback when GNSS XY disappears.
Velocity/acceleration errors grow first; quadratic invariant feedback then
amplifies large errors into numerical overflow.

This is a diagnosis of one nominal recorded experiment, not a global
observability theorem or a certified replacement observer. Reliable sampled
poses can constrain position while a particular dynamic estimator using those
poses is unstable. A positive definite information matrix alone does not prove
stability of the estimator that consumes it.

## Information reaches the observer, but is attenuated

In receiver seconds `[40,49.57)`, the matcher accepts 87 of 95 scans, with
all-frame position error below 0.398 m. Every accepted pose has positive definite
information. The maximum gap between accepted timestamps inside this interval
is 0.4714 s. This is neither a complete pose outage nor uniformly perfect data.

The observer maps each XY information eigenvalue using

\[
 w=\frac{\lambda}{\lambda+25}.
\]

Median weak/strong XY eigenvalues are 9.1883/11.763, giving median weights
0.26875/0.31997. Across these events the weights range from 0.15927 to 0.36473.
The matrix is not used directly as an inverse measurement covariance. Its
translation/heading cross block is also unused by the current split-channel
observer. The information scale is an uncalibrated composite Gaussian scale;
the constant 25 has no demonstrated statistical calibration to this matcher.

Every pose acts for only 30 ms at its physical timestamp during replay. The
nominal scan interval is about 100 ms, with rejected-frame gaps on top. Thus
the already attenuated correction is inactive much of the time. A delayed
event still acts through history replay; it is not simply discarded because
its delay exceeds the pulse length.

## The LiDAR position gain is not the GNSS position gain

The runtime uses, with `D=diag(theta.^[1,2,3,1,2,3,1])`,

\[
 \dot{\hat z}=f(\hat z,u)
 +DK\,\Omega(y_b-C_b\hat z)
 +DP^{-1}C_l^T W(y_l-C_l\hat z)
 +\frac{DN}{\theta^3}(y_h-h(\hat z,u)).
\]

GNSS absence sets the first two entries of `Omega` to zero. LiDAR XY retains
the separate `DP^-1*Cl'` gain. For an X residual, the main gains acting on
`[X,Vx,Ax]` are approximately:

| Gain before information weighting | X | Vx | Ax |
|---|---:|---:|---:|
| GNSS/base channel `D*K(:,1)` | 16.8872 | 127.8895 | 310.2204 |
| LiDAR channel `D*(P\Cl')(:,1)` | 3.8659 | 17.4637 | 57.0109 |

The latter is then multiplied by a directional weight around 0.3. Its relative
position/velocity/acceleration gains matter, not just its total magnitude.

For intuition, isolate one straight-line triple-integrator chain, omit
invariant and cross-axis terms, and apply a constant scalar weight `w`. Its
error characteristic polynomial is

\[
 s^3+w l_1s^2+w l_2s+w l_3.
\]

The cubic Hurwitz condition requires `w*l1*l2 > l3`. With the LiDAR gains
above, this means **`w > 0.84444`**, even for continuous injection. The observed
weights are well below it. This simplified calculation explains why a positive
weight can still produce unstable oscillations; it is not the certificate for
the full nonlinear, sampled system.

A separate full seven-state local calculation freezes the actual state/input
at 40 s and uses the mean recorded information-shaped weights from
`[40,49.57)`. It retains the model, invariant Jacobian, yaw correction and all
gain cross terms. A representative period has 30 ms correction followed by
70 ms without pose correction:

| Frozen local variant | Largest real eigenvalue during correction (1/s) | 100 ms transition spectral radius |
|---|---:|---:|
| Current LiDAR gain and weights | +0.71643 | 1.07853 |
| Current LiDAR gain, unit XY weight | -0.01475 | 1.07328 |
| Base position gain, actual XY weight | -0.74460 | 1.05280 |
| Base position gain, unit XY weight | -3.51665 | 0.91285 |

The no-pose Jacobian's largest real eigenvalue is +0.37940/s. Spectral radius
above one predicts local error growth in this frozen periodic model. Even a
stable correction-on matrix does not guarantee a stable pulse cycle. Actual
recorded periods, weights and states vary; these numbers are explanatory local
diagnostics, not a replay of the entire linear time-varying error system.

## Controlled recorded-input ablations

All cases start from the same initial estimate and recorded inputs at 0 s,
remove GNSS XY on `[40,60)`, and attempt to reach 61 s. Except for the named
substitution, each case keeps the same accepted D2D timestamps, information,
nominal vehicle model, gains, 150 ms delay and 30 ms pulse rule. Substitutions
apply over the whole run, not just after 40 s; no estimate is reset at outage.
The pose reference remains the same-drive GNSS/INS reference. Recorded known
roll/pitch remain available to the precomputed D2D stream.

| Diagnostic substitution | Result |
|---|---|
| None | Nonfinite at 49.5700 s |
| Replace LiDAR pose values with interpolated reference `[X,Y,psi]` | Nonfinite at 49.5300 s |
| Set LiDAR delivery delay to zero | Nonfinite at 49.5700 s |
| Saturate XY weights near one, retain original information | Nonfinite at 49.2300 s |
| Extend LiDAR hold window from 30 to 120 ms | Nonfinite at 47.3500 s |
| Use base-channel position gain for LiDAR, retain actual weights | Nonfinite at 52.2500 s |
| Use base position gain and near-unit XY weights | Finite to 61 s; outage position RMSE 0.50848 m, maximum 1.60931 m |
| Four RK4 substeps per 10 ms input interval | Nonfinite at 49.5475 s |
| Remove invariant feedback | Finite to 61 s, but outage position RMSE 1440.02 m and maximum 9493.57 m |

The oracle pose experiment preserves accepted-event gaps and measurement
information; it isolates pose-value error, not every possible measurement
availability effect. Near-unit weights use `translationInformationScale=1e-8`,
not fabricated high-information matrices. The base-gain experiment substitutes
`D*K(:,1:2)` for `D*(P\Cl')`; it is not shipped as a production repair.
The nominally successful combined substitution still has substantial transients
and no new certificate. More aggressive gains or longer holds alone are not
monotone improvements, as the failed cases demonstrate.

The 2.5 ms substep result closely tracks the original pre-overflow trajectory:
at 48 s, position errors are 20.1985 m versus 20.2309 m. The small movement of
the failure time is inconsistent with blaming the underlying growth solely on
a 10 ms numerical integration step. It does not establish convergence of every
event-discontinuity detail of the integrator.

## Why the final failure becomes explosive

The position output comes from seven states
`[X,Vx,Ax,Y,Vy,Ay,psi]`, not direct replacement with the latest LiDAR position.
At 44 s the baseline position error is 1.039 m; at 46 s it is 5.714 m, with
`Vy=-21.479 m/s` and `Ay=-5.018 m/s^2`. At 49.4 s the actual longitudinal speed
input is 12.108 m/s, while the estimated speed is 498.2 m/s. These states are
far outside the certificate's operating box.

The speed invariant uses `vx_meas^2+vy_meas^2 - Vx_hat^2-Vy_hat^2`. Its
fixed-gain contribution to both global velocity derivatives is

\[
 \frac{0.02}{\theta}
 (v_{x,m}^2+\hat v_{y,body}^2-\hat V_x^2-\hat V_y^2),\qquad\theta=3.5.
\]

For a large negative global `Vy`, a negative speed-squared residual pushes
`Vy` further negative. At 49.4 s this term alone contributes approximately
**-1417.5 m/s^2** to `dVy/dt`; the complete invariant-correction vector has norm
27886 (mixed state-derivative units, not one physical acceleration). Other
quadratic velocity/acceleration invariants also grow. This is the final
positive-feedback amplification after loss of stabilizing pose correction.
Removing that channel avoids overflow through 61 s but leaves kilometer-scale
position error, separating numerical overflow from successful estimation.

## Consequence for the next design

The preceding [certificate audit](mississippi_full_sequence_replay.md#failure-and-timing-interpretation)
already showed that the existing proof assumes GNSS XY available and excludes
pose-pulse gaps. The new ablations explain the actual trajectory failure more
directly: reliable pose data and positive information do not compensate for an
unstable correction law.

A replacement should treat GNSS and LiDAR as sources of absolute pose
constraints with explicitly modeled availability, information scaling, update
interval and fixed delay. The gain design must cover GNSS absent with bounded
LiDAR gaps, while respecting the full state/domain assumptions. Keeping a
30 ms arbitrary injection window or increasing the existing weight is not a
justified solution. The diagnosis does not identify all effects of vehicle
mismatch, map correlation or measurement calibration, and it makes no general
robustness claim for a newly chosen gain.

## Reproduction and validation

```matlab
setupVehicleLocalization;
p=fullfile(pwd,'output','mississippi_20240607_120931_20260907');
r=diagnoseMncavObserverOutage(p,fullfile(p,'outage_diagnosis'));
plotMncavOutageDiagnosis(fullfile(p,'outage_diagnosis'));
```

The diagnostic generates a temporary observer source copy with explicit
substitutions, records that source locally and deletes the temporary path
afterward. Returned diagnostic artifacts have `certified=false`; no production
observer implementation or design artifact is changed. The unmodified-copy
baseline exactly matches all seven saved online states through 48 s
(maximum absolute difference zero), and reproduces the 49.57 s failure.

All nine counterfactuals were actually executed. Failed attempts preserve their
messages and retain a finite prefix through 48 s, or an earlier whole second
before failure; finite prefixes are not advertised as successful full runs.
The integration tracer reports the end of the failing substep, rather than
claiming an exact continuous blow-up time. Full diagnostic MAT files and the
PNG/PDF plot remain locally under `output/.../outage_diagnosis/`. Compact
CSV exports are in
[`results/mncav_outage_diagnosis_20260907/`](results/mncav_outage_diagnosis_20260907/).
Both new MATLAB files have zero factory Code Analyzer findings. Assertions
check exact baseline reproduction and the stated controlled-case outcomes.
No production algorithm changes require a new production regression run.
