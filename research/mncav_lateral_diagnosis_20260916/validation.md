# MnCAV lateral observer: parameter, gain and model diagnosis

## Material Passport

Date: September 16, 2026. Mode: executed diagnostic experiments and validation.
Verification status: VERIFIED for baseline reproduction, executed controls,
model algebra and exported descriptive statistics. INSPVA is the stipulated
position/velocity/attitude reference for this experiment; its inferred planar
body lateral velocity is the benchmark, not a deployment input. This study
does not reopen the choice of reference or claim independent physical truth.

Input: the frozen `high`, `lateralCfg`, `lateralDesign`, and `lateral` objects
in `output/mncav_inspva_observer_20260915/experiment.mat`, for Mississippi
`raw_data_2024-06-07-12-09-31_0`; reference signals from the preceding
[median-error audit](../mncav_inspva_median_20260915/validation.md).
Population: all 11,690 original 100 Hz samples over 116.89 s; 10,787 moving
samples (`vx>=5 m/s`), including 6,303 approximately straight samples
(`abs(measured yaw rate)<0.03 rad/s`) and 4,484 turning samples. The same
baseline-defined masks are used for every control. No LiDAR acceptance filter
is applied to this lateral-only experiment.

## Answer

The main demonstrated failure is **model/output inconsistency with the
stipulated INSPVA trajectory, propagated by a lateral estimator whose absolute
velocity correction comes from that same model**. The current observer is
stable and effective on matched-model synthetic data, but on this recording it
rapidly draws a correct reference initialization toward a biased estimate.

Ordinary gain scaling, a substantially higher lateral-state design weight,
and local vehicle-parameter changes do not remove the principal discrepancy.
Unidentified tire parameters remain a limitation, but the controls do not
support a claim that modest stiffness/inertia retuning is the solution.
The evidence warrants revisiting the model-to-measurement mapping and the
absolute-velocity correction architecture before another gain optimization.
It does not prove that every bicycle model or LPV observer is unsuitable.

## What is actually running

The output is a hybrid estimator, not just a two-state LPV observer:

1. A master state `[vy, accelerometer bias]` predicts
   `vyDot = measuredAy - estimatedBias - measuredR * measuredVx`.
2. A hidden LPV bicycle observer estimates `[vy, yaw rate]` from steering,
   longitudinal speed, lateral acceleration and gyro yaw rate.
3. At ordinary driving speeds, a persistent correction pulls master `vy`
   toward the hidden bicycle estimate. Current velocity gain is 12/s,
   correction time constant 0.08 s, bias gain 0.5, and bias leakage time
   constant 60 s. Stationary/crawl channels handle low speed separately.
4. The global observer consumes these lateral outputs. In this replay,
   LiDAR/global pose innovations do not feed back into the lateral observer.

See [master prediction and correction](../../localization/lateralObserver/runLateralVelocityObserver.m)
and [the experiment call order](../../scripts/runInspvaMapObserverComparison.m).
The hidden LPV measurement update uses measured lateral acceleration directly;
it does not subtract the master's estimated acceleration bias. Thus that bias
state is not an independent correction of the hidden branch's velocity bias.

On the straight subset, dynamic participation is exactly 1. The master-minus-
hidden velocity difference has mean 0.000239 m/s and RMS 0.012777 m/s. Their
reference RMSEs are 0.27343 and 0.27278 m/s, respectively. Therefore this
principal discrepancy already exists in the hidden branch; gating, crawl
logic, or master smoothing is not its main source.

## The decisive model-compatibility check

The implemented bicycle output equation is

\[
a_y=-\frac{C_f+C_r}{m v_x}v_y+
\frac{l_r C_r-l_f C_f}{m v_x}r+\frac{C_f}{m}\delta.
\]

Using actual recorded steering, forward speed and gyro rate, but substituting
the stipulated reference `vy`, gives these straight-subset means:

| Quantity | Mean |
|---|---:|
| INSPVA reference lateral velocity | +0.269852 m/s |
| Actual lateral observer output | +0.006798 m/s |
| Actual corrected lateral accelerometer input | -0.015143 m/s² |
| Bicycle-model acceleration at reference `vy` | **-1.702836 m/s²** |
| Algebraic `vy` obtained by inverting the output equation | **+0.018098 m/s** |

The model/output residual has RMS 1.73997 m/s² on those straight samples.
By comparison, measured versus INS-derived lateral acceleration has RMS
0.17933 m/s². The INS derivative controls use the preceding audit's declared
0.21 s moving-mean velocity derivative. These discrepancies are distinct from
centimeter-scale position errors and use different units.

The output constraint already prefers a velocity near zero rather than the
reference value near 0.27 m/s. Stronger innovation injection primarily moves
the estimate toward this incompatible constraint; it does not make the
constraint true. An output-only steering adjustment would require a mean
2.03065-degree road-wheel change on the straight subset. This is a diagnostic
equivalent, not an identified steering offset; it need not satisfy the yaw
dynamics and is not applied.

The equation is the usual linear-tire, CG-frame bicycle relation. The
[MathWorks derivation](https://www.mathworks.com/help/ident/ug/modeling-a-vehicle-dynamics-system.html)
explicitly formulates vehicle states at the center of gravity and derives tire
forces from slip. This supports checking reference-point/frame and force-model
compatibility; it does not establish which correction this vehicle requires.

## Why a correct reference state is driven away

Write the nominal observer with `e=xhat-xReference`, and define

\[
d_f=A x_R+B\delta-\dot x_R,\qquad
d_y=Cx_R+D\delta-y_{\rm measured}.
\]

At the reference state, where `e=0`, its error derivative is

\[
\left.\dot e\right|_{e=0}=d_f-Ld_y.
\]

For a consistent model and measurement mapping these forcing terms vanish
(apart from noise). Here they do not. On the straight subset, evaluating the
nominal schedule at reference `vy` gives the following mean contribution to
the lateral-velocity derivative:

| Contribution at reference state | Mean (m/s²) |
|---|---:|
| Nominal plant mismatch `d_f` | -1.689394 |
| Innovation contribution `-L d_y` | -0.968291 |
| Total force away from the reference | **-2.657685** |

This decomposition is a reference-state evaluation, not the actual derivative
along the already biased estimate. MATLAB's algebraic identity residual is
3.55e-15; independent Python reconstruction from CSV differs by at most
9.95e-14. The average kinematic residual
`measuredAy-measuredR*measuredVx-referenceVyDot` is only -8.16e-5 m/s² on the
same subset; this small mean is not a claim that its instantaneous noise is
negligible. The bicycle restoring terms and output relation supply the strong
incorrect absolute-velocity constraint.

A controlled restart at t=20 s initializes both master `vy` and hidden
`[vy,r]` at the reference values, with zero master acceleration bias and zero
correction-injection states, and then uses the recorded inputs:

| Elapsed time | Reference-initialized estimate | INSPVA reference |
|---|---:|---:|
| 0 s | 0.383745 m/s | 0.383745 m/s |
| 0.25 s | 0.080825 m/s | 0.372463 m/s |
| 0.50 s | -0.007760 m/s | 0.365657 m/s |
| 1.00 s | 0.010726 m/s | 0.317528 m/s |

This intervention changes initial conditions, not the production run. It
shows that an accurate initial velocity alone does not solve the problem.

## Gain and parameter controls

Twenty-eight cases were requested by the script. Twenty-seven complete replay
cases succeeded. One re-synthesis case failed and has no replay result.
All successful outcomes, including worse outcomes, appear in
[controls.csv](controls.csv), with all six population strata.

| Case | Moving `vy` RMSE (m/s) | Straight signed bias (m/s) |
|---|---:|---:|
| Actual design | 0.274292 | -0.263054 |
| Zero LPV innovation gain | 0.280498 | -0.272091 |
| Two times LPV gain | 0.272139 | -0.259591 |
| Ten times LPV gain | 0.268959 | -0.254070 |
| Master velocity correction 3/s instead of 12/s | 0.275988 | -0.267097 |
| Master velocity correction 48/s | 0.275308 | -0.262961 |
| Master dynamic bias gain zero | 0.273270 | -0.261116 |
| Both tire stiffnesses multiplied by 0.7; original gains | 0.268873 | -0.266093 |
| Both tire stiffnesses multiplied by 1.3; original gains | 0.278647 | -0.261285 |
| Both tire stiffnesses multiplied by 0.7; new LMI gains | 0.263814 | -0.259386 |
| Lateral design weight 100 instead of 1; new LMI gains | 0.273257 | -0.261507 |
| INSPVA-derived ay, rate, vx, ax; original steering | 0.277926 | -0.263552 |
| Disable dynamic correction to master | 1.987430 | +1.313557 |

Additional retained controls include individual front/rear stiffness and
inertia changes by factors 0.7 and 1.3, steering scale changes by the same
factors, gain factors 0.25/0.5/4, and separate reference ay/rate replacements.
No constant steering offset was fitted or applied. These local parameter
probes are not exhaustive identification and do not rule out all possible
parameter sets or gain designs.

The 1.3-times joint-stiffness re-synthesis found no certified solution among
the existing nine tau candidates. That is a failure of this finite search,
not a proof that no feasible observer exists. The initial harness stopped
at that assertion; it was changed to retain synthesis failures and completed
a second full run. The failed case is preserved in `summary.json`, with no
fabricated metrics or claim of a passed certificate.

Scaled gains and fixed-gain parameter changes are diagnostic perturbations;
the original LMI certificate is not transferred to them. The two successful
new LMI designs passed their own grid checks. No new design is deployed.

## What the gain certificate does and does not establish

The original MnCAV design passes its stored grid certificate. Frozen
`A-LC` matrices evaluated along the moving reference-state schedule have
maximum real eigenvalue -6.96676/s. Pointwise eigenvalues do not prove a
time-varying or hybrid-system stability theorem, but the observed problem is
not an evident numerical divergence or slow transient.

The synthesis minimizes a measurement-noise H2 bound, with nominal model
matrices, error weight initially `eye(2)`, and no sensor covariance scaling
in `trace(L' P L)`. It does not optimize real-data velocity bias. It also
does not include the hybrid master, correction filters, side-slip interface
or their nonlinear participation logic in its two-state certificate.

The copied-nonlinearity Lipschitz error condition used in the certificate
bounds a term that vanishes as `e` tends to zero. An unknown parameter,
reference mapping or output-model discrepancy can remain nonzero at `e=0`,
as measured above. Increasing the configured Lipschitz number is not, by
itself, a proof that such a disturbance is covered. The configuration comment
calling it a robustness margin against unmodeled dynamics should not be read
as a real-data bias guarantee. This audit leaves production comments and
configuration unchanged.

For positive stiffness and positive speed the model's two-output matrix is
full rank (`det(C)=-(Cf+Cr)/(m*vx)`). Thus this is not an assertion of intrinsic
unobservability of the ideal LPV model. The practical limitation is that the
absolute-velocity information is inferred from a model-dependent force
relation that fails the reference-trajectory consistency check.

## Parameters and structural design implications

The runtime uses the MnCAV/Pacifica profile: mass 2273 kg, lf 1.374605 m,
lr 1.714395 m, inertia 5874.60733 kg m², front/rear axle stiffnesses
108238.095/80817.778 N/rad, steering ratio 16.2. Stock mass and geometry
are source-qualified; actual loaded mass is not identified. Inertia is a
rectangular-planform prior, and stiffnesses are legacy nominal values scaled
by mass. They are not measured MnCAV tire properties. See
[parameter provenance](../../config/mncavVehicleParameters.json).

The engineering priority is therefore:

1. Keep INSPVA as the stipulated benchmark and check the complete
   reference-to-model state mapping, steering zero/sign/ratio, and sensor/CG
   reference point. A coordinate or point correction belongs in the estimator
   interface; it is not a reason to discard the benchmark.
2. Validate the force/output and yaw-dynamics equations simultaneously before
   accepting physically positive vehicle-parameter identification. Local
   tuning against lateral velocity alone can hide a yaw/force inconsistency.
3. Provide a separately validated correction of persistent lateral/model
   bias, potentially from LiDAR displacement/velocity and heading, with
   geometry/noise-aware weighting. The existing one-way cascade cannot use
   LiDAR innovations to repair its upstream lateral estimate. Adding bias
   states alone does not establish their observability; this needs a design
   and independent validation.
4. Re-design gains for the resulting state/output model, including sensor
   noise and model-disturbance channels, and verify the actual hybrid runtime.

Removing the bicycle correction entirely is not a solution: its tested
moving RMSE is 1.98743 m/s because the integration/bias system drifts.
The desired redesign must retain bounded drift without forcing a biased
absolute velocity. No such redesign or new localization accuracy is claimed
as completed in this diagnostic task.

## Validation and reproduction

The actual master and hidden states reproduce bit-for-bit. The same MnCAV
design in a matched-model synthetic scenario (seed 2026, original varying
speed/steering profile, 0.01 s step) has post-5 s lateral-velocity RMSE
0.00101175 m/s without measurement noise and 0.00220068 m/s with the original
0.05 m/s² acceleration and 0.002 rad/s gyro noise. These results assess
matched-model convergence, not real-drive accuracy or arbitrary disturbances.

All 15 tests in `tests/lateralObserverTest.m` pass, including model algebra,
schedule interpolation, stored-certificate checks, convergence, low-speed
behavior, continuity and re-synthesis. The new MATLAB analysis function has
zero factory Code Analyzer findings. Independent Python reproduces actual
metrics on all six populations, output inversion, and reference-state error
forcing. The exported PNG/PDF were visually inspected. Runtime code,
production gains and parameters were not modified.

```matlab
setupVehicleLocalization;
analyzeMncavLateralDiagnosis();
runtests('tests/lateralObserverTest.m');
```

```sh
uv run --offline --with numpy --with matplotlib python research/mncav_lateral_diagnosis_20260916/verify_and_plot.py
```

Operational output: `output/mncav_lateral_diagnosis_20260916/`, including
complete run MAT, signals CSV, restart table, tests MAT, figure PNG/PDF and
run log. Compact metrics/checks are exported beside this report. Original
sources and outputs are identified in `artifact_manifest.json`; no recorded
datasets or generated MAT/figure binaries are committed to the public repo.

## Statistical interpretation audit

All 11 checks covered. No p-values, confidence intervals based on independent
samples, or significance claims are made for this temporally correlated drive.

| Check | Disposition |
|---|---|
| Simpson reversal | All/moving/straight/turning/early/late strata retained. |
| Ecological inference | Mean bias and RMS are distinguished; neither is every-sample error. |
| Berkson selection | Full uniform grid; fixed motion masks, not selected by output error. |
| Collider conditioning | Control-dependent participation is reported, not used for sample selection. |
| Base rates | All sample counts and unsuccessful design attempts reported. |
| Regression to mean | Identical inputs and times; model substitution and initialization interventions support the mechanism. |
| Survivorship | Failed synthesis is retained; no result is invented for it. |
| Look-elsewhere | All 28 attempted settings are accounted for; 27 replay results exported. |
| Forking paths | Exploratory controls disclosed; no fitted setting or production change selected. |
| Correlation/causation | Software interventions identify implemented behavior; physical parameter attribution remains unresolved. |
| Reverse causality | Error-forcing direction derived from implemented equations, not an observational correlation. |
