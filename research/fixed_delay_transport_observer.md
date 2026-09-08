# Fixed-delay LiDAR observer without state replay

Date: 2026-09-08. Starting implementation:
`cb3cf07a1272644688e6541babc223f3cce73184`.

The user selected a fixed LiDAR signal delay and excluded observer replay.
The runtime now advances the existing seven-state cascade once in time.
It transports delayed measurements through the existing nominal motion flow,
corrects the current state at delivery, and leaves every previous output
unchanged. No speed jerk, course angular acceleration, or extra estimated
state is required. A bounded buffer contains input-derived transition maps,
not historical observer states or corrections to reintegrate.

This changes the timing and feedback model. The old timer certificate is
retained as a verified reference for the gains; it is **not** a certificate
for the transported runtime. No new robust certificate is claimed here.

## Existing model and the input-only transport

Keep the interleaved state

\[
z=[X,V_X,A_X,Y,V_Y,A_Y,\psi]^\top,
\quad C z=[X,Y,\psi]^\top.
\]

The existing reduced nominal dynamics are affine in this state:

\[
\dot z=A_m(q_m)z+b_m(r_m)+Ew,\qquad q_m=r_m+\dot\beta_m,
\]

where the acceleration row is still
\(\dot a=q_m^2v+2q_mJa\), and \(b_m\) contains the gyro yaw-rate input.
The missing speed jerk, course angular acceleration, course-rate mismatch
and other upstream errors remain in \(w\), as derived in
[the earlier model analysis](observer_proposal_assimilation.md).
`evaluateImprovedObserverChannels` exposes `modelMatrix` and `modelInput`;
their product is exactly the preceding nominal predictor. Its four output
channels and their extension are unchanged.

For the measured input path, define the nominal affine flow

\[
\partial_t F(t,s)=A_m(t)F(t,s),\quad F(s,s)=I,
\qquad g(t,s)=\int_s^t F(t,u)b_m(u)\,du.
\]

Thus a nominal trajectory satisfies
\(z(t)=F(t,s)z(s)+g(t,s)\). The augmented matrix

\[
M(t,s)=\begin{bmatrix}F(t,s)&g(t,s)\\0&1\end{bmatrix}
\]

depends only on motion inputs. It is integrated alongside the current
estimate and stored as short segment maps. At an acquisition time inside a
segment, a third-order RK4 dense extension splits that segment algebraically.
No observer state or innovation is read from a past time. For constant
acceleration and constant yaw rate this interpolation is exact up to roundoff.
For general inputs, map integration and interpolation have numerical error
that remains outside a verified error budget.

## Correction at fixed delivery time

LiDAR frame \(k\) arrives at \(a_k=t_k+\tau\), with configured
\(\tau=0.15\) s by default. Its correction pulse occupies
\([a_k,a_k+T_{\rm on})\), \(T_{\rm on}=0.03\) s, or ends when a newer
frame supersedes it. At time \(t\) during the pulse, set

\[
\tilde z_k(t)=F_k(t)^{-1}[\hat z(t)-g_k(t)],\qquad
r_{L,k}(t)=y_{L,k}-C\tilde z_k(t),
\]

with the yaw residual wrapped in a consistent local chart. Write
\(L=T_\theta K\) and \(L_h=T_\theta N/\theta^3\), retaining the stored
gains and all scaling exponents. The implemented law is

\[
\boxed{\dot{\hat z}=A_m\hat z+b_m+
\sum_{s\in\{L,G\}}F_s L W_s r_s+
L_h[y_h-h_{\rm ext}(\hat z,\beta_m)].}
\]

Each source uses its own acquisition time and flow map. GNSS has a
zero third residual component, equivalently
\(C_G=\operatorname{diag}(1,1,0)C\). The existing full-matrix information
fusion supplies \(W_L,W_G\); GNSS continues to join LiDAR pulses or supply
position-only continuation after the recent-LiDAR regime expires.

Transporting the gain as well as the residual matters: the current injection
is \(F_s L W_s\), not merely \(L W_s\). With exact nominal tracking,
\(\tilde z_k(t)=z(t_k)\), so noiseless measurements produce zero innovation
throughout a moving pulse. There is no raw pose-aging term
\(Cz(t_k)-Cz(t)\). Model error instead enters through the explicitly
transported disturbance below.

The physical information matrix and its nullspace remain at the acquisition
pose. Since \(F_s\) is invertible, any representative change satisfying
\(W_s\delta y=0\) also satisfies \(F_s L W_s\delta y=0\).
No inverse information matrix or positive information floor is introduced.
This statement uses one valid yaw chart; wrapping an arbitrary unbounded yaw
representative across branches is not a global nullspace guarantee.

The separate normalized source weights need not be symmetric. Their sum
retains the preceding symmetric source-information sector, but different
\(F_L,F_G\) prevent replacing the actual error Jacobians by that sum.

## Actual error equation and the missing certificate

Let \(e=z-\hat z\) and define the true accumulated model discrepancy

\[
d_s(t)=\int_{t_s}^t F(t,u)Ew(u)\,du,
\quad z(t)=F_s z(t_s)+g_s+d_s.
\]

On a valid angle branch the source residual is exactly

\[
r_s=C_sF_s^{-1}(e-d_s)+n_s.
\]

Only the weighted error \(\epsilon_s=W_s n_s\) needs a bound. In particular,
a directional LiDAR pose need not have bounded unweighted error in its
nullspace. GNSS is represented by its two measured coordinates and a zero
third noise entry. With the unchanged exact incremental output matrix
\(h_{\rm ext}(z)-h_{\rm ext}(\hat z)=H(t)e\),

\[
\begin{aligned}
\dot e={}&\mathcal A_e(t)e+Ew-L_h n_h\\
&-\sum_sF_sL\epsilon_s
+\sum_s F_sLW_sC_sF_s^{-1}d_s,\\
\mathcal A_e(t)={}&A_m(t)-L_hH(t)
-\sum_sF_sLW_sC_sF_s^{-1}.
\end{aligned}
\]

This equation has no delayed estimation state. Its coefficients depend on
input history and measurement age, and its forcing includes past model
error. It is not the proposal's frozen acquisition-innovation delay equation
or the former replay/current-pose timer equation.

For example, if \(\|A_m(t)\|\le a_*\) and source age is at most \(d_*\),
then \(\|F_s\|,\|F_s^{-1}\|\le e^{a_*d_*}\) and

\[
\|d_s(t)\|\le\|E\|
\frac{e^{a_*d_*}-1}{a_*}\|w\|_{[t_s,t],\infty},
\]

with the quotient interpreted as \(d_*\) at zero. Thus a unified bound can
use the stack \(\nu=[w,n_h,\epsilon_L,\epsilon_G]\), without statistical
independence. Longer delay can amplify model/measurement error and increase
flow conditioning even when a nominal periodic system remains stable.

A sufficient new timer condition would require a uniformly bounded positive
metric \(P\), non-increasing metric resets at allowed delivery events, and

\[
\dot P+\operatorname{He}(P\mathcal A_e)+2\rho P\prec0
\]

for all admissible source ages, flows, weights, input paths and output
increments, or the equivalent condition in the existing scaled coordinates.
If this is independently verified and the displayed forcing has bound
\(\|P^{1/2}d_{\rm total}\|\le g_*\|\nu\|_{[0,t],\infty}\), then

\[
\|e(t)\|\le\sqrt{p_+/p_-}e^{-\rho t}\|e(0)\|
+\frac{g_*}{\rho\sqrt{p_-}}\|\nu\|_{[0,t],\infty}
\]

follows by the usual scalar comparison, with any accepted prehistory also
included in the disturbance horizon. This is a conditional derivation, not
a verified numerical result. Merely inserting the old timer matrices fails
to check the new \(F_sLW_sC_sF_s^{-1}\) terms. The thirteen output bounds
remain reusable, but the transported drift family must be enclosed and
verified. Robust synthesis, disturbance budgets, chart continuation and
discretization error remain open.

Consequently, the runtime reports `certificateVerified=false`,
`certified=false` and `certificateApplicable=false`. The separate
`referenceCertificateVerified` reports only the historical matrix check.
The reference artifact still verifies 720,896 flow inequalities plus resets;
passing its source-weight or scheduling audit does not certify this runtime.

## Why not simply freeze the delayed innovation with the old gains?

`scripts/screenFixedDelayObserver.m` evaluates exact linear period maps for
constant course rate \(q\in\{0,-0.6,0.6\}\), scalar source weights
\(w\in\{0.8,1\}\), periods \(h\in\{0.05,0.08,0.1,0.11\}\) s,
30 ms pulses and 150 ms delay. Auxiliary injection and GNSS are disabled
explicitly; this is a finite nominal diagnostic.

With an untransported gain acting on a frozen acquisition-time error, the
largest period-map spectral radius is **1.193232008**, greater than one.
Thus those recorded gains cannot be presumed suitable just by changing the
residual timestamp. For the transported correction and constant \(A_m=A\),
the period map is similar to

\[
e^{Ah}e^{-LwC T_{\rm on}}.
\]

Its largest radius over the same 24 cases is **0.990101347**. Fixed delay
cancels from this nominal similarity, which requires ordered, nonoverlapping
single-source pulses and constant coefficients. This does not establish
stability with nonlinear auxiliary outputs, multiple source ages, changing
course rate, arbitrary anisotropic information or long missing-frame gaps.
No gain magnitude or test acceptance threshold was reduced to obtain it.

## Runtime contract and tests

- Event-split RK4 advances only forward. Each delivery and pulse expiry is
  handled at its sensor-clock boundary, including between high-rate outputs.
  The maximum integration step remains 10 ms; input interpolation follows
  the selected linear or previous-sample-hold contract.
- The one-second `inputHistoryDuration` contains segment flow maps only.
  A LiDAR configuration must cover the fixed delay plus one integration step.
  Events without available input history are rejected explicitly. Old poses
  cannot initialize a current state without the intervening motion history.
- Missing LiDAR delivery metadata is inferred from the declared fixed delay
  and flagged as assumed. Explicit inconsistent delays are rejected. GNSS
  retains separate arrival timestamps; a superseded older acquisition cannot
  replace a more recent active measurement.
- Pulses start at delivery. The internal yaw lift stays continuous through
  \(\pm\pi\); public heading is wrapped. `z=onlineZ`, and there is no
  `revisedZ` or replay mode. Recorded-data functions with historical `Replay`
  names mean dataset playback, not observer-state recomputation.
- Event-clock incorporation is not a wall-clock deadline measurement.
  End-to-end registration time, scheduling and hardware latency require their
  own measurements. Fixed delay here is the requested signal model.

All **82** targeted MATLAB tests pass: 56 global observer, 14 lateral observer
and 12 registration information tests. Two unchanged synthesis tests were
excluded; stored matrices were verified. The tests cover immutable prefix
outputs, zero-error moving and off-grid constant-acceleration fixtures,
delayed yaw wrapping, actual transported gain, bounded input history, fixed
delay validation, partial-curb stationary heading, missing longitudinal
anchoring, coupled nullspace invariance, and GNSS expiry inside a pulse.
The 24-second seed-2026 cascade cases retain the existing tracking limits.
All 12 changed/new MATLAB files have zero factory Code Analyzer findings.

## Recorded comparison and limits

Three full 116.89-second cases use the same 11,690 high-rate samples and
previously computed semantic D2D events from 1,170 scans. There are 963
exported full-pose events, of which 962 arrive within the run. Perception
and registration were reused, not rerun for this change. The fixed delay is
150 ms and the gains are unchanged. XY and yaw are scored against the recorded
GNSS/INS reference also used for mapping, with the same biased initialization
as the previous experiment.

| Scenario | Previous replay position RMSE / peak (m) | Fixed-delay transport RMSE / peak (m) | Previous / new yaw RMSE (deg) |
| --- | --- | --- | --- |
| Fusion | 0.2680 / 2.0291 | 0.2668 / 2.6600 | 0.5551 / 0.5428 |
| GNSS XY outage, 40--60 s | 0.2710 / 2.0291 | 0.2755 / 2.6600 | 0.5566 / 0.5433 |
| LiDAR only | 0.3906 / 1.8931 | 0.3668 / 1.7258 | 0.5867 / 0.5625 |

Within the GNSS-outage window, position RMSE changes from 0.2854 to 0.2588 m.
The full-run fusion peak worsens; these results do not establish a general
performance improvement. All three runs complete, with no state replay.
They store at most 155 input-history segments with fusion and 134 without
GNSS. Re-execution after the final flow lifecycle change reproduces every
saved state exactly; observer core times in that check were approximately
6.22, 6.14 and 5.52 seconds respectively. Those aggregate times exclude
perception, registration, plotting and file export and do not prove deadlines.

The drive has 184 accepted-pose gaps longer than 110 ms (maximum 1.2987 s),
and all 962 LiDAR weights have minimum eigenvalue below the reference 0.8
sector (minimum 0.1107). Fusion and the outage case each have 55 estimated
velocity-envelope violations. Robust full-run certification remains false.
The reused matching log contains four calls longer than the imposed 150 ms
delay. D2D retains recorded roll/pitch, its map uses the evaluation drive, and
the vehicle inertia/stiffness remain nominal rather than identified.
The synthetic GNSS position outage is not a complete GNSS/INS device loss.

An initial recorded-run attempt encountered a saved verification snapshot
missing the newer `onTime` provenance field. The dataset runner now rechecks
the stored matrices with the current reference verifier before running.
The diagnostic failure is preserved locally; the three successful runs and
their final rechecks are separately recorded. No failed result is presented
as successful, and no gain artifact was regenerated.

## Reproduction and artifacts

```matlab
setupVehicleLocalization;
folder = 'research/results/fixed_delay_transport_20260908';
screen = screenFixedDelayObserver(folder);
suite = [testsuite('tests/improvedObserverTest.m'), ...
         testsuite('tests/lateralObserverTest.m'), ...
         testsuite('tests/registrationInformationTest.m')];
results = run(suite(~contains({suite.Name},'synthesis')));

for scenario = ["fusion","positionOutage","lidarOnly"]
    report = runMncavObserverReplay( ...
        'output/anisotropic_observer_20260908/recursive_8threads', ...
        'output/anisotropic_observer_20260908/final_design.mat', ...
        'output/mississippi_20240607_120931_20260907/vehicle_parameters.json', ...
        fullfile('output/fixed_delay_transport_20260908',scenario),scenario,.15, ...
        SensorFolder='output/mississippi_20240607_120931_20260907/sensors');
end
```

Committed numerical summaries are under
`research/results/fixed_delay_transport_20260908/`: per-test results,
Code Analyzer counts, exact nominal screen, recorded comparison and individual
summaries, final state recheck, provenance and validation metadata. Large
recorded inputs, MAT traces and figure exports remain in their normal ignored
local locations. The source weight, yaw, lateral-stage and geometry capabilities
remain intact; this change does not add reverse-driving support or claim a
new complete ISS certificate.
