# Conditions for bounded localization without GNSS

Date: 2026-09-07. Analysis baseline: `fc1fe755b29edc660cb988425287ed9291bfbcf4`.
Requested localization output remains **[X, Y, psi]**. The observer's velocity
and acceleration states are internal states whose stability still matters.

## Answer and scope

A useful guarantee is possible: sufficiently informative, accurate and frequent
LiDAR poses can replace GNSS position corrections in a properly designed
observer. With bounded disturbances the relevant conclusion is a bounded
estimation error, not exact convergence to ground truth. The present production
observer does **not** have that guarantee. Increasing its LiDAR information
alone cannot repair its gain structure over the stated operating envelope.

This work provides a counterexample for the existing observer, derives
directional information and timing conditions, and constructs a numerically
verified LiDAR-only **candidate** using the existing base pose gain and a
smaller invariant gain. It adds research scripts and numerical artifacts; it
does not install a production replacement or claim that the previous complete
Mississippi experiment is now stable.

The candidate admits qualified pose intervals in **[80, 110] ms**, with 30 ms
correction pulses, and arbitrary model variation within the specified box.
This is a verified sufficient interval, not a necessary minimum data rate or
the maximum feasible intermeasurement interval. The previously observed
471.4 ms accepted-pose gap is outside this certificate.

## 1. What the present LMI actually establishes

Write the internal state and scaling as

\[
z=[X,V_x,A_x,Y,V_y,A_y,\psi]^T,\qquad
T=\operatorname{diag}(\theta^{[1,2,3,1,2,3,1]}),\quad\theta=3.5.
\]

Let `Cb` select X, Y and heading, and `Cl` select X and Y. In normalized error
coordinates epsilon = T^{-1}(hat(z)-z), each homogeneous error matrix has form

\[
\dot\epsilon=\theta\{A+F_\theta+NH_\theta
 -K\Omega C_b-P^{-1}C_l^TW C_l\}\epsilon.
\]

The sign of the invariant Jacobian is absorbed by its symmetric mean-value
box. There are 8192 output-Jacobian vertices, four known-input vertices and
two heading-weight endpoints. The recorded operating bounds are componentwise
velocity 16 m/s, acceleration 5 m/s² and track-angle rate 0.6 rad/s. They must
cover the true and estimated states, not only the recorded true trajectory.

The current certificate always sets the GNSS XY entries of Omega to one.
Heading weight is in [0.15,1]. Exhaustive verification of that certificate
gives a largest Schur-complement eigenvalue of **-0.05605069**. Removing GNSS
and setting LiDAR XY weight to its maximum, W=I, gives **+1.03117724** instead.
The latter invalidates this certificate; infeasibility alone would not prove
that every trajectory is unstable.

A separate physical counterexample supplies that stronger diagnostic evidence.
Consider straight travel at 16 m/s, heading -135 degrees, zero acceleration,
zero sideslip and zero yaw rate. Both global velocity components are -11.3137
m/s, inside the box. Give the observer continuous, perfect, zero-delay LiDAR
XY and heading with unit weights, and remove GNSS. The exact zero-error
trajectory is an equilibrium in moving error coordinates. Its seven-state
Jacobian has spectral abscissa **+0.030570033 /s**. Thus it is locally unstable.
Finite differences of the production channel evaluator at steps 1e-4, 1e-5
and 1e-6 reproduce the Jacobian with maximum Frobenius discrepancy 1.10e-9;
the equilibrium residual is zero. This is not a claim that every such initial
error runs away, but it disproves uniform local asymptotic stability for the
unchanged observer even with ideal continuous LiDAR.

For comparison, the actual-theta homogeneous calculation with continuously
active GNSS XY **and heading weight at least 0.15** gives
dot(V) <= -1.49310856 V. Under bounded perturbations and an invariant operating
region this supports an ISS conclusion. A receiver continuously producing
5 Hz fixes is not continuously active correction in the existing 30 ms pulse
implementation. GNSS XY by itself also cannot uniformly determine heading
at standstill. The GPS-available branch therefore needs its actual sampled
schedule and heading observability included, rather than equating signal
availability with the continuous LMI's hypotheses.

## 2. A computable threshold for the existing gain structure

For any chosen model vertex, let M denote the symmetric Lyapunov matrix with
GNSS XY absent, before LiDAR XY damping. Partition its position indices
a=[1,4] and other indices u=[2,3,5,6,7]. When M_uu is negative definite,

\[
M-2C_l^TWC_l\prec0
\quad\Longleftrightarrow\quad
W\succ\Gamma_v,
\qquad
\Gamma_v=\tfrac12(M_{aa}-M_{au}M_{uu}^{-1}M_{ua}).
\]

The condition must hold for **every** vertex. For an isotropic effective
weight cI it becomes c > max_v lambda_max(Gamma_v). The exact fixed-P
thresholds, not universal physical information thresholds, are:

| Certificate budget | Required effective weight c |
|---|---:|
| Original sigma=3, retained decay and noise Schur budget | >15.83951461 |
| Actual theta=3.5, homogeneous flow only | >3.693378779 |

The uncorrected hidden block is negative in both calculations. Production
weights satisfy W=Hxy(Hxy+25I)^{-1} <= I, so neither threshold can be met.
The numbers 15.84 and 3.69 are weights, **not** raw D2D information eigenvalues.

If the auxiliary LiDAR gain were multiplied by rho, the homogeneous
isotropic sufficient condition would be

\[
\rho>c_* ,\qquad
\lambda_{\min}(H_{xy})>\frac{25c_*}{\rho-c_*},
\quad c_*=3.693378779.
\]

Its directional form is rho W > Gamma_v for every v. Provided
rho I > Gamma_v, the equivalent raw-information inequality is

\[
H_{xy}\succ25\{(I-\Gamma_v/\rho)^{-1}-I\}.
\]

These statements concern the matrix used by the current split XY channel;
they do not validate that matrix as a calibrated inverse covariance. Merely
scaling this gain is also unattractive for the present pulse schedule. The
same P gives an all-pose-off growth bound dot(V)<=12.23636301 V. Effective
XY weight 4 gives on-mode dot(V)<=-0.16503294 V, requiring a good-mode time
fraction above 0.98669 by this common-P argument. At weight 128 that bound
still requires 0.89082, far above a nominal 30 ms / 100 ms duty fraction.
These are conservative sufficient bounds, not evidence that all 10 Hz
observers are impossible. They motivate a timer-dependent certificate.

## 3. What information the matcher must supply

### Geometry and direction

Each accepted match should provide timestamp, pose [X,Y,psi], and the full
symmetric 3x3 pose information matrix in the same coordinates and units.
Translation/heading cross terms matter. Large trace, many matches, or a large
strong-direction eigenvalue cannot compensate for an unconstrained direction.
For example, an extended parallel curb can strongly constrain lateral
position while weakly constraining translation along the curb.

Choose explicit position and angular scales D before interpreting eigenvalues
of D^T I D. Meters and radians should not be combined through an undocumented
scalar threshold. When examining translation separately, use the marginalized
information

\[
I_{xy}^{\rm marg}=I_{xy,xy}-I_{xy,\psi}I_{\psi\psi}^{-1}I_{\psi,xy},
\]

when I_psipsi is positive. The raw XY principal block assumes heading is fixed
and can overstate joint pose identifiability.

Full directional information at every admitted pose is an easily enforced
sufficient interface. It is not necessary in every frame. Complementary weak
directions can accumulate over a bounded window. The relevant seven-state
observability quantity then includes dynamics:

\[
\mathcal G(t,T_w)=\sum_{t_k\in[t,t+T_w]}
\Phi(t_k,t)^T C_b^T I_k C_b\Phi(t_k,t)\succeq\alpha I_7,
\]

in fixed, explicitly scaled coordinates. Phi is the local error-model state
transition, not identity. This also tests whether successive positions
constrain internal velocity and acceleration. Such a window condition describes
available information; it does not, by itself, stabilize an arbitrary fixed
gain or constitute a global nonlinear observer theorem. A design exploiting
window observability would need a matching proof.

### Accuracy, including false correspondences

Information curvature alone cannot rule out a confident wrong match. To turn
I_k into a deterministic error bound, require a justified error set such as

\[
\nu_k^T I_k\nu_k\le\eta^2,
\qquad\nu_k=\hat p_k-p(t_k).
\]

The radius eta must cover correspondence errors, map bias and uncertainty,
not just optimizer curvature. It has not been calibrated for the current
composite D2D objective. Assuming Gaussian noise instead yields a probabilistic
statement with a declared confidence level, not an absolute no-divergence
guarantee over an infinite run.

Here is a directional information threshold tied directly to an observer
error budget. If G maps physical pose error into normalized state derivative,
and the desired measurement contribution is at most d_L in the Lyapunov norm,
require

\[
\boxed{I_k\succeq\frac{\eta^2}{d_L^2}G^TP(s)G}
\]

for all relevant pulse phases s and admitted heading weights. Then
||P(s)^(1/2) G nu_k|| <= d_L follows immediately. For the candidate below,
G=K diag(1,1,w_psi), **without** an extra theta: normalizing the physical
injection TK by T cancels that scaling. This uses the full information matrix
and gives an explicit meaning to "enough information" once the permissible
state error and model/input disturbance budgets are specified. The threshold
is not an arbitrary feature count or a fabricated inverse covariance.

This proposed candidate uses information to admit bounded-error poses and
then uses unit XY base gain. Unit gain is a design choice after admission;
it does not mean that finite raw information was converted into W=I by the
old lambda/(lambda+25) formula. The candidate has no unproven attenuation of
that base gain. More sophisticated directional gain scheduling needs its own
robust inequalities.

## 4. A feasible LiDAR-only timing certificate

Keep theta=3.5 and the recorded K. Route qualified LiDAR [X,Y,psi] through
the base pose channel, replace N by **0.1 N**, and omit the separate auxiliary
XY injection in this candidate. No GNSS is present. During each 30 ms pulse,
Omega=diag(1,1,w_psi), with w_psi anywhere in [0.15,1]; outside pulses Omega=0.
Qualified timestamps obey

\[
0.08\le t_{k+1}-t_k\le0.11\quad\text{seconds}.
\]

Define a timer s=t-t_k and a positive definite, piecewise affine P(s), with
knots every 10 ms from 0 to 110 ms. For every model vertex, require

\[
A_v(s)^TP(s)+P(s)A_v(s)+P'(s)+2rP(s)\prec0,
\quad r=0.01\ {\rm s}^{-1},
\]

and at every possible timer reset,

\[
P(0)-P(h)\preceq0,\qquad h\in[0.08,0.11].
\]

The physical observer state is continuous at these resets; only the timer
changes. This proves decay of V=epsilon^T P(s) epsilon during flows and
nonincrease at resets. Endpoint checks cover each whole time segment because
P and P' enter affinely there; model-box vertex checks also cover arbitrary
time variation inside that box. This is not verification only at sampled
timer points.

Recovered matrices in `timer_certificate.json` pass an independent exhaustive
check:

| Quantity | Verified value |
|---|---:|
| Flow inequalities, excluding duplicate unit-XY endpoints | 1,441,792 |
| Largest flow eigenvalue including +2rP | -0.004594126871 |
| Smallest P eigenvalue across timer knots | 0.03635963092 |
| Largest P eigenvalue | 2.420255211 |
| Largest reset eigenvalue | -0.00009999980401 |
| Symmetry error | 0 |

SeDuMi reports **status 4 (numerical difficulties)** during synthesis, not a
clean optimum. The first recovered candidate fails full verification. Adding
its violating vertex yields the second candidate above. Acceptance relies
on the recovered matrices satisfying all strict inequalities with margins
well beyond the verification tolerance; no optimality claim is made. The
committed synthesis function reproduces these results. Restoring N to its
original value while keeping this P yields a largest flow eigenvalue
**+0.8437340033**, a failing negative control.

Exploration was not uniformly successful. A two-endpoint timer/sector ansatz
with the original invariant gain failed for tested auxiliary gain multipliers
8 and 32; the original base-gain, theta=3.5 periodic ansatz became infeasible
after adding violating vertices. A theta=6 attempt encountered numerical
difficulties without an accepted certificate. The reduced-invariant candidate
first passed a fixed 100 ms periodic check, then the aperiodic check above.
Tested attenuated-XY candidates (minimum weight 0.5 over 80–150 ms and 0.8 over
90–110 ms) were infeasible in the chosen synthesis ansatz. None of these
failures establishes a global impossibility result or an optimal threshold.

## 5. From homogeneous decay to a no-runaway guarantee

Let all additive normalized perturbations be d(t), including pose error,
held-measurement age error, upstream lateral-estimation/input error, unmodeled
dynamics and any justified numerical-integration error. Suppose

\[
\|P(s)^{1/2}d(t)\|\le\bar d.
\]

Writing R=sqrt(V), the timer certificate gives during flows

\[
\dot R\le-rR+\bar d,
\qquad
R(t)\le e^{-r(t-t_0)}R(t_0)+\frac{\bar d}{r}
(1-e^{-r(t-t_0)}).
\]

Timer resets cannot increase R. Thus the disturbance-free ideal system is
exponentially stable **within its certified domain**, and bounded disturbances
give ultimate bound bar(d)/r. r=0.01 is a conservative proved rate, not a
measured convergence rate. Held old positions create nonzero age error even
with perfect timestamped poses: at pulse duration h, translation age error is
bounded by speed times h, with an analogous angular-rate bound. These terms
must be budgeted or eliminated by a separately justified output predictor.

The domain condition must not assume the desired result. If the true velocity
and acceleration components remain strictly inside their bounds with margins
Delta_i, define

\[
R_* =\min_{s,i}
\frac{\Delta_i}{T_{ii}\sqrt{[P(s)^{-1}]_{ii}}}.
\]

Also impose a valid local heading-error chart and the track-rate bound. A
strict inequality max(R(t0),bar(d)/r)<R_* supplies a continuation argument:
the bound prevents the estimate leaving the box in which that bound was
derived. Timer-knot endpoints suffice for conservative inverse-diagonal
maxima by matrix convexity of inversion. Without true-state margins,
initial-error bounds and a bounded input budget, this local/region result
cannot honestly be called a global guarantee. The complete lateral-to-global
cascade still needs its upstream error bound; the present experiment uses
nominal, incompletely identified vehicle dynamics.

## 6. Explicit fixed delay

A fixed delivery delay tau does not delete timestamped information. For an
ideal fixed-lag replay realization, all pose events up to t-tau have arrived;
the corrected lag state follows the measurement-time system, and propagation
from that state produces the causal current estimate. This argument assumes
correct time alignment, sufficient history, identical replay dynamics, and
bounded high-rate input/interpolation errors. It does not apply to injecting
an old pose as if it measured the current state.

The candidate's normalized off-mode Euclidean logarithmic norm is bounded by
L_off=2.550844898 /s. A pure prediction interval obeys

\[
\|\epsilon(t)\|\le e^{L_{off}\tau}\|\epsilon(t-\tau)\|
+\bar d_E\frac{e^{L_{off}\tau}-1}{L_{off}}.
\]

At 150 ms the homogeneous factor is 1.46613. In replay which finishes the
last known pose pulse, at most 30 ms of that tail also has pose injection.
The on-mode logarithmic-norm bound is 15.74821562 /s. A conservative
homogeneous tail factor is then

\[
M_\tau=\exp\{L_{off}\tau+
\max(0,L_{on}-L_{off})\min(0.03,\tau)\},
\]

which is **2.17831 at 150 ms**, with additive disturbance bound no larger than
M_tau tau bar(d_E). These are bounds in normalized coordinates, not meters of
position error. Use the effective delay including timestamp/grid allowance;
150 ms computation does not by itself certify a 10 ms discretized runtime.

Finite fixed delay therefore preserves boundedness if the lag-state bound,
tail propagation and disturbances all stay in the certified region. It
increases the required error-budget margin. This derivation supplies the
appropriate delay condition; it is not a new end-to-end numerical proof of
the existing replay implementation or its RK4 discretization.

## 7. Consequences for this project

The desired contract is: **either an appropriately certified GNSS/heading
mode is available, or LiDAR delivers correctly associated, directionally
informative poses within a certified maximum gap; the known delay and all
perturbations fit the invariant-region budget.** The same observer and its
switching law must be certified for whichever alternatives it actually uses.

The current numerical counterexample rules out claiming this for the existing
gain routing merely from stronger LiDAR information. The candidate shows
that the architecture can admit such a guarantee at a practical nominal
10 Hz rate. Its full-pose gain, invariant gain, timing, calibrated error set
and region assumptions must be treated together. It does not cover the
recorded 471.4 ms gap, the full GNSS/LiDAR switching schedule, or unbounded
matching/input errors. No calibrated raw-information cutoff or maximum
permissible position error is claimed before those budgets are established.

## Reproduction and validation

Run from the repository root in MATLAB; no bag is required for these audits:

```matlab
setupVehicleLocalization;
out = "output/lidar_information_conditions_reproduced";
metrics = analyzeLidarInformationConditions("", out);
check = verifyLidarOnlyTimerCertificate;
assert(check.passed);
% Optional synthesis: first put YALMIP and SeDuMi on the MATLAB path.
candidate = designLidarOnlyTimerCertificate;
assert(candidate.passed);
```

Small input/output exports are in
`research/results/lidar_information_conditions_20260907/`. They contain the
recorded P/K/N/X design, numerical audit metrics, rates, finite-difference
checks, candidate P(s), verification and the original-invariant negative
control. Raw exploratory MAT files remain under
`output/lidar_information_condition_20260907/` and are not published.

Executed validation: full baseline-box audit; physical counterexample and
three finite-difference steps; exhaustive candidate flow/reset verification;
reusable synthesis with post-solver verification; original-invariant negative
control; factory MATLAB Code Analyzer for all three new scripts. No production
algorithm changed and no new full dataset replay is claimed. The unrelated
observer directory relocation and instruction/draft/reference edits are
outside this work's commit.

## Research basis and review

- Hichem Bessafa, Cédric Delattre, Zehor Belkhatir, Ali Zemouche and Rajesh
  Rajamani, *Generalized multi-output high-gain observer with application to
  ego vehicle trajectory and orientation estimation*, Automatica 188 (2026),
  112915, [DOI](https://doi.org/10.1016/j.automatica.2026.112915). Examined the
  local PDF's operating-region/Lipschitz discussion, Theorem 5 and its ISS
  derivation. Its continuous-output theorem does not automatically cover
  this repository's GNSS-off and pulse-gap switching. The paper explicitly
  identifies sampled/delayed outputs as extensions; experimental loss-of-signal
  results are not a replacement for those mode inequalities.
- Francesco Ferrante and Alexandre Seuret, *Observer Design for Linear
  Aperiodic Sampled-Data Systems: A Hybrid Systems Approach*, IEEE Control
  Systems Letters 6 (2022), 470–475,
  [DOI](https://doi.org/10.1109/LCSYS.2021.3081345),
  [full preprint](https://arxiv.org/pdf/2102.10652). Read the sampling bounds,
  hybrid formulation and timer-dependent Lyapunov results. These motivate
  explicit flow/reset and maximum-gap conditions. Our pulse observer and
  nonlinear box are different; the inequalities above are derived directly,
  rather than presented as a verbatim application of that linear theorem.
- Daoyuan Zhang and Yanjun Shen, *Continuous Sampled-Data Observer Design for
  Nonlinear Systems With Time Delay Larger or Smaller Than the Sampling
  Period*, IEEE Transactions on Automatic Control 62(11) (2017), 5822–5829,
  [DOI](https://doi.org/10.1109/TAC.2016.2638043). Publisher metadata and abstract
  were checked; the full proof was not reviewed. It supports considering
  sampling and delay explicitly, but is not used to certify our replay.

Internal challenge checks were performed inline: scope distinguished available
information from a stabilizing estimator; analysis retained failing candidates
and separated LMI infeasibility from physical instability; final review checked
the solver's numerical warning, region/noise/delay qualifications, and the
gap between this candidate and the actual recorded pipeline. All new numerical
claims have executable checks. These are computational research results and
conditional derivations, not independent peer review or a shipped repair.
