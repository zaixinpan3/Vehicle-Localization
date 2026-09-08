# Assimilating the observer proposal into the existing architecture

Date: 2026-09-08. Inspected baseline:
1a1a202dd0a6c8d24e5ecf93d6afb1de262440c4.

The supplied proposal is a theoretical reference, not the runtime
specification. Its useful contributions are explicit observability arguments,
physical disturbance accounting, and a checkable ISS proof structure. The
existing seven-state cascade, full information gains, exact LPV drift
enclosure, aperiodic pulses and acquisition-time replay remain the engineering
baseline. This note gives the coefficient bounds and an ISS argument for that
baseline, rather than transferring the proposal's different observer theorem.

## Decisions and implementation

| Proposal element | Decision for the current code |
| --- | --- |
| Independent yaw, nonsingular four-output map | Already implemented; retain and explain the information sources below. |
| Nominal speed jerk and course angular acceleration inputs | Keep these as explicit residuals. No new numerical differentiation or sensor interface is required. |
| Static bicycle sideslip | Retain the independent dynamic/stationary/crawl lateral observer and its sideslip-rate interface. |
| Full-matrix information and preserved nullspace | Retain the existing spectral weight and separate GNSS/LiDAR residuals. Fix automatic initialization, which previously copied unobserved coordinates from a partial-rank pose. |
| Uniform continuous pose anchoring | Replace this hypothesis with the existing pulse-sector and inter-pulse timing hypotheses. Prediction intervals intentionally have no pose correction. |
| Fixed-delay, acquisition-innovation feedback held at arrival | Retain physical-time pulse replay, variable/out-of-order arrivals and causal public outputs. Their error equation is different. |
| Exact incremental output polytope | Make all 13 coefficient bounds and the four drift vertices explicit below. |
| Scalar drift Lipschitz penalty and prescribed matched gain | Retain the existing full-LPV timer LMIs and verified gains. Their anisotropic sector proof does not require a new matched gain. |
| Curb observability and weighted measurement error | Adopt, with local-chart and nuisance-position qualifications. Add marginalized heading information and motion sensitivity diagnostics. |
| Unified ISS and gain/delay tradeoff | Derive a conditional timer ISS bound and a separate bounded replay-tail bound below. Do not claim that the recorded drive satisfies all hypotheses. |

Automatic initialization now selects the latest **physical** timestamp among
already arrived events. Full-rank initialization and GNSS position precedence
are preserved. For partial-rank LiDAR, its existing bounded physical weight is
applied to the local residual relative to the fallback/GNSS prior; GNSS
position, when present, remains the position initializer. Changing the LiDAR
representative along its information nullspace cannot change this correction
on the same yaw branch. The unobserved initial error is still part of the
initial-error budget; a fallback is not a measurement.

The information audit now checks the actual GNSS/LiDAR combined weight on
every constant-weight segment of each settled LiDAR pulse, including GNSS
starts and expirations between high-rate samples. Its
informationWithinCertificate field concerns these combined weights.
lidarInformationWithinCertificate and the original LiDAR eigenvalue/count
fields retain the separate LiDAR-only assessment. Prediction intervals are
excluded from this sector audit and handled by the timer certificate.
Timing, delay, chart and disturbance hypotheses remain separate.

## Independent yaw and the physical outputs

The implemented order is
\[
z=[X,V_X,A_X,Y,V_Y,A_Y,\psi]^T,
\]
a permutation of the proposal's \([p^T,v^T,a^T,\psi]^T\).
Let \(J_2=[0,-1;1,0]\), \(t(\chi)=[\cos\chi,\sin\chi]^T\),
\(n(\chi)=J_2t(\chi)\), and \(\chi=\psi+\beta\). The outputs are
\[
h(z,\beta_m)=
\begin{bmatrix}
\|v\|^2\\v^Ta\\v\times a\\n(\psi+\beta_m)^Tv
\end{bmatrix}.
\]
Only the first three are rotational invariants of \(v,a\).
The fourth is a yaw--sideslip--velocity consistency channel. The legacy
invariant names in the API group these four outputs for compatibility;
they do not assert that all four are rotation invariants of \(v,a\).

For compensated body inertial acceleration \(a_b^m\), the measured side is
\[
y_h=
\begin{bmatrix}
(v_x^m)^2+(\hat v_y^b)^2\\
v_x^m a_x^m+\hat v_y^b a_y^m\\
v_x^m a_y^m-\hat v_y^b a_x^m\\0
\end{bmatrix}.
\]
The longitudinal speed comes from the high-rate sensor stream and lateral
velocity from the independent upstream observer. The global observer does
not construct both sides from its own global velocity/acceleration estimate.
The lateral estimate is nevertheless **not an independent sensor**:
its errors and its use of common IMU inputs enter the output-error budget.
Statistical independence is unnecessary.

In particular, if the body-velocity error is \(\delta v_b\) and acceleration
error is \(\delta a_b\), sufficient component bounds are
\[
|n_{h,1}|\le2\|v_b\|\|\delta v_b\|+\|\delta v_b\|^2,
\]
\[
|n_{h,2}|,\ |n_{h,3}|
\le\|v_b\|\|\delta a_b\|+\|a_b\|\|\delta v_b\|
 +\|\delta v_b\|\|\delta a_b\|,
\qquad
|n_{h,4}|\le\|v\||\beta_m-\beta|+b_{\rm roll}.
\]
The last term bounds violation of the rolling-direction relation.
Acceleration errors include gravity/attitude, reference-point lever arm,
extrinsics, bias and unmodeled dynamics; body velocity derivatives or raw
specific force cannot silently substitute for inertial acceleration.

For any common planar rotation of \(v,a\), the first three outputs are
unchanged. Moreover,
\[
\partial_\psi h_4=-t(\psi+\beta_m)^Tv.
\]
At a stationary equilibrium \(v=a=0\), this is zero. Changing yaw while
keeping stationary position fixed leaves GNSS position and all four outputs
unchanged. Independent yaw therefore permits geometric correction at rest
without implying that motion channels observe stationary absolute heading.
The new motionHeadingSensitivity diagnostic evaluates this derivative at
the extended estimated state and uses the revised-history time basis; it is
not a truth-based observability certificate.

This gives a suitable paper motivation: motion constraints describe
rotation-invariant kinematics and the alignment of body orientation with
velocity, while the environment supplies an absolute orientation reference.
Independent yaw allows these complementary sources to remain separate when
velocity direction is undefined.

## Division-free drift and an explicit disturbance model

With physical speed \(s\), course rate \(\omega\), and \(\chi=\psi+\beta\),
\[
v=st,\quad a=\dot s\,t+s\omega n,\quad
v^Ta=s\dot s,\quad v\times a=s^2\omega,
\]
\[
\dot a=(\omega^2I+\dot\omega J_2)v+2\omega J_2a+\ddot s\,t.
\]
Direct differentiation establishes these identities, including acceleration,
braking and changing steering. The nominal implementation uses
\[
q=r_m+\dot\beta_m,\qquad f_0(v,a,q)=q^2v+2qJ_2a,\qquad\dot{\hat\psi}=r_m
\]
before corrections. Its physical plant representation is exact if
\[
w_j=\ddot s\,t+\dot\omega J_2v+
(\omega^2-q^2)v+2(\omega-q)J_2a+w_{\rm extra},
\qquad w_\psi=r-r_m
\]
are included. It does not set \(\dot s,\ddot s,\dot\omega\) to zero.
For vector-norm bounds \(V_b,A_b\), \(|q|\le Q\),
\(|\omega-q|\le\Delta_q\), \(|\ddot s|\le J_s\),
\(|\dot\omega|\le A_\chi\), a checkable bound is
\[
\|w_j\|\le J_s+A_\chi V_b+
\Delta_q(2Q+\Delta_q)V_b+2\Delta_q A_b+\bar w_{\rm extra}.
\]
The configured limits are **component** limits, so one may take
\(V_b=\sqrt2\,16\ {\rm m/s}\), \(A_b=\sqrt2\,5\ {\rm m/s^2}\).
These are physical operating hypotheses, not established by clipping the
estimated nonlinear outputs. \(J_s,A_\chi,\Delta_q\) require justified bounds;
no values are inferred from innovation magnitude or a successful simulation.

At stops where course itself has no smooth continuation, use the direct
Cartesian residual \(w_j=\dot a-f_0(v,a,q)\). Bounded Cartesian jerk, \(v,a,q\)
suffice without assigning an observable course angle at zero speed.
The signed-speed algebra in the proposal does not establish reverse-driving
support for the existing lateral model: its public longitudinal-speed input
remains nonnegative. Extending reverse motion requires its own model and
validation, not removal of that input check.

## Exact incremental coefficients and vertex families

The runtime clips only the velocity and acceleration arguments of \(h\) to
\([-V,V]\) and \([-A_b^{\rm comp},A_b^{\rm comp}]\).
This continuous piecewise-smooth map is globally Lipschitz on a yaw lift.
Its generalized derivatives include clip slopes in \([0,1]\).
An integral along a segment, or coordinate-by-coordinate telescoping secants,
therefore gives the **exact** relation
\[
h(z,\beta_m)-h(\hat z,\beta_m)=H(z-\hat z).
\]
This is not a first-order Taylor approximation. A smooth saturation is
unnecessary: absolute continuity and bounded generalized derivatives suffice.
The extension supplies incremental bounds; it does not repair observability.

On the unclipped region, writing \(u=\psi+\beta_m\), the nonzero entries are
listed below. The bounds remain valid for the clipped extension.
All omitted entries, including both position columns, are zero.
Let
\[
T_\theta=\operatorname{diag}(\theta,\theta^2,\theta^3,
 \theta,\theta^2,\theta^3,\theta),\qquad
H_\theta=\theta^{-4}HT_\theta .
\]
This is the implementation's scaling, including its common extra factor
\(\theta\) relative to the proposal's permuted \(D_\theta\).

| Row, column | Unclipped coefficient | Absolute bound | Bound in \(H_\sigma\) |
| --- | --- | --- | --- |
| 1,2 | \(2V_X\) | \(2V\) | \(2V/\sigma^2\) |
| 1,5 | \(2V_Y\) | \(2V\) | \(2V/\sigma^2\) |
| 2,2 | \(A_X\) | \(A_b^{\rm comp}\) | \(A_b^{\rm comp}/\sigma^2\) |
| 2,5 | \(A_Y\) | \(A_b^{\rm comp}\) | \(A_b^{\rm comp}/\sigma^2\) |
| 2,3 | \(V_X\) | \(V\) | \(V/\sigma\) |
| 2,6 | \(V_Y\) | \(V\) | \(V/\sigma\) |
| 3,2 | \(A_Y\) | \(A_b^{\rm comp}\) | \(A_b^{\rm comp}/\sigma^2\) |
| 3,5 | \(-A_X\) | \(A_b^{\rm comp}\) | \(A_b^{\rm comp}/\sigma^2\) |
| 3,6 | \(V_X\) | \(V\) | \(V/\sigma\) |
| 3,3 | \(-V_Y\) | \(V\) | \(V/\sigma\) |
| 4,2 | \(-\sin u\) | \(1\) | \(1/\sigma^2\) |
| 4,5 | \(\cos u\) | \(1\) | \(1/\sigma^2\) |
| 4,7 | \(-V_X\cos u-V_Y\sin u\) | \(2V\) | \(2V/\sigma^3\) |

At \(V=16,A_b^{\rm comp}=5,\sigma=3.5\), these five distinct bounds are
\(2.612244898,\ 0.4081632653,\ 4.571428571,\ 0.08163265306,\
0.7463556851\), respectively for \(2V/\sigma^2,A_b^{\rm comp}/\sigma^2,
V/\sigma,1/\sigma^2,2V/\sigma^3\).
The \(2V\) yaw bound is conservative; \(\sqrt2V\) is also sufficient, but
the existing verified box is retained.

For each of the 13 entries independently choose either signed endpoint.
This gives \(2^{13}=8192\) vertices and their convex hull. It encloses
correlated physical coefficients, including zero, without asserting those
coefficients are independent physical variables. For \(\theta\ge\sigma\),
the output box nests inside the one at \(\sigma\).
That nesting alone does **not** authorize changing the fixed-\(\theta\)
timer certificate.

Let \(B\) insert two jerk components into rows 3 and 6. Define \(F(q,\rho)\)
with only
\[
F_{1,2}=\rho,\quad F_{1,6}=-2q,\quad
F_{2,5}=\rho,\quad F_{2,3}=2q .
\]
The scaled drift family is
\[
\mathcal F_\sigma=\sigma^{-4}BF(q,\rho)T_\sigma,\qquad
q\in\{-Q,Q\},\quad\rho\in\{0,Q^2\}.
\]
Its four vertices enclose the exact nominal drift with \(\rho=q^2\).
The physical scaled drift contribution is \(\theta\mathcal F_\theta\).
No scalar Lipschitz subtraction is needed. Together there are 32,768
output/drift pairs. With the shipped 11 timer segments and two endpoints,
the sector verifier checks 720,896 flow inequalities plus reset conditions.
The old omegaVertices in the generic data builder belong to the historical
diagonal certificate, not the production full-sector test.

For \(\epsilon=T_\theta^{-1}(z-\hat z)\), the auxiliary feedback contributes
\(-\theta NH_\theta\epsilon\). The verifier writes a plus sign and enumerates
the symmetric box; replacing every \(H_v\) by \(-H_v\) leaves the family
unchanged. This is a bounding-set convention, not a reversed innovation.
The connection to Bessafa is precisely the incremental representation,
block scaling and finite enclosing polytope, not reuse of its vehicle theorem.

## Geometric information, curb proposition and weighted errors

For the fixed normalization \(D=\operatorname{diag}(\text{poseScales})\),
write \(J_L=D\mathcal I_LD\), \(s_I=\text{gainInformationScale}>0\) and
\[
W_L=(s_II+J_L)^{-1}J_L.
\]
Thus \(0\preceq W_L\preceq I\), eigenvectors and cross terms are retained,
and \(\ker W_L=\ker J_L\). The physical map \(DW_LD^{-1}\) annihilates
\(\ker\mathcal I_L\). No positive information floor is introduced.
Tiny negative eigenvalues are only removed within the existing numerical
roundoff tolerance. Gains in physical coordinates need not be symmetric.

**Proposition (local straight-curb information).** For positive residual
weights \(w_i\), curb normal \(e_Y\), and longitudinal lever arms \(\xi_i\),
\[
\mathcal I_L=
\begin{bmatrix}
0&0&0\\0&S_0&S_1\\0&S_1&S_2
\end{bmatrix},\quad
S_0=\sum_iw_i,\quad S_1=\sum_iw_i\xi_i,\quad S_2=\sum_iw_i\xi_i^2 .
\]
The longitudinal direction is unobservable. Lateral position and yaw are
jointly locally identifiable exactly when
\(S_0S_2-S_1^2>0\), and the heading information after eliminating lateral
position is
\[
\mathcal I_{\psi\mid Y}=S_2-S_1^2/S_0
=\sum_iw_i(\xi_i-\bar\xi_w)^2 .
\]
**Proof.** Summing \(w_i[0,1,\xi_i]^T[0,1,\xi_i]\) gives the matrix.
Its nonzero block is positive definite exactly when its determinant is
positive. Minimizing its quadratic form over the lateral perturbation gives
the stated Schur complement. There is no vehicle speed in this calculation.

When every lever arm is equal, the yaw diagonal can be positive yet the
joint block is singular: lateral translation and rotation are confounded.
Thus lateral position is directly sensitive with yaw fixed, but independent
lateral and yaw identifiability requires spread (or another anchor).
This qualification avoids overstating the single-point curb case.
The new marginalizedHeadingInformation diagnostic generalizes the Schur
complement using the translation-block pseudoinverse, without changing gains.
Known GNSS position can additionally resolve geometric translation/yaw
coupling; the full combined matrix is the relevant object in that case.

On a consistent local yaw chart, assume only
\[
W_LD^{-1}y_L=W_LD^{-1}Cz(t_k)+\epsilon_L .
\]
No raw error bound is needed in an exact LiDAR nullspace. Measurements still
must have finite numeric representatives for the implementation, and a large
coupled yaw/translation displacement cannot leave the valid angle branch.
For a pulse at time \(t\), define the hold term
\[
b_L(t)=W_LD^{-1}C[z(t_k)-z(t)] .
\]
The runtime residual then contains \(\epsilon_L+b_L\), not just
\(\epsilon_L\). For duration \(h_L\), a sufficient bound is
\[
\|b_L\|\le h_L
\sqrt{2V^2/\min(D_{11},D_{22})^2+\bar r^2/D_{33}^2}.
\]
GNSS similarly has a physical position hold term \(b_G\) bounded by
\(\sqrt2Vh_G\). These bounds apply on the pulse's acquisition-time axis.
Replay removes arrival-time misalignment but does not make a held pose track
the vehicle during its 30 ms pulse.

The combined GNSS/LiDAR implementation also preserves the weighted model.
With \(E_G=[I_2;0]\), \(G=D E_G I_G E_G^TD\), and
\(S=s_II+J_L+\gamma G\), define
\[
A_L=S^{-1}(s_II+J_L),\qquad A_G=S^{-1}\gamma G .
\]
The LiDAR weighted error enters as \(D A_L(\epsilon_L+b_L)\);
GNSS enters as \(D A_GD^{-1}E_G(n_G+b_G)\).
This follows from \(S^{-1}J_L=A_LW_L\).
Individual fused weights need not be symmetric or share LiDAR's eigenvectors;
the total \(S^{-1}(J_L+\gamma G)\) is symmetric and a contraction.
The LiDAR injection still annihilates its own nullspace.
Also \(\|A_L\|\le1+\|G\|/s_I\), \(\|A_G\|\le\|G\|/s_I\);
bounded weighted error is not invalidated by arbitrarily strong \(J_L\).

The upstream D2D exporter continues to require full-rank accepted poses.
Constructed curb/partial-rank observer tests do not establish a partial-rank
D2D matching mode. Point-to-line geometry here is an explanatory local
example, not a replacement for the repository's Gaussian D2D information
construction or a claim that ICP covariance formulas calibrate D2D errors.

## Conditional ISS for the retained pulse and replay architecture

The following is a project-specific sufficient result for the ideal
measurement-time observer. Assume:

1. The true operating bounds, compensated sensor model, and valid common
   yaw/registration chart above hold. The extended output is used at the
   estimate. All disturbance components below are essentially bounded.
2. LiDAR pulses have the verified 30 ms duration, their physical start
   intervals lie in 50--110 ms, and their **total normalized** weight lies
   in \(0.8I\preceq W\preceq I\) throughout each pulse. Between pulses,
   the certified prediction mode applies. Initial time is a valid timer
   reset; a preceding startup interval requires a separate finite bound.
3. The stored full-sector flow and timer-reset inequalities hold at the
   actual \(\theta=\sigma=3.5\). Write their metric bounds as
   \(p_-I\preceq P(\zeta)\preceq p_+I\) and norm decay rate \(\rho>0\).

GNSS can meet condition 2 jointly with rank-deficient LiDAR. If GNSS is absent
through a pulse, LiDAR must itself meet the sufficient sector. A stationary
curb-only scene without GNSS cannot guarantee convergence of along-curb
position; no proof parameter repairs that indistinguishability.
Condition 2 is sufficient and conservative, not a necessary observability
criterion or an online rejection rule.

Define \(E=[B,b_\psi]\) and the explicit stack
\[
\nu=[w^T,n_G^T,n_h^T,\epsilon_L^T,b_G^T,b_L^T,d_{\rm num}^T]^T,\quad
w=[w_j^T,w_\psi]^T.
\]
Here \(d_{\rm num}\) is an additive error in the scaled dynamics. Treating
RK4, interpolation and interface discretization this way requires a
justified defect bound; finite numerical output alone does not supply one.
In the local lift, the exact scaled error equation is
\[
\dot\epsilon=\mathcal A(t)\epsilon+\mathcal G(t)\nu .
\]
During a LiDAR pulse its disturbance part is
\[
T_\theta^{-1}Ew-\theta^{-3}Nn_h
-KD A_L(\epsilon_L+b_L)
-KD A_GD^{-1}E_G(n_G+b_G)+d_{\rm num}.
\]
Unavailable channels have zero matrices. In a GNSS-only continuation,
use \(A_L=0\), \(A_G=(s_II+G)^{-1}G\).
This continuation is relevant to the finite online tail; persistent
GNSS-only operation is outside condition 2.

Every upstream error now has an explicit entry point. For finite coefficients
let
\[
g_P=\sup_t\|P(\zeta(t))^{1/2}\mathcal G(t)\|_2,\qquad
g_E=\sup_t\|\mathcal G(t)\|_2 .
\]
These are computably bounded without a minimum LiDAR eigenvalue.
For example, with \(g_0=\|G\|/s_I\), \(R_G=D^{-1}E_G\), a deliberately
conservative bound, in the declared fixed coordinate units, is
\[
g_E\le\left[
\theta^{-2}+\theta^{-6}\|N\|^2+
2\|KD\|^2\{(1+g_0)^2+g_0^2\|R_G\|^2\}+1
\right]^{1/2},\qquad g_P\le\sqrt{p_+}\,g_E .
\]
The factors of two account for separate acquisition-error and hold-error
blocks. For specific matrices and sensor budgets, direct block-norm
calculation is sharper. Correlations do not affect these deterministic
induced-norm inequalities.

**Proof.** For \(V=\epsilon^TP(\zeta)\epsilon\), the verified flow LMIs give
\[
\dot V\le-2\rho V+2\epsilon^TP\mathcal G\nu .
\]
The metric reset is nonincreasing, while the ideal observer state is
continuous at pulse boundaries. With \(R=\sqrt V\), the upper Dini
derivative satisfies \(D^+R\le-\rho R+g_P\|\nu\|\), including \(R=0\).
Scalar comparison, including nonincreasing resets, yields
\[
R(t)\le e^{-\rho t}R(0)+
\frac{g_P}{\rho}(1-e^{-\rho t})\|\nu\|_{[0,t],\infty}.
\]
Since \(\|T_\theta\|=\theta^3\), \(\|T_\theta^{-1}\|=\theta^{-1}\),
\[
\boxed{\|z(t)-\hat z_{\rm revised}(t)\|
\le M_\theta e^{-\rho t}E_0+
\Gamma_\theta\|\nu\|_{[0,t],\infty}},
\]
\[
M_\theta=\theta^2\sqrt{p_+/p_-},\qquad
\Gamma_\theta=\frac{\theta^3g_P}{\rho\sqrt{p_-}},\qquad
E_0=\|z(0)-\hat z(0)\|.
\]
This proves the conditional ISS statement for the retained timer model.
It is local for physical registration and wrapped yaw. A first-exit
argument additionally needs the resulting error bound, in consistent chart
units, to remain strictly inside the uniform valid-chart radius.

For causal outputs, suppose replay is consistent and **all relevant
accepted events** through \(t-\bar\tau\) have been incorporated at output
time \(t\). The bound \(\bar\tau\) includes variable delivery delay,
high-rate polling and replay completion, with sufficient history retained.
Let \(L\ge0\) bound the Euclidean logarithmic norm of every possible
scaled tail mode, including prediction, partial-information pulses and
GNSS-only continuation. The existing verifier supplies such a conservative
bound for \(0\preceq W\preceq I\).
Writing \(\Phi(L,\tau)=(e^{L\tau}-1)/L\), with \(\Phi(0,\tau)=\tau\),
\[
\|\epsilon_{\rm online}(t)\|\le
e^{L\bar\tau}\|\epsilon_{\rm revised}(t-\bar\tau)\|
+g_E\Phi(L,\bar\tau)\|\nu\|_{[0,t],\infty}.
\]
For \(t\ge\bar\tau\), composition gives
\[
M_{\rm online}=M_\theta e^{(L+\rho)\bar\tau},\qquad
\Gamma_{\rm online}=\theta^3\left[
\frac{e^{L\bar\tau}g_P}{\rho\sqrt{p_-}}+
g_E\Phi(L,\bar\tau)\right].
\]
The initial finite tail has the corresponding Gronwall bound.
This composition permits bounded variable/out-of-order delivery; it is
not the proposal's delay-differential observer with a frozen acquisition
innovation applied continuously after arrival.
The existing fixed-delay diagnostic remains explicit about its narrower
assumption and does not certify arbitrary delivery schedules.

Increasing high gain can improve nominal contraction but also amplify sensor
errors, acceleration transients and tail growth. In the proposal's different
held-innovation observer it also tightens the small-gain delay condition.
For replay, finite-tail amplification and available history are the relevant
limitations; the proposal's latency formula is not a runtime limit here.
The stored timer certificate must be reverified for any gain change.

## Evidence and remaining scope

The production gains, timer metric, vertex families, normalization, lateral
observer and pulse/replay semantics are retained. The tests exercise partial
and coupled-nullspace initialization, GNSS precedence, physical timestamps,
curb Schur complements, standstill yaw, exact pulse-sector auditing,
out-of-order replay, variable-motion residual identities, and exact
incremental coefficients across clipping. Existing certificate and tracking
tests remain applicable. Numerical results are recorded in
[the validation exports](results/observer_proposal_assimilation_20260908/).

The final MATLAB R2026a Update 3 run passed **66 tests**: 45 global-observer,
14 lateral-observer and seven registration-information tests. Twelve global
cases were added. The unchanged lateral synthesis test was filtered because
YALMIP/SDP tools were absent from the session path; global gain synthesis was
deliberately excluded because gains and certificate matrices did not change.
The stored certificate was exhaustively reverified:
maximum flow eigenvalue \(-0.007437670568\), maximum reset eigenvalue
\(-9.999978859\,10^{-5}\), and metric eigenvalue bounds
\([0.03969819301,2.76499452534]\). All four changed MATLAB files have zero
factory Code Analyzer findings. The default synthetic scenario uses seed
2026; no new recorded-drive or perception replay was performed.

An initial test incorrectly demanded zero motion-heading sensitivity along
every geometrically corrected estimate of a stationary vehicle. Small
velocity transients from cross-coupled gains violate that assertion.
The corrected test checks the actual stationary state and the GNSS-only
stationary trajectory, and separately verifies geometric yaw-error reduction.
No gain or tolerance was weakened.

For these matrices \(M_\theta=102.2345894\), \(\rho=0.01\ {\rm s^{-1}}\),
and the existing all-mode tail bound is \(L=44.18733530\ {\rm s^{-1}}\).
Those conservative constants do not provide a tight deployment error budget.

Reproduce from the repository root:

~~~matlab
setupVehicleLocalization;
suite = testsuite('tests/improvedObserverTest.m');
suite = suite(~contains({suite.Name},'synthesisProduces'));
suite = [suite, testsuite('tests/lateralObserverTest.m'), ...
    testsuite('tests/registrationInformationTest.m')];
results = run(suite);
assertSuccess(results); % inspect Incomplete as well as Failed
design = improvedObserverReferenceDesign(improvedObserverConfig);
data = buildImprovedObserverCertificateData(improvedObserverConfig);
~~~

The original Bessafa PDF in the local reference directory was read, especially
Sections 3--4, Eqs. (29)--(38), Theorem 5, and the variable-motion discussion
in Section 5.3. Its six-state vehicle transformation reconstructs yaw using
velocity components and employs restrictive nominal identities. Its later
experiments do include varying speed/steering and gyro measurements; it would
be inaccurate to claim those subjects are wholly absent.
The paper is *Generalized multi-output high-gain observer with application to
ego vehicle trajectory and orientation estimation*, Hichem Bessafa, Cedric
Delattre, Zehor Belkhatir, Ali Zemouche and Rajesh Rajamani, Automatica 188
(2026), 112915. Metadata was checked against the
[author institution record](https://experts.umn.edu/en/publications/generalized-multi-output-high-gain-observer-with-application-to-e/);
the DOI resolver was unavailable in this session. No reference PDF is
redistributed.

The geometric interpretation is consistent with Bonnabel, Barczyk and
Goulette's warning that rematching and the residual model matter for
[ICP covariance](https://arxiv.org/html/1410.7632v3), and with Tuna et al.'s
directional treatment of weak registration geometry in
[X-ICP](https://arxiv.org/html/2211.16335v4).
The proposal's delayed-observer citation,
[Ahmed-Ali, Karafyllis and Lamnabhi-Lagarrigue](https://arxiv.org/abs/1207.1061),
provides context for sampled/delayed observer analysis; it does not establish
the repository's replay theorem by citation.
These were targeted primary-source checks, not a systematic literature or
novelty review. The coefficient, curb, disturbance and timer/replay
specializations above are explicit project derivations.

No calibrated registration-error, upstream cascade-disturbance, numerical
defect or chart-continuation budget has been established by these changes.
The previously recorded drive also violates the uniform timing/information
hypotheses. Accordingly, certificateVerified denotes checked conditional
matrix inequalities and observer.certified remains false.
This note was developed with AI-assisted analysis and executable checks;
publication claims still require scientific review of these stated
assumptions and bounds.
