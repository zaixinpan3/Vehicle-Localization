# Continuous observer theory: overlap closure and historical MO-HGO proofs

Current sensor-redundancy theorem: 2026-10-14. Overlap closure: 2026-10-12.
Historical mode-specific derivation: 2026-09-13.

Implementation revision, 2026-10-13: the default global observer now uses an
intrinsic eight-state predictor. IMU measurements enter invariant auxiliary
innovations and the upstream lateral observer. Section 0.5 specifies the
implemented state extension, fresh gain synthesis and remaining conditional
proof obligations. The former measured-course model and its gains are historical.

Section 0 gives the current continuous observer and its local nonlinear proof.
Section 0.6 adds one common certificate for GNSS-sufficient/LiDAR-degraded and
LiDAR-sufficient/GNSS-absent operation. It replaces an unspecified actual-operator
margin with explicit geometric mismatch bounds and finite joint LMIs. A separate
feasible theoretical witness is provided; it does not certify the default stored
gains or the geometry of a particular recording. Sections 1--6 preserve historical
mode-specific analyses; those certificates do not transfer to the current observer.

Runtime clarification, 2026-09-16: the GNSS-only and LiDAR-only systems below
are independent analysis cases, not a requirement to exclude one sensor from
the complete localization module. The current simultaneous, sampled,
zero-delay motion-aided runtime has a
[separate equation-level conditional analysis](research/full_observer_20260916/design.md).
This document and its original certificates do not certify that different
implementation merely because its sensors have the same names.

**Historical scope of Sections 1--6.** Those sections specify the continuous
MO-HGO analysis runner and treat two separate,
continuously measured systems: GNSS position, and uniformly informative LiDAR
pose with a known constant delay. The seven-state vehicle model and observer
are continuous-time differential equations. The LiDAR observer is a retarded
functional differential equation because its correction uses a past estimate.
Numerical integration does not change this modeling choice.

Those historical matrices are constant within each continuous measurement mode.
Their gains and certificates are separate, and make no switching claim. They
are distinct from the one-observer, common-metric result in Section 0.6.

The GNSS interface in this repository measures position only. Consequently,
its seven-state result requires motion and a local heading chart. An optional
independent GNSS heading output removes that motion requirement, as stated
separately below. LiDAR measures all three planar pose coordinates and its
fixed delay is retained explicitly. Neither measurement continuity nor
full-rank information alone is a stability certificate.

The historical analysis runner `runImprovedVehicleObserver.m` integrates the GNSS equations (8)
and the delayed LiDAR equation (23). Its numerical steps approximate these
continuous equations. The implementation uses separate, reverified constant
certificates, an explicit LiDAR initial history, and immutable state outputs.
See [the runtime contract](localization/README.md) and
[implementation validation](research/continuous_observer_runtime_20260913/validation.md).
Numerical success does not verify all physical, initial-error or disturbance
hypotheses.

## 0. Current overlap: completed local-region and bias closure

The guarantee considered here is local and continuous in time. Sections 0.1--0.5
first state the overlap-closure problem; Section 0.6 permits either sensor to
supply the absolute reference under one unified observer and certificate. Fix a constant
high-gain parameter `theta` (the current configuration uses 4) and heading length
`ell` (currently 10 m). No certificate for discretization, two indefinitely
inadequate sensors or arbitrary initial errors is claimed by this scope.

### 0.1 Actual overlap error and the deferred certificate

Use the normalized state and pose

$$
z_\ell=[X,V_X,A_X,Y,V_Y,A_Y,\ell\psi,\ell r]^\top,\qquad
q=Cz_\ell=[X,Y,\ell\psi]^\top,
$$

where `C` selects state components 1, 4 and 7. With

$$
T=\operatorname{diag}(\theta,\theta^2,\theta^3,
                       \theta,\theta^2,\theta^3,\theta,\theta^2),\qquad
\varepsilon=T^{-1}(z_\ell-\hat z_\ell),
$$

the pose error is exactly $q-\hat q=\theta C\varepsilon$.
Let $g_t(q)$ be the actual spatial overlap-gradient field in these pose units.
It must be continuously differentiable on each declared local region, measurable
in time and locally bounded with its Jacobian uniformly on finite time intervals.
The continuous plant is forward complete; the observer admits a local solution.
For a fixed smooth scalar feature-field cost, $g_t=\nabla c_t$ and its Jacobian
is the true Hessian. Pose-dependent cropping or weighting must be included in
the definition and Jacobian of the actual field; this section does not assume
those derivatives vanish or certify a spatial discontinuity.

Define the true-pose gradient bias and exact secant operator by

$$
\beta_t=g_t(q),\qquad
\bar H_t=\int_0^1 Dg_t(q+s(\hat q-q))\,ds.
$$

The fundamental theorem of calculus gives

$$
g_t(\hat q)=\beta_t+\bar H_t(\hat q-q).
$$

For the unit directional reliabilities used by the current configuration,
$M_t\succeq0$ is the Gauss--Newton surrogate,
$F_t=(M_t+\lambda_*I)^{-1}$ with $\lambda_*=64$, and
$W_t=F_t\bar H_t$ is the **actual** effective feedback. With the matched gain
$K_L=\kappa_LP^{-1}C^\top$, the scaled error equation has the form

$$
\dot\varepsilon=\mathcal A_t(\varepsilon)\varepsilon+d_t,\qquad
\mathcal A_t=\theta(A_{0,t}-K_LW_tC),\qquad
d_t=K_LF_t\beta_t+d_{0,t}.
$$

$A_{0,t}$ contains the intrinsic drift, invariant-output, scalar gyro/speed
and any admitted GNSS error terms;
$d_{0,t}$ contains their additive measurement and model disturbances, including
the upstream lateral-observer error. Their declared maneuver and disturbance
bounds must also hold throughout the region. No LiDAR pose solve is introduced.

The combined deferred stability hypothesis is: there are constant $P\succ0$ and
$\mu>0$ such that, uniformly in time and for all admitted errors with
$\|q-\hat q\|<r$,

$$
\mathcal A_t(\varepsilon)^\top P+P\mathcal A_t(\varepsilon)
\preceq-\mu I. \tag{O1}
$$

**(O1) has not been established for the active overlap operator.** The existing
LMI uses $S_t=F_tM_t$, which is generally different from $W_t$; full rank of
$M_t$, or positive definiteness of a true Hessian alone, does not prove (O1).
The results below close the other two gaps once (O1) is established. They do not
claim that changing the neighborhood or bounding bias can establish (O1).

### 0.2 Observation bias: ideal stationarity and bounded error

Two theoretical contracts are now explicit:

* **Ideal observations:** $\beta_t=0$. This requires a stationary overlap field
  at the true pose, not merely a full-rank surrogate.
* **Perturbed observations:** $\|\beta_t\|\le\bar\beta$ uniformly, and
  $\|d_{0,t}\|\le\bar d_0$. These are theoretical bounds; no runtime estimator
  or per-frame enforcement of them is required.

Ideal stationarity has a concrete geometric realization. Suppose, for each
semantic class and smoothing scale, the map and scan are identical nonzero smooth
square-integrable density fields after the true rigid transform, with fixed
class weights, complete integration and no pose-dependent admission. Their
overlap as a function of relative pose $\xi$ is
$E(\xi)=\langle\rho,U_\xi\rho\rangle$. Rigid transformation preserves the
$L^2$ norm, so Cauchy--Schwarz gives $E(\xi)\le E(0)=\|\rho\|^2$.
Consequently every smooth $-\log E$ is stationary at the correct alignment,
as is their weighted sum. This argument proves stationarity, not strict
curvature or uniqueness; those belong to the deferred operator analysis.
Partial views, unequal fields and changing evidence weights do not inherit this
ideal argument automatically and are represented by the bounded-bias contract.

Since $M_t\succeq0$ and $\lambda_*>0$,

$$
\|F_t\|_2\le\lambda_*^{-1},\qquad
\|d_t\|\le\bar d:=
\frac{\kappa_L\|P^{-1}C^\top\|_2}{\lambda_*}\bar\beta+\bar d_0.
\tag{O2}
$$

The bound is in normalized pose and scaled-state units. A persistent nonzero
bias permits convergence to an error neighborhood; it does not justify
convergence to exactly zero. Setting only $\beta_t=0$ does not remove the
other disturbances in $d_{0,t}$.

### 0.3 Local region, forward invariance and regional ISS

Choose a radius $r>0$ on which the spatial and coefficient hypotheses hold
uniformly, and require $r<\pi\ell$ so that the admitted yaw error stays on a
single angular branch. Write $p_- =\lambda_{\min}(P)$ and
$p_+=\lambda_{\max}(P)$, and define

$$
V=\varepsilon^\top P\varepsilon,\qquad
\chi=\lambda_{\max}(CP^{-1}C^\top),\qquad
c_* =\frac{r^2}{\theta^2\chi},\qquad
a=\frac{\mu}{2p_+},\qquad b=\frac{2p_+^2}{\mu}. \tag{O3}
$$

**Conditional local-region proposition.** Assume (O1), the field and maneuver
conditions above, and an essentially bounded disturbance satisfying (O2).
If

$$
V(0)<c_*,\qquad b\bar d^2<ac_*, \tag{O4}
$$

then the solution remains in $V<c_*$, hence in $\|q-\hat q\|<r$ and the
stated angular branch, for all $t\ge0$. Moreover,

$$
V(t)\le e^{-at}V(0)+\frac ba\bar d^2(1-e^{-at}), \tag{O5}
$$

and

$$
\|\varepsilon(t)\|\le
\sqrt{\frac{p_+}{p_-}}e^{-at/2}\|\varepsilon(0)\|
+\sqrt{\frac{b}{ap_-}}\bar d. \tag{O6}
$$

The same estimate holds with the disturbance supremum on $[0,t]$ in its
second term. The invariant-region admission condition uses a uniform bound on
the entire time interval. In physical normalized-state units,
$\|z_\ell-\hat z_\ell\|\le\|T\|_2\|\varepsilon\|$; in particular,
$\|q-\hat q\|\le\theta\|C\varepsilon\|$. This is a regional ISS statement
for inputs satisfying (O4), not global ISS for arbitrary disturbances.

**Proof.** Matrix Cauchy--Schwarz gives
$\|q-\hat q\|^2\le\theta^2\chi V$, so $V<c_*$ lies in the validity
region. While in that region, (O1) and Young's inequality imply, almost
everywhere,

$$
\dot V\le-\mu\|\varepsilon\|^2+2p_+\|\varepsilon\|\|d_t\|
\le-\frac\mu2\|\varepsilon\|^2+\frac{2p_+^2}{\mu}\|d_t\|^2
\le-aV+b\bar d^2.
$$

The comparison bound (O5), applied up to any proposed first exit, stays below
$c_*$ because both $V(0)$ and $(b/a)\bar d^2$ are strictly below $c_*$.
Thus no first exit occurs. Local solvability, the finite-time bounded plant
and the bounded error permit continuation; the bound holds for all time.
Finally $p_-\|\varepsilon\|^2\le V$ and
$\sqrt{x+y}\le\sqrt x+\sqrt y$ give (O6). No time derivative of $W_t$ is
used because $P$ is constant and (O1) is pointwise. With $d_t\equiv0$ the
error converges exponentially to zero. With persistent bounded disturbance,
$\limsup\|\varepsilon\|\le\sqrt{b/(ap_-)}\bar d$.

### 0.4 Remaining work and document scope

The bias model and invariant-region proof are complete **conditional on (O1)**.
Section 0.6 now supplies an explicit sufficient operator domain and common
LMI construction, including the finite nonlinear remainder. Applying its
certificate to a specific gain set requires solving/verifying the joint LMIs;
applying it to a specific map requires the geometric bounds stated there.
Neither follows from full rank alone, and this section does not assert them
for the default gain file or recorded maps. The manuscript still contains the earlier
affine LiDAR-pose model and needs a later route-specific revision after that
connection is resolved.

Temporary audit and feasibility studies were consolidated into this existing
document at the user's request. Research cleanup removes superseded studies;
the current configuration provenance, latest evaluations, directly referenced
materials and unrelated working changes remain. No new dated study or standalone
temporary report is required for this closure.

### 0.5 Implemented architecture: intrinsic prediction and invariant IMU outputs

The default is now `mncavFullObserverConfig`, which returns parameters only.
`synthesizeMncavInvariantDesign(cfg)` solves temporary gains immediately before
each test/experiment, without reading or writing a gain file.
`invariantObserverSupport.predict` accepts
only the internal state, time step and estimated lateral sideslip derivative.
No measured gyro or acceleration enters that function. Its eight physical
states are

$$
z=[X,V_X,A_X,Y,V_Y,A_Y,\psi,r]^\top,\qquad
\hat q=\hat r+\dot{\hat\beta},\qquad
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
\tag{I1}
$$

The nominal continuous predictor is

$$
\dot{\hat p}=\hat v,\quad \dot{\hat v}=\hat a,\quad
\dot{\hat a}=\hat q^2\hat v+2\hat qJ\hat a,\quad
\dot{\hat\psi}=\hat r,\quad \dot{\hat r}=0.
\tag{I2}
$$

For the plant, the acceleration-chain residual contains
`ddot(speed)*unitCourse + dot(courseRate)*J*v`; yaw acceleration is the residual
in `rdot`. Errors of the upstream estimated sideslip derivative are additional
model disturbances. Maneuver bounds concern both true states and the local
observer region. The eighth state removes the earlier `cross(v,a)/||v||^2`
quotient: there is no speed division, selected regularizer or concealed positive
speed assumption. Stopped rotation can be estimated through the gyro output.
The public `.z` and `.onlineZ` remain seven columns; `.internalState` and
`.yawRate` expose the extra estimated state.
The global auxiliary interface consumes the supplied lateral angle/rate pair;
it no longer reconstructs `atan2(vy,vx)`. The standard lateral observer exports
these from one persistent second-order state, tracking its zero convention
through stops. That convention does not give a stopped vehicle a physical
motion direction. Output-point adapters must export a pair for their own
declared motion point. The lateral reciprocal-speed
LPV certificate covers [1,30] m/s; its division-free low-speed runtime does not
extend that LMI to a unified stopped/hybrid theorem. These scope distinctions
are separate from the actual-overlap margin still to be proved.

IMU affects estimates through output innovations, so this placement restriction
is not a claim of IMU-independent localization. Estimated lateral signals may
also depend on IMU. Frontend tilt and source-window deskew retain their explicit
measured-motion interfaces; they are separate from the global predictor.

Let `s_*` be the configured maximum speed, `ell` the heading length,
`d(theta)=[cos(theta),sin(theta)]'`, and `n(theta)=J*d(theta)`. The implemented
scalar auxiliary outputs are

$$
h(z)=\begin{bmatrix}
 v^\top v/(2s_*)\\v^\top a/s_*\\\operatorname{cross}(v,a)/s_*\\
 \ell r\\[\operatorname{cross}(v,a)-\|v\|^2(r+\dot{\hat\beta})]/s_*\\
 n(\psi+\hat\beta)^\top v\\d(\psi)^\top a\\n(\psi)^\top a
\end{bmatrix},\qquad
y=\begin{bmatrix}
 (v_{x,m}^2+\hat v_y^2)/(2s_*)\\
 (v_{x,m}a_{x,m}+\hat v_ya_{y,m})/s_*\\
 (v_{x,m}a_{y,m}-\hat v_ya_{x,m})/s_*\\
 \ell r_m\\0\\0\\a_{x,m}\\a_{y,m}
\end{bmatrix}.
\tag{I3}
$$

Under a simultaneous global rotation, `v` and `a` rotate and `psi` shifts by
that angle. Every scalar in (I3) stays unchanged. The last two are projections
onto the estimated body directions; the measurement remains body acceleration,
without transforming it into a measured global vector. They retain acceleration
direction when wheel speed is absent. The discarded acceleration-norm prototype
lost that information. Course direction uses the upstream `sideSlipAngle`, with
its paired `sideSlipAngleRate` used in (I1); the global observer does not derive
either from body velocity. Synchronization carries both upstream series. Missing
angle series are rejected, and unavailable velocity withdraws direction feedback.
The standard filtered angle need not equal the instantaneous velocity direction.
For angle error `delta_beta=beta_hat-beta`, the true direction-output discrepancy
is `n(psi+beta_hat)'v=-s*sin(delta_beta)`, bounded by
`s_max*abs(delta_beta)`. Filter lag and output-point adapter errors therefore
enter the auxiliary disturbance; this bound vanishes at a stop. The separate
speed-channel angle error is treated likewise below. No new exact-angle or
zero-speed lateral stability assumption is inferred from consuming this interface.
The zero kinematic constraint is a model output, not an independent sensor.
Its residual includes sideslip-estimation and modeling errors.

Missing velocity withdraws outputs 1--3 and 6 and the separate speed innovation;
missing acceleration withdraws 2--3 and 7--8. No global predicted velocity fills
a missing measured speed. An invalid gyro withdraws output 4. Output 5 does not
require wheel data. Invalid payloads are ignored under their masks. KITTI uses
zero sideslip derivative as an explicit approximation and does not run the
vehicle-specific lateral observer.

The nonlinear innovations use the analytic Jacobian at the estimated state
and a matched metric solve. Output 4 has a separate scalar rate correction,
`rdot += g_r*(r_measured-r_hat)`, which changes the estimated rate only. The main
heading equation remains `psidot=r_hat`. Likewise a separate scalar, rotation
invariant course-speed output is
`d(psi_hat+beta_hat)'*v_hat`, measured as `hypot(vx_measured,vy_hat)`. Its correction
acts along that estimated direction. It uses no normalization by estimated
speed. These scalar gains are solved, rather than copied from the former gyro
input route: for process/noise scale ratio `b`, minimize `Q` subject to

$$
 g>0,\qquad
 \begin{bmatrix}Q&b&g\\b&2g&0\\g&0&2g\end{bmatrix}\succeq0.
\tag{I4}
$$

The analytic optimum `g=b` is an independent numerical check. The angular
process scale is the configured course-rate bound divided by a declared two
second maneuver scale; the speed process scale is twice maximum acceleration.
Neither ratio uses navigation reference poses at runtime.

**Fresh design and its exact scope.** Set `sigma=theta=4`,
`E=diag(1,1,1,1,1,1,ell,ell)` and
`T=diag(theta^[1,2,3,1,2,3,1,2])`. Gains use physical conversion `E^-1*T`
and `P=blkdiag(Pc,Pc,Ph)`. `Ph` is diagonal, so the LiDAR heading innovation
does not directly change the rate state. The metric-balance constraint
`Ph <= Pc(1:2,1:2)` prevents the very weak angular authority of an earlier
prototype. The intrinsic secant enclosure includes the new rate-state coupling
bounded by `2*q_max*v_max+2*a_max`; it cannot reuse the old measured-course
cancellation or its 768 vertices.

The main/gyro/aligned-speed surrogate inequalities are solved at 256 vertices
(128 without a velocity output):

$$
 P A_v+A_v^\top P
 -2\kappa_L C^\top S_v C
 -2\kappa_G w_v C_p^\top C_p
 +\kappa_G\rho_v^2 e_7e_7^\top+\lambda I\prec0,
\tag{I5}
$$

where `A_v` includes the intrinsic Jacobian enclosure and the scalar gyro
feedback; the speed-present vertices also include the aligned projected-speed
feedback. Gyro availability is a hypothesis of this certificate. The angular
output-point coupling is conservatively included. Verification recomputes every
vertex eigenvalue and the scalar LMIs from stored numbers without an SDP solver.
Changing any declared synthesis input rejects the saved gains.

A nominal noise objective also uses all eight normalized error coordinates.
Its four MnCAV operating points have a pose anchor: LiDAR, LiDAR plus GNSS,
weak LiDAR plus GNSS, and GNSS only. The former artificial dead-reckoning
steady-state objective was removed: invariants alone cannot determine global
translation/rotation. This noise proxy is a design objective, not a measured
covariance guarantee; reused sensors create correlated output errors. KITTI
must include weak LiDAR **without** GNSS in its objective, and must remove
velocity outputs from that objective. Those omissions caused poor prototypes.

**Finite auxiliary innovations can be closed locally without a new global
sector assumption.** In normalized coordinates write
`F=E^-1*T`, `H_hat=Gamma*Dh(z_hat)*F`, where `Gamma` contains the output
scalings, and let `H_bar` be the exact secant on the true/estimated segment.
On a compact maneuver region the polynomial/trigonometric outputs in (I3)
have finite bounds `||H_hat||<=M_H` and normalized Jacobian Lipschitz constant
`L_H`. There is no velocity quotient. Thus

$$
 \|H_{\rm bar}-H_{\rm hat}\|\le\tfrac12 L_H\|\varepsilon\|.
\tag{I6}
$$

For any admitted diagonal nonnegative auxiliary weight `K_I`, the matched
innovation contributes to `Vdot`, apart from measurement/model disturbances,

$$
 -2\theta\varepsilon^\top H_{\rm hat}^\top K_I H_{\rm hat}\varepsilon
 -2\theta\varepsilon^\top H_{\rm hat}^\top K_I
        (H_{\rm bar}-H_{\rm hat})\varepsilon
 \le\theta M_H\|K_I\|L_H\|\varepsilon\|^3.
\tag{I7}
$$

The first term is nonpositive; the unmatched finite secant is cubic in the
Lyapunov derivative. Withdrawn outputs have zero weights and satisfy the same
bound. This is stronger than simply assuming that the point Jacobian represents
the whole finite innovation.

For the separate speed channel let true `v=s*d(psi+beta)`, with `s>=0`.
With an exact lateral angle and heading difference `delta_psi`, its error
consists of the aligned rank-one damping already enclosed in (I5), plus
`-g_s*d_hat*s*(1-cos(delta_psi))`. This remainder is quadratic because
`1-cos(delta_psi)<=delta_psi^2/2`. In normalized velocity coordinates its norm
is at most `g_s*s_max*||epsilon||^2/(2*ell^2)`.
Lateral-angle and speed-measurement errors add bounded disturbances;
`|cos(a+b)-cos(a)|<=|b|` suffices. At a stop the `s` remainder vanishes. No
course-angle division or positive-speed premise is introduced.

Consequently, **if the intrinsic/main-output/gyro/aligned-speed error operator
has a uniform strict margin `mu_0` for the actual overlap**, then

$$
 \dot V\le-\mu_0\|\varepsilon\|^2+c_I\|\varepsilon\|^3
            +2\|P\|\|\varepsilon\|\|d\|,\qquad
 c_I=\theta M_H\|K_I\|L_H+
       \|P\|g_s s_{\max}/\ell^2.
\tag{I8}
$$

For `||epsilon||<=R<mu_0/(2*c_I)` (or any admitted radius when `c_I=0`), the
nominal quadratic margin is at least `mu_0/2`. Combine this additional normalized
radius with the motion/chart domain and the invariant-sublevel argument of
Section 0.3. The same continuous regional ISS argument then applies, with
bounded auxiliary measurement/model errors included in `d`. This closes the
finite auxiliary-remainder argument **conditionally on the strict base margin**.
It does not state a numerical radius or enforce one in the discrete runtime.

(I5) supplies that base margin for its **surrogate** domain. It does not supply
it for the actual overlap: `W=F*Hbar` still differs from `S=F*M`. Establishing
that actual base-operator inequality is the remaining difficult step for the
new intrinsic model. No completed nonlinear stability certificate is claimed
for the real overlap implementation. Diagnostics distinguish the conditional
local auxiliary closure from a certified numerical radius, and retain
`fullContinuousStateCertified=false`. No sampled-system, arbitrary-outage or
degenerate-scene guarantee is added.

Those flags describe the scope of the October 13 default gain file. Section 0.6
gives the separate joint theoretical design. Section 0.8 now adds the refined
conditions used to re-synthesize active gains; actual map and numerical-radius
flags still remain false, with conditional common-gain checks verified.

The former direct-gyro main-observer configurations, runtime paths and gain
synthesis were deleted on 2026-10-13. They remain recoverable at Git commit
`5cf9fae8103e87b519a679ea236ed5fda2d6a08e`; historical numerical results identify
that executed version and do not select a live branch.
This is a project-specific state extension inspired by Bessafa's intrinsic
prediction and auxiliary-output framework, not a claim that the paper proves
this new eight-state system. The initial seven-state quotient candidate in the
October 13 design record has been superseded by the implemented state extension.


### 0.6 One complete observer with complementary GNSS/LiDAR guarantees

Revision, 2026-10-14: G and L below name **sets of adequate measurements**, not
separate observers. One eight-state predictor, one state estimate, one constant
high-gain scale, one constant gain set and one Lyapunov metric run throughout.
No pose registration, state restart or alternate GNSS-only algorithm is added.
The only time variation is in measurements, geometry and declared channel
availability. The statement concerns continuous equations; it does not require
online enforcement or transfer its conclusion to a discretization.

#### Necessary motion condition for position-only GNSS

Let the receiver position be `y_G=p+R(psi)d_G`, with known body lever `d_G`.
At rest, for every constant heading `phi`, the state

$$
 p_\phi=y_G-R(\phi)d_G,\qquad v=a=0,\quad\psi=\phi,\quad r=0
\tag{R1}
$$

has identical GNSS, speed, acceleration and gyro outputs. If LiDAR contributes
no heading information, these states are indistinguishable for all time. This
remains true for a nonzero receiver lever. Thus sufficient **position precision**
alone cannot imply full-state asymptotic convergence or full-state ISS under
persistent gyro error. This is an observability obstruction, not a numerical
integration issue. A genuine independent heading measurement would remove it;
a gyro rate sample is not such a measurement.

For the G guarantee below, the velocity-direction and body-acceleration auxiliary
channels are available, gyro output is available, and the true course speed
satisfies `s_0 <= ||v|| <= s_max`, with declared `s_0>0`. Bounded upstream lateral
angle/rate errors are disturbances. This pointwise motion floor is a sufficient
condition; Section 0.7 also permits finite stops through an information-time
condition. LiDAR-sufficient operation does not impose this motion floor.

#### Unified equations and quantitative sensor adequacy

Retain the predictor (I2), invariant outputs (I3) and all their nonlinear
innovations. Put `B=E^-1 T`, `H_hat=Gamma*Dh(z_hat)*B`, with the implemented fixed
output normalization `Gamma=diag(theta^-[2,3,3,2,3,2,3,3])`, and use

$$
 K_I=\operatorname{diag}(0,k_a,k_a,0,k_c,k_d,k_a,k_a),\qquad k_a,k_c,k_d\ge0.
\tag{R2}
$$

Unavailable auxiliary outputs have zero weights. The auxiliary correction is
`theta*B*P^-1*H_hat'*K_I*Gamma*(y_I-h(z_hat))`. The scalar gyro and projected-speed
corrections remain those of Section 0.5, with physical gains
`g_r=theta*gbar_r` and `g_s=theta*gbar_s`. Raw IMU does not enter the predictor.
GNSS adds
`B*kappa_G*P^-1*Cp'*W_G*(y_G-p_hat-R(psi_hat)d_G)`;
LiDAR adds `-B*kappa_L*P^-1*C'*F_t*g_t(q_hat)`.
These terms are simultaneously present in the same observer.

Here `0 <= W_G <= I`, and `W_G=0` when GNSS is absent. For LiDAR, let `M_t`
be the admitted PSD information in normalized pose units, and let `Pi_t` project
onto its admitted range. For the theoretical exact support, `Pi_t*M_t=M_t`.
Numerically discarded small modes belong to the admission/model error rather
than silently redefining this equality. With unit directional reliabilities,

$$
 F_t=(M_t+\lambda_*I)^{-1}\Pi_t,\quad S_t=F_tM_t\succeq0,\quad
 W_t=F_t\overline H_t=S_t+\Delta_t,\quad
 \Delta_t=F_t(\overline H_t-M_t).
\tag{R3}
$$

`Pi_t=I` recovers Section 0.1 for fully informative LiDAR. Rank zero gives
`Pi_t=F_t=S_t=0`; no fictitious heading information is retained. This support
projection is the one used by the current gradient injection. The norm bound
`||F_t||<=1/lambda_*` remains valid. Reliability factors in [0,1] can be included
provided the same admitted S and norm/mismatch bounds hold.

Each time belongs to at least one of these two sets:

* **L: LiDAR sufficient.** The position block of S is at least `s_L I2`,
  its heading entry is at least `h_L>0`, and the norm of its position/heading
  coupling vector is at most `c_L`. This norm bound is preserved by the course
  rotation and implies the component box used in the LMIs. Also
  `||Delta_t||<=delta_L`. GNSS may be absent
  or have any weight `0<=W_G<=I`. Velocity/acceleration auxiliary channels may
  be withdrawn. Gyro remains available.
* **G: GNSS sufficient.** `W_G>=w_0 I2`, `w_0>0`, and the motion/auxiliary
  conditions stated above hold. LiDAR information may have any rank, including
  zero: only `S_t>=0` and `||Delta_t||<=delta_G` are required. Thus degraded
  LiDAR is permitted to have a bounded negative incremental contribution;
  it is not assumed harmless or replaced by a positive heading floor.

Biases of GNSS, overlap, motion outputs, the upstream lateral estimate and the
nominal jerk/rate model are uniformly bounded disturbances. The true trajectory
is forward complete and stays in the declared bounded motion domain. A fixed
neighborhood of it lies in the smooth motion/yaw chart on which the coefficient
and derivative bounds hold. Consider the estimate up to its first exit from
that neighborhood; the sublevel argument below prevents exit for admitted
initial errors and disturbances. Sensor fields and auxiliary flows have the local
regularity needed for an absolutely continuous solution. Support/availability
can change measurably in time; no derivative of the information weights is used.
Spatially discontinuous admission is not certified merely by these assumptions.

The mismatch bounds concern geometry, not stability of the observer. They can
be obtained without identifying the true secant with Gauss--Newton information.
If, uniformly in the admitted patch, `Dg` and M have Lipschitz constants `L_H`
and `L_M`, the true-pose discrepancy is bounded by `d_i`, and
`||q_hat-q||<=r`, then

$$
 \|\Delta_t\|\le
 \frac{d_i+(L_H/2+L_M)r}{\lambda_*}\le\delta_i,
 \qquad i\in\{L,G\}.
\tag{R4}
$$

Indeed the segment average differs from `Dg(q)` by at most `L_H r/2`, and
`M(q_hat)` differs from `M(q)` by at most `L_M r`; multiplication by F gives
(R4). A positive gap `lambda_* delta_i-d_i` therefore supplies a local radius.
A rank statement alone supplies neither this gap nor the L-sector bounds.
At rank zero the effective mismatch is exactly zero, irrespective of an unused
unprojected gradient. The bias bound and ideal-stationarity contract of Section
0.2 remain separate. The theorem does not assume full-state stability as a
sensor-adequacy condition.

#### Common finite LMIs

Use one `P=blkdiag(Pc,Pc,Ph)>0`, with identical translational blocks. Let `e_j`
select scaled state j, `C` select states 1,4,7, `Cp` select 1,4, and write
`E_a=[e_3,e_6]`, `D_a=E_a E_a'`, and `He(X)=X+X'`.
Rotate position, velocity and acceleration errors
jointly so the ideal course direction is X. P is unchanged by that rotation;
its generator is P-skew, hence contributes zero to the Lyapunov derivative.

The intrinsic true-point Jacobian, divided by theta, is enclosed by vertices
`A_v` with chain entries `(1,2),(2,3),(4,5),(5,6),(7,8)=1` and

$$
 A_v[\{3,6\},\{2,5\}]=uI_2/\theta^2,\quad
 A_v[\{3,6\},\{3,6\}]=2\omega J/\theta,\quad
 A_v[\{3,6\},8]=\frac{b}{\theta^2\ell}(\pm1,\pm1)^\top,
\tag{R5}
$$

where `u in {0,q_max^2}`, `omega in {-q_max,q_max}` and
`b=2*q_max*s_max+2*a_max`. Independent boxes are conservative but contain the
true Jacobian. Finite intrinsic secants are handled by the nonlinear remainder
below, not by asserting that the Jacobian is constant.

Let `rho=||d_G||/ell`, choose Young parameters `tau,eta in (0,1)` and desired
`lambda>0`, and define

$$
 A_v^\zeta=A_v-\bar g_r e_8e_8^\top-\zeta\bar g_s e_2e_2^\top.
$$

For every A vertex, every corner
`S_v=[[s_L I2,c],[c',h_L]]`, `c_j in {-c_L,c_L}`, and `zeta in {0,1}`, require

$$
 \operatorname{He}(PA_v^\zeta)
 -2\kappa_L C^\top S_vC
 +\kappa_G\rho^2 e_7e_7^\top
 +2\kappa_L\delta_L C^\top C+\lambda I\prec0.
\tag{R6-L}
$$

This includes no GNSS lower-information assumption. Its positive lever term
bounds any optional GNSS channel, including weight zero.

For G, the course-direction row at the true state is
`h_d(s)=e_5' - s/(theta*ell)e_7'`. Its matched negative term must be retained:
it carries the missing heading information through measured motion. Checking
only the endpoints of `-h_d(s)'h_d(s)` is insufficient. On each subinterval
`[s_i,s_{i+1}]` define three Bernstein controls

$$
 D_0=h_d(s_i)^\top h_d(s_i),\quad
 D_1=\tfrac12[h_d(s_i)^\top h_d(s_{i+1})+h_d(s_{i+1})^\top h_d(s_i)],\quad
 D_2=h_d(s_{i+1})^\top h_d(s_{i+1}).
\tag{R7}
$$

Their convex combination with weights `(1-t)^2,2t(1-t),t^2` is exactly
`h_d((1-t)s_i+t s_{i+1})'h_d((1-t)s_i+t s_{i+1})`. Consequently, for every A
vertex and all three controls of all subintervals, require

$$
\begin{aligned}
 &\operatorname{He}(PA_v^1)
 -2\kappa_G(1-\tau)w_0 C_p^\top C_p-2k_dD_j
 -2k_a(1-\eta)D_a\\
 &\quad+
 \left[\frac{\kappa_G\rho^2}{\tau}
 +\frac{2k_a(\eta^{-1}-1)a_{\max}^2}{\theta^4\ell^2}\right]e_7e_7^\top
 +2\kappa_L\delta_G C^\top C+\lambda I\prec0.
\end{aligned}
\tag{R6-G}
$$

The G constraint contains **no positive LiDAR information or heading strength**.
All inequalities use the same P and the same `kappa_L,kappa_G,k_a,k_d`.
For fixed domain/scalar gains these are affine LMIs in those unknowns. There is
no requirement to alternate between gain designs. Additional nonnegative
invariant gains, including `k_c`, are permitted; their leading matched terms
are nonpositive and their finite nonlinear remainders are retained below.

The bounds underlying (R6) are explicit. With `x=Cp*epsilon`, the GNSS lever
secant has norm at most rho. Its derivative contribution is at most
`theta*kappa_G*rho^2*epsilon_7^2` in L, and at most
`theta*kappa_G[-2(1-tau)w_0||x||^2+rho^2/tau*epsilon_7^2]` in G. For the two
body-acceleration projections, the normalized true-point rows together are
`H_a=E_a'-(J a)/(theta^2*ell)e_7'`, up to an orthogonal row rotation. Thus

$$
 \|H_a\varepsilon\|^2\ge
 (1-\eta)\|E_a^\top\varepsilon\|^2
 -(\eta^{-1}-1)\frac{a_{\max}^2}{\theta^4\ell^2}\varepsilon_7^2.
\tag{R8}
$$

For exact overlap feedback, its difference from S contributes at most
`2*theta*kappa_L*delta_i*||C epsilon||^2`. These three bounds yield (R6);
none replaces an exact nonlinear output difference by a point Jacobian without
accounting for the difference.

#### Complete local nonlinear theorem and arbitrary sensor switching

**Unified continuous redundancy theorem.** Assume the uniform geometry,
regularity, motion and disturbance contracts above. Suppose the finite joint
LMIs (R6-L,G) have one constant positive definite P and one constant gain set.
At every time, at least one of L or G holds. Then the complete nonlinear
continuous observer has a local regional ISS error bound under arbitrary
measurable transitions between these adequacy sets. In particular:

* L gives the full eight-state guarantee with GNSS absent, including at a stop.
* G gives the full eight-state guarantee with LiDAR rank deficient or entirely
  absent, subject to its position-only motion/auxiliary conditions.
* Both channels may be present. The proof label does not select a different
  algorithm, reset the state or change any gain. No dwell time is needed while
  the union-adequacy condition holds.

**Proof.** (R6), their convex vertex enclosures, and (R7) give the ideal base
bound `Vdot <= -theta*lambda*||epsilon||^2`, with `V=epsilon'P epsilon`.
The actual overlap perturbation is already included through (R3)--(R4).
On the compact motion/yaw chart, the intrinsic predictor and the complete
invariant-output correction have uniformly bounded second derivatives in the
state. Expanding about the true state gives a vector-field remainder bounded
by `L_N||epsilon||^2`, including both the change in the estimated Jacobian gain
and the nonlinear output secant. The scalar-speed heading remainder and all
retained invariant channels are included in this bound. Upstream angle/rate,
sensor and model errors add a uniformly bounded scaled disturbance `d(t)`;
the filtered angle is not assumed equal to the instantaneous direction.
Therefore, with `p_+=lambda_max(P)` and `c_N=2*p_+*L_N`,

$$
 \dot V\le-\theta\lambda\|\varepsilon\|^2
 +c_N\|\varepsilon\|^3+2p_+\|\varepsilon\|\|d(t)\|.
\tag{R9}
$$

Choose a scaled-state radius `R` that lies in the motion domain and satisfies
`theta*R <= min(r_L,r_G)`, `theta*R < pi*ell`, and
`c_N R <= theta*lambda/2`. The first two conditions follow from
`q-q_hat=theta*C*epsilon` and keep the whole pose segment in the geometric patch
and one yaw chart. Put `p_-=lambda_min(P)` and

$$
 a=\frac{\theta\lambda}{4p_+},\qquad
 b_d=\frac{4p_+^2}{\theta\lambda},\qquad c_R=p_-R^2.
$$

Young's inequality gives `Vdot <= -a V+b_d||d||^2`. If
`V(0)<c_R` and `(b_d/a)*||d||_infinity^2<c_R`, the first-exit argument yields

$$
 V(t)\le e^{-at}V(0)+\frac{b_d}{a}\|d\|_\infty^2(1-e^{-at}),\qquad
 \|\varepsilon(t)\|\le
 \sqrt{\frac{p_+}{p_-}}e^{-at/2}\|\varepsilon(0)\|
 +\sqrt{\frac{b_d}{a p_-}}\|d\|_\infty.
\tag{R10}
$$

V has no jump when adequacy changes because P and the estimate are the same.
The derivative inequality holds almost everywhere, so (R10) survives arbitrary
measurable switching. Zero total disturbance gives local exponential
convergence; persistent bounded disturbance gives a local error neighborhood.
This proves the full nonlinear statement under the stated sufficient conditions.
It does not assert global convergence, stability under two indefinitely
inadequate sensors, or a numerical attraction radius. **QED.**

This is the complete eight-state global-observer error result, with bounded
upstream lateral-estimation error included in d. An augmented cascade result
also requires an upstream error bound on the same vehicle/input domain; it
does not follow solely from the global LMIs. No new low-speed hybrid lateral
certificate is inferred here.

#### Feasible joint witness and boundary of its applicability

The following witness demonstrates that the simultaneous sufficient LMIs are
not merely a formal condition. It uses the adopted MnCAV bounds and scalar
rate/speed gains, ten speed subintervals, and a GNSS motion floor of 10 m/s.
That floor is a conservative feasibility-domain choice, not a physical law or
an automatic requirement of every GNSS observer. Smaller floors require a new
successful certificate; exploratory 5--15 m/s solves with the stated box and
Young enclosures were infeasible and do not establish impossibility of that
motion domain. The general theorem retains a symbolic positive `s_0`.

The witness metric and matched gains are **separate theoretical parameters**.
They were not installed as runtime defaults, do not replace
any current observer, and do not certify a later temporary design. They also do
not verify (R4) or the L-sector bounds for an actual map. The verified assertion
is feasibility of one common constant design under those explicit geometric
and signal contracts. The numerical block below is a historical record only;
no runtime or test parses it for gains. `verifySensorRedundancyTheory` now solves
a temporary theoretical witness by default. Supplying an in-memory `Witness`
and freshly solved `Config` permits solver-free reverification.

<!-- SENSOR_REDUNDANCY_WITNESS_BEGIN -->
```json
{
  "inputs": {
    "theta": 4,
    "headingLengthM": 10,
    "maximumSpeed": 15,
    "maximumAcceleration": 2,
    "maximumCourseRate": 0.4,
    "minimumLidarPositionStrength": 0.25,
    "minimumLidarHeadingStrength": 0.97,
    "maximumLidarCoupling": 0.06,
    "minimumGnssWeight": 0.5,
    "gnssBodyOffset": [
      1.72485903887133,
      0.2644114700440067
    ],
    "gyroGainNormalized": 27.777578635450332,
    "speedGainNormalized": 3.9062239575646722
  },
  "domain": {
    "minimumGnssSpeed": 10,
    "informativeLidarMismatch": 0.001,
    "degradedLidarMismatch": 0.001,
    "speedPartitions": 10,
    "accelerationSplit": 0.5,
    "gnssSplit": 0.5,
    "lambda": 1
  },
  "P": [
    [
      30.875590672687,
      -70.18091224259713,
      -287.99466662991614,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    [
      -70.18091224259713,
      388.8036113999334,
      -192.66799320550328,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    [
      -287.99466662991614,
      -192.66799320550328,
      6169.715507157887,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    [
      0.0,
      0.0,
      0.0,
      30.875590672687,
      -70.18091224259713,
      -287.99466662991614,
      0.0,
      0.0
    ],
    [
      0.0,
      0.0,
      0.0,
      -70.18091224259713,
      388.8036113999334,
      -192.66799320550328,
      0.0,
      0.0
    ],
    [
      0.0,
      0.0,
      0.0,
      -287.99466662991614,
      -192.66799320550328,
      6169.715507157887,
      0.0,
      0.0
    ],
    [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.01,
      0.0
    ],
    [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      223.88604800784861
    ]
  ],
  "kappaL": 183.57316735373308,
  "kappaG": 70.37006277381806,
  "accelerationGain": 1298.5773627406172,
  "directionGain": 741.1675319450827
}
```
<!-- SENSOR_REDUNDANCY_WITNESS_END -->

The retained text table above is the certificate summary, not a replay/cache
array. The verifier checks 128 L vertices and 480 G Bernstein controls. The
matrix/gains were homogeneously rescaled by 1.01 after synthesis to give a clear
floating-point verification margin without changing matched gain ratios.

### 0.7 Finite stops and intervals outside instantaneous adequacy

A motion floor is not required at every time if information recurs often enough.
Suppose the same V satisfies `Vdot<=-a V+B||d||^2` on adequate intervals and
`Vdot<=gamma V+B||d||^2` on the remaining intervals within the same local region.
The second bound follows from uniform local vector-field bounds; it is not a
claim of convergence in an unobservable parked interval. Let `chi(t)` mark the
adequate intervals. If for every `s<=t`,

$$
 \int_s^t[a\chi(\tau)-\gamma(1-\chi(\tau))]d\tau
 \ge\bar a(t-s)-K,\qquad\bar a>0,
\tag{R11}
$$

then integrating factors give

$$
 V(t)\le e^K e^{-\bar a t}V(0)
 +\frac{B e^K}{\bar a}\|d\|_\infty^2.
\tag{R12}
$$

A sufficient first-exit admission is
`e^K*(V(0)+B||d||_infinity^2/bar_a)<c_R`. For example, if the total inadequate
time on every interval is at most `rho_bad*(t-s)+T_0`, then (R11) holds with
`bar_a=a-(a+gamma)rho_bad>0` and `K=(a+gamma)T_0`. This permits finite stops or
finite intervals of simultaneously weak signals while sufficient information
recurs; it excludes indefinite parking with position-only GNSS and no heading
observation. An independent heading output would instead remove the obstruction
in (R1), requiring the corresponding extra heading term in the joint LMI.


### 0.8 Active-gain synthesis with tighter GNSS enclosures

Revision, 2026-10-14: the old active gains passed the L inequalities but failed
the new G inequalities (worst eigenvalues approximately -2.81681 and +3.50637).
This is a failure of those sufficient conditions, not a proof of instability.
The active synthesis must therefore include the new common conditions, rather
than merely cite the separate witness above. The original 256 surrogate checks
are retained as additional constraints; the following valid refinements reduce
conservatism in G without changing the observer or switching its gains.

#### Exact kinematic restriction in the moving G set

Use the ideal true angle/rate pair in the nominal analysis. Differences between
that pair and the supplied filtered pair enter the bounded input on the compact
region. For positive speed, the identity `v=s*u(course)` and `vdot=a` give, in
the true course frame,

$$
 v=(s,0)^\top,\qquad a=(\dot s,sq_c)^\top,\qquad
 2q_cv+2Ja=(0,2\dot s)^\top.
\tag{R13}
$$

Thus the true-point rate column in (R5) can be restricted in G to
`A[{3,6},8]=[0; +/-2*a_max/(theta^2*ell)]`. Also
`|q_c|<=min(q_max,a_max/s_0)`, so the u/rate vertices in G use that smaller
bound. L retains the original broad box, including zero speed. This identity
is used only in the proof and synthesis; no velocity quotient is evaluated in
the predictor. For example the predictor discrepancy due to upstream rate
error is bounded on the local region by
`(2*q_hat,max*||v_hat||+2*||a_hat||)*|delta_betaDot| +
||v_hat||*delta_betaDot^2`, where `q_hat,max` bounds the ideal-input estimate
coefficient on the declared local chart. This does not assert that the estimate
always satisfies the true-point bound. The uniform input/remainder argument applies.

#### Retain the GNSS lever coupling instead of a Young penalty

In the course frame, the true-point normalized GNSS row is
`C_G=Cp+l*e7'`, with `||l||<=rho=||d_G||/ell`. Its exact leading correction is
`-theta*kappa_G*P^-1*Cp'*W_G*C_G*epsilon`.
Enclose l by the four component corners `(+/-rho,+/-rho)`.
To cover every symmetric weight `w_0 I<=W_G<=I`, put
`m=(1+w_0)/2`, `c=(1-w_0)/2` and use the six vertices

$$
 W_j\in\left\{w_0I,I,
 mI+c\begin{bmatrix}\pm1&\pm1\\\pm1&\mp1\end{bmatrix}\right\},
\tag{R14}
$$

where the diagonal sign and off-diagonal sign vary independently. To verify
coverage, write any symmetric W as `zI+[[x,y],[y,-x]]`. Its eigenvalues are
`z +/- sqrt(x^2+y^2)`. Hence its cross-section at z has radius at most
`min(z-w_0,1-z)`. Below m, interpolate between `w_0I` and the middle square
`|x|,|y|<=c`; above m interpolate between that square and I. The disk is
contained in the square, proving the convex enclosure. Some vertices slightly
exceed the original spectral interval, which is conservative. They do not
assert extra sensor information. The product of a weight and lever row is
multi-affine, so checking all weight/lever corners covers their convex product.

For every refined G drift vertex, Bernstein control and weight/lever corner,
an alternative sufficient G inequality is

$$
\begin{aligned}
 &\operatorname{He}\left(PA_v^1-
 \kappa_G C_p^\top W_j(C_p+l_j e_7^\top)\right)
 -2k_dD_i-2k_a(1-\eta)D_a\\
 &\quad+\frac{2k_a(\eta^{-1}-1)a_{\max}^2}{\theta^4\ell^2}e_7e_7^\top
 +2\kappa_L\delta_G C^\top C+\lambda I\prec0.
\end{aligned}
\tag{R15}
$$

It replaces the two GNSS Young terms of (R6-G) with the retained exact leading
coupling. There is still no positive LiDAR floor. The L inequalities (R6-L)
are unchanged and use the same P and gains. With ten speed cells, the refined
G box has eight drift vertices and therefore 5760 G controls; L has 128. When the declared GNSS lever is exactly zero, the G correction is `-kappa_G*Cp' *W_G*Cp`. Since `W_G>=w_0*I`, all larger weights add dissipation. Only `w_0*I` is needed and the four lever corners coincide, reducing G to 240 controls while covering every admissible GNSS weight. This exact zero-lever specialization also avoids the unnecessary conservatism of the larger weight polytope. Removing those duplicates avoids the numerical false-infeasibility observed during fresh zero-lever synthesis.

The exact finite GNSS output is not silently replaced by its point Jacobian.
The lever remainder has norm at most
`||d_G||*theta^2*epsilon_7^2/(2*ell^2)` before multiplication by
`kappa_G*P^-1*Cp'*W_G`. It is uniformly quadratic on the chart and is added to
`L_N` in (R9). All other nonlinear and disturbance bounds are retained.
Consequently the complete regional theorem, common-metric switching proof and
finite-stop extension apply with (R15) instead of (R6-G). This is a refinement
of the sufficient certificate for the same constant-gain observer.

#### Feedback authority is constrained inside the optimization

A small raw scalar gain is not necessarily a small applied correction because
P is also optimized. For `P>0`, `k>=0`, and a declared positive bound g,

$$
 k\,e^\top P^{-1}e\le g
 \quad\Longleftrightarrow\quad
 P-\frac{k}{g}ee^\top\succeq0.
\tag{R16}
$$

Congruence by `P^-1/2` reduces this equivalence to a rank-one eigenvalue test.
The active synthesis limits matched position and velocity-direction self-feedback
by these affine LMIs; the equivalence also applies to other coordinates. With
diagonal Ph, heading limits are `kappa_L<=g_L*P77` and `k_d<=g_d*P77`.
These are fixed design constraints, not runtime clipping
or a fallback observer. The optional balance constraint `Ph<=Pc(1:2,1:2)` is
not a hypothesis of the theorem and may be withdrawn when it obstructs these
feedback bounds. Rotation invariance and positive definiteness are retained.


The re-synthesis at commit `698df54986619aa2fdc318f8fd7692be91fa1a08` used the
refined certificate and feedback bounds. Its recorded scalars were kappa_L=351.511784646,
kappa_G=279.929704281, k_a=1682.03354042,
k_c=2.02349895467, k_d=948.761480676;
P and all inputs remain recoverable from that commit. Its 128 L and 5760 G
controls have worst eigenvalues -0.0100005770652 and
-0.0100001085111, in addition to 256 retained surrogate checks.
Runtime recomputes all inequalities and feedback caps independently of claims.
The old gains also failed the refined G family (+2.37977567581);
even its zero-lambda raw matrix margin was positive (+1.37977567581), so
homogeneous rescaling of that old metric/gain set alone cannot establish it.
These numbers are historical examples, not current defaults. The gain JSON
was subsequently removed: each test/experiment calls the public solver and
uses its in-memory result. Tests share a solve only within one fixture lifetime
and discard it before the next run. Both scalar channel gains are also solved
again; no persistent gain cache supplies a new run. Changed fixed theta or
common-domain inputs reject stale in-memory metadata.
The upstream lateral design and physical vehicle/sensor parameters are unchanged.


## 1. State, continuous plant, and disturbance interface

Let

$$
z=[X,V_X,A_X,Y,V_Y,A_Y,\psi]^\top,\quad
v_g=[V_X,V_Y]^\top,\quad a_g=[A_X,A_Y]^\top,
$$

and let $e_i$ denote the $i$th column of $I_7$. Define

$$
A_3=\begin{bmatrix}0&1&0\\0&0&1\\0&0&0\end{bmatrix},\quad
A=\operatorname{diag}(A_3,A_3,0),\quad
B=[e_3,e_6],\quad b(t)=e_7 r_m(t),
$$

$$
F(q)=\begin{bmatrix}
0&q^2&0&0&0&-2q&0\\
0&0&2q&0&q^2&0&0
\end{bmatrix},\qquad M(q)=A+BF(q).
$$

The measured course-rate interface is $q=r_m+\dot{\hat\beta}$, and the plant is

$$
\boxed{\dot z=M(q(t))z+b(t)+w(t).} \tag{1}
$$

Here $w=Bd_j-e_7\nu_r$, with $r_m=\dot\psi+\nu_r$. Other bounded model
residuals can also be included in $w$. In particular, $d_j$ is not set to zero
merely to obtain a stability result.

For nonzero speed, write $v_g=v[\cos\theta_c,\sin\theta_c]^\top$,
$\theta_c=\psi+\beta$, $q_c=\dot\theta_c$, and
$J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$. Differentiation gives

$$
\dot a_g=q_c^2v_g+2q_cJa_g+
\ddot v\begin{bmatrix}\cos\theta_c\\\sin\theta_c\end{bmatrix}
+\dot q_cJv_g. \tag{2}
$$

Thus the precise model residual is

$$
d_j=\ddot v\begin{bmatrix}\cos\theta_c\\\sin\theta_c\end{bmatrix}
+\dot q_cJv_g+(q_c^2-q^2)v_g+2(q_c-q)Ja_g. \tag{3}
$$

Equation (1) itself has no division by speed. At zero speed one defines $d_j$
from the actual Cartesian jerk minus the nominal jerk, without introducing
an undefined course angle. Bounded $q$ and bounded $w$ are explicit hypotheses,
not consequences of a position measurement.

The upstream lateral observer uses wheel speed, steering and IMU signals,
without feedback from the global observer. For a cascade result, its complete
interface error, including the error of $\dot{\hat\beta}$, must satisfy an ISS
bound. An ISS bound on $\hat\beta-\beta$ alone does not prove a bound on its
derivative. Derivatives of bounded sensor noise are not assumed bounded.

## 2. Auxiliary outputs and high-gain scaling

After gravity and lever-arm compensation, body inertial acceleration is
$a_b=[a_x,a_y]^\top$ and body velocity is $[v_x,v_y]^\top$. Rotation preserves
squared norm, inner product, and planar cross product. The auxiliary outputs are

$$
h(z,t)=\begin{bmatrix}
V_X^2+V_Y^2\\ V_XA_X+V_YA_Y\\ V_XA_Y-V_YA_X\\
V_Y\cos(\psi+\hat\beta)-V_X\sin(\psi+\hat\beta)
\end{bmatrix},\qquad
 y_h=\begin{bmatrix}
v_x^2+\hat v_y^2\\ v_xa_x+\hat v_ya_y\\ v_xa_y-\hat v_ya_x\\0
\end{bmatrix}=h(z,t)+n_h. \tag{4}
$$

In particular $n_{h,4}=-v\sin(\beta-\hat\beta)$, so
$|n_{h,4}|\le v|\beta-\hat\beta|$. The other three errors include wheel-speed,
acceleration and lateral-velocity errors. Products of bounded signals admit
class-$\mathcal K$ disturbance bounds on a stated operating envelope; no
statistical independence or white-noise assumption is used.

Assume the true velocity and acceleration components obey
$|V_X|,|V_Y|\le\bar v$ and $|A_X|,|A_Y|\le\bar a$.
Use the extension $h_{\rm ext}$ obtained by clipping only these four arguments
of $h$ to their respective intervals. Position, heading, the estimated state,
and the linear predictor are not clipped. The extension agrees with the true
output inside the envelope and has bounded generalized Jacobians. Along the
line segment between $z$ and $\hat z$,

$$
h_{\rm ext}(z,t)-h_{\rm ext}(\hat z,t)=H(t)(z-\hat z). \tag{5}
$$

The thirteen possible nonzero entries of $H$ obey the following outer box;
all unlisted entries are zero. Clip derivatives in $[0,1]$ preserve these bounds.

| Output row | State columns | Absolute coefficient bounds |
|---|---|---|
| 1 | 2, 5 | $2\bar v,2\bar v$ |
| 2 | 2, 5, 3, 6 | $\bar a,\bar a,\bar v,\bar v$ |
| 3 | 2, 5, 6, 3 | $\bar a,\bar a,\bar v,\bar v$ |
| 4 | 2, 5, 7 | $1,1,2\bar v$ |

For a fixed design parameter $\theta\ge1$, define

$$
T=\operatorname{diag}(\theta,\theta^2,\theta^3,
\theta,\theta^2,\theta^3,\theta),\quad
L=TK,\quad L_h=TN/\theta^3. \tag{6}
$$

The scaling parameter is fixed during operation. In particular,
$T^{-1}AT=\theta A$ and $CT=\theta C$ for a pose-selection matrix $C$.
The invariant error enters as $-\theta^{-3}NHT\varepsilon$, where
$\varepsilon=T^{-1}(z-\hat z)$. All measurement-noise terms have the negative
sign implied by defining the error as true state minus estimate.

## 3. Mode G: continuous GNSS position

### 3.1 Why the seventh state needs a hypothesis

The actual GNSS output is

$$
y_G=C_Gz+n_G,\qquad C_G=\begin{bmatrix}e_1^\top\\e_4^\top\end{bmatrix}. \tag{7}
$$

At rest, take $v_g=a_g=0$, $r_m=0$, and zero disturbances. Every constant
heading produces the same GNSS and auxiliary outputs. Two different headings
are therefore indistinguishable. A seven-state zero-input error cannot be
required to converge to zero from all such initial conditions. This is a
counterexample to unconditional seven-state ISS from position-only GNSS.
A small nonzero yaw-rate bias also causes uncorrected heading drift at rest.

The following result supplies the needed motion and heading-domain assumptions
and an explicit gain structure. It does not silently borrow LiDAR heading.

### 3.2 Continuous triangular MO-HGO

Put $x=[X,V_X,A_X,Y,V_Y,A_Y]^\top$, let $M_6$ be the upper-left block of $M$,
and let $h_3$ denote the first three auxiliary outputs. Define
$T_6=\operatorname{diag}(\theta,\theta^2,\theta^3,\theta,\theta^2,\theta^3)$
and $C_6x=[X,Y]^\top$. Choose

$$
\begin{aligned}
\dot{\hat x}&=M_6(q)\hat x+T_6K_G(y_G-C_6\hat x)
+T_6N_G\theta^{-3}[y_{h,1:3}-h_{3,\rm ext}(\hat x)],\\
\dot{\hat\psi}&=r_m+k_\psi[\hat V_Y\cos(\hat\psi+\hat\beta)
-\hat V_X\sin(\hat\psi+\hat\beta)],\quad k_\psi>0.
\end{aligned} \tag{8}
$$

The yaw row uses the raw estimated velocity; it remains linear in that
velocity. Its error will be bounded by the preceding six-state estimate.
This is a member of the seven-state MO-HGO family: the first six rows of
$N$ use the first three auxiliary outputs and its last row has
$N_{7,4}=-k_\psi\theta^2$, with all other entries in the last row zero.
The GNSS gain has a zero last row. The fourth auxiliary output is not fed
back into the first six rows. This triangular design avoids asking a
sign-indefinite global heading-Jacobian box to establish heading observability.
It is an explicit new gain constraint, not a claim about arbitrary stored $N$.

Let $\epsilon=T_6^{-1}(x-\hat x)$. Its exact dynamics are

$$
\dot\epsilon=A_G(q,H_3)\epsilon+u_G,\quad
A_G=T_6^{-1}M_6T_6-\theta K_GC_6-\theta^{-3}N_GH_3T_6,
$$

$$
u_G=T_6^{-1}w_{1:6}-K_Gn_G-\theta^{-3}N_Gn_{h,1:3}. \tag{9}
$$

### 3.3 Constant-matrix six-state certificate and feasibility

For all admissible $q$ and incremental matrices $H_3$, require a constant
$P_6=P_6^\top\succ0$ and $a>0$ such that

$$
\operatorname{He}(P_6A_G)\preceq-aI_6. \tag{10}
$$

This is a finite LMI test after enclosing $(q,q^2)$ by four vertices and
$H_3$ by its coefficient box. At fixed $\theta$, synthesis can use
$Y_G=P_6K_G$ and $Z_G=P_6N_G$. The same matrices must satisfy every vertex.

The condition is not vacuous. Select $K_G$ so that
$A_6-K_GC_6$ is Hurwitz, and solve
$\operatorname{He}[P_6(A_6-K_GC_6)]=-I_6$.
For example each third-order chain can use $[3,3,1]^\top$.
For any fixed finite nonzero $N_G$, write
$A_G=\theta(A_6-K_GC_6)+E_G$. Uniformly for $\theta\ge1$,

$$
\|E_G\|\le\bar q\sqrt{\bar q^2+4}+\|N_G\|L_3,
\quad L_3=\sqrt{12\bar v^2+4\bar a^2}, \tag{11}
$$

where $L_3$ is the Frobenius upper bound of the ten-entry $H_3$ box.
The exact known-input contribution before relaxing $\theta$ is
$|q|\sqrt{q^2/\theta^2+4}$. Therefore (10) holds with

$$
a=\theta-2\|P_6\|[\bar q\sqrt{\bar q^2+4}+\|N_G\|L_3]>0. \tag{12}
$$

This proves existence for the six-state block at sufficiently large fixed
$\theta$. It is conservative; solving (10) can permit smaller gains.

### 3.4 Seven-state ISS theorem for moving GNSS

**Theorem G.** Suppose (1), (4), and (7)--(10) hold, all indicated inputs are
bounded, and the true speed satisfies
$0<v_*\le v(t)\le v^*$. Let $e_\psi=\psi-\hat\psi$ be a consistently lifted
heading error. Choose $0<\delta<\pi/2$ and restrict the initial error and
inputs as specified below so that $|e_\psi|\le\delta$ is invariant.
Then the observer (8) is locally exponentially ISS in all seven errors.

**Proof.** With $e_v=v_g-\hat v_g$ and
$\tilde\beta=\beta-\hat\beta$, the yaw innovation satisfies

$$
\hat V_Y\cos(\hat\psi+\hat\beta)-\hat V_X\sin(\hat\psi+\hat\beta)
=v\sin(e_\psi+\tilde\beta)-
[-\sin(\hat\psi+\hat\beta),\cos(\hat\psi+\hat\beta)]e_v.
$$

Consequently

$$
\dot e_\psi=-k_\psi v\sin e_\psi+d_\psi,\quad
|d_\psi|\le k_\psi\theta^2\|\epsilon\|+u_\psi,\quad
u_\psi=k_\psi v^*|\tilde\beta|+|\nu_r|. \tag{13}
$$

Define $a_\psi=k_\psi v_*\sin\delta/\delta$ and $b_\psi=k_\psi\theta^2$.
For $|e_\psi|\le\delta$, $e_\psi\sin e_\psi\ge
(\sin\delta/\delta)e_\psi^2$. Choose a constant

$$
0<c\le\frac{a a_\psi}{8b_\psi^2},\qquad
V_G=\epsilon^\top P_6\epsilon+c e_\psi^2. \tag{14}
$$

Writing $p=\lambda_{\max}(P_6)$, differentiating (14), and applying Young's
inequality to the three cross terms gives

$$
\begin{aligned}
2p\|\epsilon\|\|u_G\|&\le\tfrac a4\|\epsilon\|^2+
\tfrac{4p^2}{a}\|u_G\|^2,\\
2c b_\psi|e_\psi|\|\epsilon\|&\le\tfrac a4\|\epsilon\|^2+
\tfrac{4c^2b_\psi^2}{a}e_\psi^2,\\
2c|e_\psi|u_\psi&\le\tfrac{c a_\psi}{2}e_\psi^2+
\tfrac{2c}{a_\psi}u_\psi^2.
\end{aligned}
$$

Together with (10) and (14), these imply

$$
\dot V_G\le-\tfrac a2\|\epsilon\|^2-c a_\psi e_\psi^2+
\tfrac{4p^2}{a}\|u_G\|^2+\tfrac{2c}{a_\psi}u_\psi^2
\le-\lambda_G V_G+D_G(t), \tag{15}
$$

where $\lambda_G=\min\{a/(2p),a_\psi\}>0$ and $D_G$ is the displayed
nonnegative disturbance sum. Define the constant physical-error metric

$$
\bar P_G=\operatorname{diag}(T_6^{-\top}P_6T_6^{-1},c),\quad
\bar p_- =\lambda_{\min}(\bar P_G),\quad
\bar p_+ =\lambda_{\max}(\bar P_G).
$$

Scalar comparison proves the explicit seven-state ISS estimate

$$
\boxed{\|e(t)\|\le
\sqrt{\bar p_+/\bar p_-}\,e^{-\lambda_G t/2}\|e(0)\|
+\sqrt{\frac{\|D_G\|_{[0,t],\infty}}{\lambda_G\bar p_-}}.} \tag{16}
$$

There is no circular invariance assumption. From (9)--(10), independently of
yaw, with $\kappa_6=p/\lambda_{\min}(P_6)$,

$$
\|\epsilon(t)\|\le R_\epsilon:=
\sqrt{\kappa_6}\|\epsilon(0)\|+
\frac{2p\sqrt{\kappa_6}}{a}\|u_G\|_\infty. \tag{17}
$$

If $|e_\psi(0)|<\delta$ and

$$
b_\psi R_\epsilon+\|u_\psi\|_\infty
<k_\psi v_*\sin\delta, \tag{18}
$$

then at either boundary $e_\psi=\pm\delta$ the yaw derivative points strictly
inward. A first-exit argument establishes invariance, validating the use of
(15) for all $t\ge0$. Equations (16) and (18) are a local ISS result with a
specified disturbance neighborhood. They do not cover standstill or arbitrary
heading errors. In the noise-free case the error converges exponentially.
This proves Theorem G.

### 3.5 If GNSS also independently measures heading

If an actual sensor provides $y_G=Cz+n_G$, with
$C=[e_1,e_4,e_7]^\top$, one may instead use all four extended auxiliary outputs
and the usual continuous observer
$\dot{\hat z}=M\hat z+b+TK_G(y_G-C\hat z)+L_h(y_h-h_{\rm ext}(\hat z))$.
For its scaled matrix
$A_G=T^{-1}MT-\theta K_GC-\theta^{-3}N_GHT$, a constant certificate
$\operatorname{He}(P_GA_G)\preceq-2\rho_GP_G$ gives

$$
\|e(t)\|\le\theta^2\sqrt{\kappa(P_G)}e^{-\rho_Gt}\|e(0)\|
+\frac{\theta^3\sqrt{\kappa(P_G)}}{\rho_G}\|u_G\|_{[0,t],\infty}. \tag{19}
$$

This follows from the Dini derivative of $\sqrt{\epsilon^\top P_G\epsilon}$.
It permits standstill, subject to the bounded-input and lifted-heading
assumptions. Course over ground at zero speed is not such an independent
heading measurement. This is an alternative measurement contract, not the
position-only interface used for Theorem G.

## 4. Mode L: continuous, informative LiDAR with a fixed delay

### 4.1 Measurement and uniform information condition

Let $d_L>0$ be known and constant. Fix the physical pose scales
$S_p=\operatorname{diag}(s_X,s_Y,s_\psi)\succ0$, and express the measured
pose and its noise in these normalized coordinates:

$$
y_L(t)=Cz(t-d_L)+n_L(t),\qquad
C=S_p^{-1}\begin{bmatrix}e_1^\top\\e_4^\top\\e_7^\top\end{bmatrix}. \tag{20}
$$

This output is available continuously. Its timestamp offset is always $d_L$.
The pose at $t-d_L$ and its information matrix belong to the same physical
measurement. No delivery clock is introduced.

Information must be compared in the same dimensionless pose coordinates. Let
$I_L$ be the physical pose information, and
$\bar I_L=S_p^\top I_LS_p$. Require a uniform lower bound

$$
\bar I_L(t)\succeq\iota_*I_3,\qquad\iota_*>0. \tag{21}
$$

A convenient bounded weight is

$$
W=\bar I_L(\bar I_L+\lambda_I I_3)^{-1},\quad\lambda_I>0,
\qquad w_* I_3\preceq W(t)\preceq I_3,\quad
w_*=\frac{\iota_*}{\iota_*+\lambda_I}>0. \tag{22}
$$

This is a spectral matrix function, retaining translation-heading cross terms.
It does not insert an artificial eigenvalue floor. Pointwise nonsingularity
with eigenvalues tending to zero would not meet (21). The information matrix
must have a defensible pose-error interpretation; a registration Hessian by
itself does not establish a deterministic noise bound. For example an
independently justified bound
$n_L^\top\bar I_L n_L\le\eta^2$ implies
$\|n_L\|\le\eta/\sqrt{\iota_*}$, or a physical-noise bound
$\|S_p n_L\|\le\|S_p\|\eta/\sqrt{\iota_*}$.
Otherwise bounded $n_L$ remains a separate assumption.

In the following algebra $W$ acts on the normalized pose residual in (20);
$CT=\theta C$ still holds. The same convention must be used in a numerical
certificate. The equal-unit formulas use $S_p=I_3$.

### 4.2 Continuous delayed observer and exact error equation

Choose

$$
\boxed{\dot{\hat z}(t)=M(q(t))\hat z(t)+b(t)
+TK_LW(t)[y_L(t)-C\hat z(t-d_L)]
+TN_L\theta^{-3}[y_h(t)-h_{\rm ext}(\hat z(t),t)].} \tag{23}
$$

The pose residual compares two states at $t-d_L$. The current auxiliary
outputs still compare current states. Seven differential state coordinates
are retained, together with a continuous history on $[-d_L,0]$.
Retarded equations require a history; they are continuous-time systems, though
their phase space is infinite-dimensional. This is not state replay or
input-flow transport.

Subtracting (23) from (1), and using (5), gives exactly

$$
\boxed{\dot\varepsilon(t)=A_0(t)\varepsilon(t)
+A_d(t)\varepsilon(t-d_L)+u_L(t),} \tag{24}
$$

$$
\begin{aligned}
A_0&=T^{-1}M(q)T-\theta^{-3}N_LHT,\\
A_d&=-\theta K_LWC,\\
u_L&=T^{-1}w-K_LWn_L-\theta^{-3}N_Ln_h.
\end{aligned}  \tag{25}
$$

In particular, the delayed error is not an independent disturbance. Substituting
$C\hat z(t)$ for $C\hat z(t-d_L)$ would add the motion term
$C[z(t-d_L)-z(t)]$ and generally destroy zero-error invariance even in noiseless
constant-velocity motion. That is a different observer.

### 4.3 Constant-matrix Lyapunov--Krasovskii condition

Fix $\theta$, $d_L$, the gains, and a target rate $\rho>0$. Set
$r_d=e^{-2\rho d_L}$. Seek constant matrices
$P=P^\top\succ0$, $Q=Q^\top\succ0$, $R=R^\top\succ0$ and $g>0$.
For every admissible pair $(A_0,A_d)$ require

$$
\boxed{
\begin{bmatrix}
\Pi & d_L\mathcal F^\top R\\
d_L R\mathcal F & -R
\end{bmatrix}\prec0,\qquad
\mathcal F=[A_0\;A_d\;I_7],} \tag{26}
$$

where the $21$-by-$21$ symmetric block is

$$
\Pi=\begin{bmatrix}
\operatorname{He}(PA_0)+2\rho P+Q-r_dR & PA_d+r_dR & P\\
* &-r_d(Q+R)&0\\
*&*&-gI_7
\end{bmatrix}. \tag{27}
$$

All matrices here are constant; the functional's history integrals carry the
delay. This is an LMI in $P,Q,R,g$ for fixed gains, rate and delay. Joint
optimization over gains and these matrices is generally bilinear and must
not be described as this same convex feasibility problem.

If $A_0,A_d$ lie in a verified convex hull, checking (26) at every hull vertex
suffices, because its Schur form is affine in the two coefficient matrices.
The four-vertex $(q,q^2)$ enclosure and the thirteen-entry output box are
valid choices. A full-matrix $W$ sector must also be enclosed, including
all off-diagonal entries. Checking only $W=w_*I$ and $W=I$ does not cover
arbitrary anisotropic matrices. An entrywise outer box centered at
$(1+w_*)I/2$ with halfwidth $(1-w_*)/2$ on its six independent symmetric
entries is valid but conservative. No bound on $\dot W$ or $\dot H$ is needed.
A failed outer-box LMI means this certificate is unavailable; it is not a
proof that the actual system is unstable.

### 4.4 LiDAR ISS theorem and proof

**Theorem L.** Suppose the bounded-model/output hypotheses, (20)--(22), and
(26) hold uniformly along a forward-complete trajectory in a consistently
lifted pose chart. For every continuous initial error history and bounded
$u_L$, observer (23) is exponentially ISS in all seven errors, with respect
to the supremum norm of that history.

**Proof.** For brevity write $d=d_L$ and $x=\varepsilon$. For an absolutely
continuous solution segment, use

$$
\begin{aligned}
V_L(x_t)={}&x(t)^\top Px(t)
+\int_{t-d}^t e^{2\rho(s-t)}x(s)^\top Qx(s)\,ds\\
&+d\int_{-d}^0\int_{t+s}^t e^{2\rho(u-t)}
\dot x(u)^\top R\dot x(u)\,du\,ds.
\end{aligned} \tag{28}
$$

It satisfies $V_L\ge p_-\|x(t)\|^2$, where $p_-=\lambda_{\min}(P)$.
Let $x_d=x(t-d)$. Differentiating the integrals almost everywhere yields

$$
\begin{aligned}
\dot V_L+2\rho V_L={}&2x^\top P\dot x+2\rho x^\top Px
+x^\top Qx-r_dx_d^\top Qx_d+d^2\dot x^\top R\dot x\\
&-d\int_{t-d}^t e^{2\rho(u-t)}\dot x(u)^\top R\dot x(u)\,du.
\end{aligned}  \tag{29}
$$

Since $e^{2\rho(u-t)}\ge r_d$, Jensen's inequality and
$x-x_d=\int_{t-d}^t\dot x(u)\,du$ imply

$$
-d\int_{t-d}^t e^{2\rho(u-t)}\dot x^\top R\dot x\,du
\le-r_d(x-x_d)^\top R(x-x_d). \tag{30}
$$

With $\zeta=[x^\top,x_d^\top,u_L^\top]^\top$, equation (24) gives
$\dot x=\mathcal F\zeta$. Equations (29)--(30) become

$$
\dot V_L+2\rho V_L-g\|u_L\|^2
\le\zeta^\top[\Pi+d^2\mathcal F^\top R\mathcal F]\zeta<0
\quad(\zeta\ne0), \tag{31}
$$

where the strict inequality is exactly the Schur complement of (26).
Integration from any time $t_0$ with an admissible solution segment gives

$$
\|x(t)\|\le
\sqrt{V_L(x_{t_0})/p_-}\,e^{-\rho(t-t_0)}
+\sqrt{g/(2\rho p_-)}\,\|u_L\|_{[t_0,t],\infty}. \tag{32}
$$

To establish ISS in the ordinary continuous-history norm, one must not
bound $V_L(x_0)$ using an unprovided derivative of the initial history.
The following first-interval argument removes that gap. Let

$$
m=\sup_t\|A_0(t)\|+\sup_t\|A_d(t)\|,\quad
M_0=\|x_0\|_{[-d,0],\infty},\quad U_t=\|u_L\|_{[0,t],\infty}.
$$

On $[0,d]$, the integral equation and Gronwall's inequality give
$\sup_{[-d,d]}\|x\|\le e^{md}(M_0+dU_d)$ and
$\|\dot x\|_{[0,d],\infty}\le m e^{md}(M_0+dU_d)+U_d$.
With $p_+=\lambda_{\max}(P)$, $q_+=\lambda_{\max}(Q)$,
$r_+=\lambda_{\max}(R)$, define

$$
\begin{aligned}
b_0&=\sqrt{p_++dq_+},\qquad b_1=\sqrt{d^3r_+/2},\\
C_0&=e^{md}(b_0+mb_1),\qquad C_1=dC_0+b_1,\\
A_L&=e^{\rho d}\max\{e^{md},C_0/\sqrt{p_-}\},\\
B_L&=\max\{de^{md},C_1/\sqrt{p_-}+\sqrt{g/(2\rho p_-)}\}.
\end{aligned}  \tag{33}
$$

The weights in (28) are at most one; the triangular double-integral area is
$d^2/2$. Thus $\sqrt{V_L(x_d)}\le C_0M_0+C_1U_d$, where $x_d$ here denotes
the history segment at time $d$. Applying (32) from $t_0=d$, and using the
first-interval estimate for $0\le t<d$, proves

$$
\|x(t)\|\le A_Le^{-\rho t}M_0+B_LU_t,\qquad t\ge0. \tag{34}
$$

Finally $\|T\|=\theta^3$ and $\|T^{-1}\|=\theta^{-1}$. Returning to the
physical seven-state error yields

$$
\boxed{\|e(t)\|\le\theta^2A_Le^{-\rho t}
\|e_0\|_{[-d_L,0],\infty}
+\theta^3 B_L\|u_L\|_{[0,t],\infty}.} \tag{35}
$$

The disturbance on the right is external and bounded by

$$
\|u_L\|\le\theta^{-1}\|w\|+
\|K_L\|\|n_L\|+\theta^{-3}\|N_L\|\|n_h\|. \tag{36}
$$

Using the structure $w=Bd_j-e_7\nu_r$ improves its first term to
$\theta^{-3}\|d_j\|+\theta^{-1}|\nu_r|$. Zero external disturbances yield
exponential convergence from any admitted history. This proves Theorem L.

The heading qualification is explicit: pose yaw must be continuously lifted
on a consistent branch, or the result applies only while a valid local chart
is maintained. It does not prove global stability on the circle for arbitrary
principal-value yaw representatives or cycle slips. For GNSS Theorem G the
stronger local restriction (18) is also required.

### 4.5 Why delay and gain must be checked together

Even the scalar continuously measured heading channel

$$
\dot e_\psi(t)=-\ell e_\psi(t-d_L),\qquad\ell>0, \tag{37}
$$

is exponentially stable only for $0<\ell d_L<\pi/2$.
Indeed the characteristic equation is $s+\ell e^{-sd_L}=0$; its first
imaginary-axis crossing is $s=i\ell$, $\ell d_L=\pi/2$.
At the boundary the nondecaying solution $e_\psi(t)=\sin(\ell t)$ already
contradicts ISS with zero input. The crossing is toward the right half-plane
as delay increases. This elementary special case disproves any claim that
full information permits arbitrary fixed delay or arbitrarily large gain.

Condition (26) is a sufficient delay-dependent test and need not recover
this exact scalar limit. A value such as the earlier runtime's 150 ms is
certified only after the new inequalities are checked with the new gains,
actual information sector, and operating bounds. The historical gain files
supply no such conclusion. Small-delay feasibility follows by continuity
from a strict delay-free certificate: choose $R$ large enough to absorb the
cross term in $x-x_d$, then choose $Q$ sufficiently small and $g$ sufficiently
large. The remaining positive term $d^2\mathcal F^\top R\mathcal F$ is a
small perturbation for sufficiently small $d>0$. This establishes a nonempty
small-delay range whenever a strict delay-free common certificate exists;
it does not specify its numerical upper endpoint.

## 5. Cascade interpretation and design procedure

If the upstream interface error $\eta$ is ISS in exogenous disturbances $d_u$,
and $w,n_h$ obey bounds of the form
$\|[w,n_h]\|\le\alpha_1(\|\eta\|)+\alpha_2(\|d_u\|)$ on the stated envelope,
then (16) or (35), combined with the upstream estimate, gives cascade ISS.
For example an exponentially decaying upstream transient enters the downstream
variation-of-constants integral as a convolution of decaying exponentials;
this also decays, including the equal-rate case $te^{-\rho t}$.
The remaining term is a class-$\mathcal K$ function of the exogenous input
supremum. No small-gain feedback condition is needed because the connection
is one-way. GNSS retains its local invariant-domain restriction.

A complete numerical design follows this order:

1. Declare the actual GNSS output (position alone, or independently measured
   position and heading) and fix the physical state/pose units.
2. Declare bounded $q$, velocity, acceleration and all upstream interface
   residuals. For position-only GNSS, declare $v_*$, $\delta$ and the admitted
   initial-error/input set.
3. For GNSS position, impose the triangular gains (8), verify (10), select
   $c$ by (14), and verify (18). A general unsigned heading-Jacobian box
   cannot replace this observability argument.
4. For LiDAR, establish the uniform three-direction information bound and
   pose-noise contract. Select $d_L$, $\theta,K_L,N_L$ jointly as a design
   choice, then verify (26) for the entire uncertainty enclosure using
   constant $P,Q,R$ and a strictly positive margin.
5. Report the resulting rate and ISS constants. A solver success flag,
   frozen-system eigenvalues, sampled weight directions, or numerical
   trajectories alone are not exhaustive robust verification.

No continuous-mode switching theorem, certificate for missing measurements,
or certificate for the old event-based runtime follows from these results.
The two theorems prove the continuous measurement contracts stated above.

## 6. Sources and verification scope

The improved model and mode-specific proofs above are project derivations.
The multi-output scaling and incremental-output construction build on
Bessafa et al., Section 4, equations (41)--(58), especially Theorem 5.
The supplied local PDF was inspected; its six-state observer is not asserted
to contain the new seven-state GNSS or delayed-LiDAR theorem. The constant-matrix
history-functional method follows the Lyapunov--Krasovskii framework described
by Fridman; equations (28)--(35) are derived explicitly here.

- H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani,
  *Generalized multi-output high-gain observer with application to ego vehicle
  trajectory and orientation estimation*, Automatica 188 (2026), 112915.
  [DOI](https://doi.org/10.1016/j.automatica.2026.112915);
  [author institution record](https://eprints.soton.ac.uk/509851/).
- E. Fridman, *Tutorial on Lyapunov-based methods for time-delay systems*,
  European Journal of Control 20(6) (2014), 271--283.
  [DOI](https://doi.org/10.1016/j.ejcon.2014.10.001).

No supplied paper PDF is redistributed. The numerical checks accompanying
this revision are documented in
[the validation record](research/continuous_observer_iss_20260913/validation.md).
They distinguish a verified conservative parameter neighborhood from certification
of the full operating and information sectors used by the older runtime.


### Structured MnCAV numerical enclosure (2026-09-14)

The new `mncav` LiDAR profile verifies the same Schur-form constant-functional
condition over the rectangle enclosing `(q,q^2)` in
`[-0.4,0.4] x [0,0.16]`, instead of absorbing the turn terms into an
unstructured norm ball around zero. Affinity of the Schur block in the drift
makes four common-matrix vertex checks sufficient for the whole rectangle.
The auxiliary Jacobian and information variations retain the previous
analytic norm perturbation bound. This gives a verified positive uniform
margin at theta=2 and delay 0.15 s without imposing a turn-rate clamp.
The exact convex weights, residual bound, stored matrices, numerical checks
and wider-turn MnCAV experiments are in the
[structured certificate validation](research/mncav_lidar_envelope_20260914/validation.md).
This is a less conservative numerical realization of Theorem L; all its
continuous-measurement and physical/disturbance hypotheses remain applicable.
