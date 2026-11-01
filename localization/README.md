# Vehicle localization

The maintained estimator is a one-way cascade: the lateral LPV observer
supplies motion outputs to the intrinsic eight-state global observer, whose
LiDAR channel evaluates semantic overlap gradients. There is one production
global-observer runtime, `runFullLocalizationObserver`.

This is the module's runtime and current gain-design guide. The nominal fusion
construction below supersedes the L/G/M certificates, polynomial boxing,
refinement and computed-region sections in the
[historical observer derivation](../improved_observer_derivation.md).

## Entry points

| Stage | Entry point |
| --- | --- |
| Vehicle and replay parameters | `mncavVehicleConfig`, `mncavReplayConfig` |
| Wheel/IMU/steering preparation | `prepareMncavMotionInputs` |
| Lateral gain synthesis | `designLateralObserverGains` |
| Lateral execution | `runLateralVelocityObserver` |
| Global configuration | `mncavFullObserverConfig`, `mncavSmallOffsetObserverConfig` |
| Fresh global gain synthesis | `synthesizeMncavInvariantDesign` |
| Frame-clock alignment | `localizationSupport.synchronizeLocalizationInputs` |
| Global execution | `runFullLocalizationObserver` |
| Current LiDAR measurement | `localizationSupport.evaluateOverlapGradient` |

Current dataset runners are `runMncavFullObserverExperiment`,
`replayDowntownGnssFreeLocalization`, `runNuscenesOverlapTrial`,
`runKittiOverlapTrial`, and `runMncavVdbLocalization`. Their initialization,
sensor availability and mapping/reference protocols are experiment-specific.
Covariance-filter, source-observer and FAST-LIO2 comparisons live separately
under [scripts/baselines](../scripts/baselines/README.md).

## State and sensor interfaces

The internal global state is `[X,VX,AX,Y,VY,AY,psi,r]`. Public `.z` and
`.onlineZ` retain seven columns for position, velocity, acceleration and
heading; `.internalState` and `.yawRate` expose the estimated rate. The
seven-column output is not a seven-state observer branch.

Intrinsic prediction uses the estimated rate `r`, upstream sideslip-rate
estimate and internal states. Raw IMU measurements enter the auxiliary
outputs and upstream lateral observer, not global prediction. The five
auxiliary outputs are squared speed, velocity-acceleration dot product,
velocity-acceleration cross product, course direction and yaw rate.

The lateral module directly exports its LPV velocity/rate state, with
algebraic output-point transport and a separate angle/rate filter. It has no
inertial master, bias, stationary/crawl correction, filtered LPV injection or
shadow velocity estimator. Its positive-speed design interval is [1,30] m/s.
Outside that interval, or on invalid dynamic input, the LPV state is held
and `velocityValid=false`; modes are `dynamic` and `unavailable`.

The global observer reads the supplied `lateralVelocity`, `sideSlipAngle`
and `sideSlipAngleRate`; it does not reconstruct the angle with
`atan2(vy,vx)`. On unavailable velocity it withdraws auxiliary rows 1--4 and
the upstream sideslip-rate prediction term. Missing acceleration withdraws
rows 2--3. GNSS, LiDAR and gyro corrections retain their own availability.
Do not interpret held lateral states as valid low-speed estimates.

Use the output-point conventions and current vehicle/replay configuration in
[config/MNCAV_CONFIGURATION.md](../config/MNCAV_CONFIGURATION.md). Rebuild
inputs if their vehicle, clock or calibration metadata is stale.

## Fresh gains and stability scope

```matlab
setupVehicleLocalization;
cfg = mncavSmallOffsetObserverConfig();
% Apply this experiment's sensor, geometry and domain overrides before solving.
[design, cfg] = synthesizeMncavInvariantDesign(cfg);
lc = lateralObserverConfig();
ld = designLateralObserverGains(lc);
lateral = runLateralVelocityObserver(data.highRate, ld, lc);
[data, lateral] = localizationSupport.synchronizeLocalizationInputs(data, lateral, cfg);
estimate = runFullLocalizationObserver(data, ld, cfg, LateralInputs=lateral);
```

Gain synthesis requires YALMIP and SeDuMi. Each experiment solves fresh gains;
there is no saved-gain fallback. A single SDP selects a constant metric and
matched feedback gains, including the nominal motion-response constraint and
primitive sensor-noise objective. The scalar gyro LMI has a closed-form
optimum and requires no separate optimization. Every returned fusion and
response vertex is checked again numerically, including at runtime without
an SDP solver. Old configurations and gains are rejected.

The default domain is [10,15] m/s. The small-offset scenario explicitly uses
[1,30] m/s and hypothetical GNSS geometry. The certificate covers nominal
simultaneous LiDAR, GNSS and all five auxiliary outputs. It no longer certifies
switching among L/G/M sensor-adequacy families. Dataset configurations with
nominal GNSS weight zero use the same LMI for their declared LiDAR-only pose
source. Runtime withdrawal of invalid packets remains available.

### Single convex design

Let `t=theta=sigma`, `s0` be the minimum course speed, `V` the maximum speed,
`A` the acceleration bound, `Q` the course-rate bound, and `ell` the heading
length. The normalized state scaling remains
`B=diag(t.^[1,2,3,1,2,3,1,2]./[1,1,1,1,1,1,ell,ell])`.
The matched squared-speed and dot/cross weights are now synthesized directly
as `c1,c2` and applied with the single bounded factor
`V^2/(s0^2+sMeasured^2)`. Direction has the constant weight `c3`. This uses the
existing squared-speed measurement and is finite at rest; there is no
baseline gain plus a separately optimized augmentation.

Use the metric

\[
P=\operatorname{diag}(P_c,P_c,P_h),\qquad
P_c=U^T\operatorname{diag}(p,R)U,\qquad
U=\begin{bmatrix}1&-1&1\\0&1&0\\0&0&1\end{bmatrix}.
\]

Then `(inv(P))(2:3,2:3)=inv(R)`. This fixed coordinate shear is an explicit
structural restriction, not a claim that every unrestricted metric has this
form. It makes the motion Schur metric `R` affine in the same solve while
retaining pose-to-motion feedback. Including acceleration in this shear
restores the weak-LiDAR standstill recovery test without another solve. LiDAR and GNSS share one normalized pose
gain, `kappaL=kappaG`, so the nominal noise objective cannot switch off GNSS
by driving its independent gain to zero. Sensor information weights and
noise scales remain distinct. There is no penalty-weight search.

At a course-aligned nominal state, write `v=[s,0]`, `a=[b,s*q]`, with
`s0<=s<=V`, `abs(q)<=Q`, and `b^2+s^2*q^2<=A^2`. Set `d=s0^2+s^2`.
The affine coefficient vector is

\[
f=(q,q^2,b,s^2/d,sb/d,s^2q/d,s,s^2).
\]

Its analytic intervals are

| Coefficient | Interval |
| --- | --- |
| `q` | `[-Qe,Qe]`, `Qe=min(Q,A/s0)` |
| `q^2` | `[0,Qe^2]` |
| `b` | `[-A,A]` |
| `s^2/d` | `[1/2,V^2/(s0^2+V^2)]` |
| `sb/d` | `[-A/(2*s0),A/(2*s0)]` |
| `s^2*q/d` | `[-min(Qe,A/(2*s0)),min(Qe,A/(2*s0))]` |
| `s` | `[s0,V]` |
| `s^2` | `[s0^2,V^2]` |

The mixed bounds use `s/(s0^2+s^2)<=1/(2*s0)` and `abs(s*q)<=A`.
The diagonal velocity term of the scheduled dot/cross Gram is nonnegative;
omitting it gives a Loewner lower bound while retaining all cross terms.
`fusionMatrices` is affine in these eight coefficients. Checking all 256
Cartesian vertices therefore bounds every physical nominal matrix; degenerate
intervals reduce the count. These are coefficient vertices, not sampled
motion endpoints. No Bernstein controls, partitions, acceleration multipliers
or cutting-plane iterations remain.

The single fusion condition is

\[
\operatorname{He}(P A(f)-c_1 S(f)-c_2 B(f)-c_3 D(f))
+L_{\rm pose}+\lambda I\prec0
\]

at every vertex. Here `S,B,D` are the squared-speed, lower dot/cross and
direction matrices. For pose selector `C`, translation selector `Cp`, and
heading unit vector `e7`, the common pose term is

\[
L_{\rm pose}=-2\kappa C^T S_L C-\kappa w C_p^T C_p
+\kappa w\rho^2 e_7e_7^T,
\quad \rho=\|\ell_{GNSS}\|/\ell.
\]

`S_L=diag(pmin,pmin,hmin)-(couplingBound+operatorMismatch)*I` is the
informative-LiDAR lower bound. A fixed Young inequality bounds GNSS lever
orientation for nominal **isotropic** information `w*I`. This does not certify
arbitrary anisotropic GNSS weights or actual overlap operators.

For the motion-only subsystem, the same solve imposes

\[
\operatorname{He}(R A_m)+2\alpha R
-2t\left(c_1\begin{bmatrix}r&0\\0&0\end{bmatrix}
+c_2\begin{bmatrix}0&u/t\\u/t&r\end{bmatrix}\right)\prec0,
\quad A_m=\begin{bmatrix}0&t\\0&0\end{bmatrix},
\]

at the four `(r,u)` interval vertices. The omitted acceleration-dependent
velocity diagonal is again nonnegative. `alpha=1/timeConstantS`; the default
0.5 s target preserves braking/restart response under this more conservative
coefficient enclosure. It is a nominal straight-course velocity/acceleration
contraction target with exact packets and no pose correction, not a nonlinear
full-pose settling-time promise. It is imposed together with the existing
self-feedback caps, `P>=I`, and the full primitive-column noise epigraphs.

The same solve imposes three affine component-response bounds, with
`pL=S_L(1,1)`, `hL=S_L(3,3)`, and `sbar=(s0+V)/2`:

\[
\kappa p_L\ge(\alpha/t)P_{11},\qquad
\kappa h_L\ge(\alpha/t)P_{77},\qquad
c_3(\bar s/(t\ell))^2\ge(\alpha/t)P_{77}.
\]

Since `(inv(P))(j,j)>=1/P(j,j)`, these give sufficient direct correction
rates for LiDAR position, LiDAR heading and direction-heading at the nominal
midpoint speed. They prevent the noise objective from suppressing useful
pose response. They use the same configured time target, with no new tuning
weights. They are component-authority constraints, not a full-pose settling
theorem or another sensor-availability certificate.

### Nonlinear scope and the source paper

Bessafa et al., *Generalized multi-output high-gain observer with application
to ego vehicle trajectory and orientation estimation*, Automatica 188 (2026),
112915, Lemma 1 (p. 2), supplies the differential-mean-value representation
and affine-polytope convexity principle. `secants` retains exact finite drift
and output identities. The implemented vertex LMI bounds the **nominal
zero-error Jacobian family** of our matched-gradient observer; it must not be
mistaken for a bound on every finite product `Hhat'*Hsec`. The local compact-set
assumption and strict nominal margin support a conditional sufficiently-small
error/disturbance argument, not a certificate for arbitrary initial errors.

Following Remark 3 (p. 5), nonlinear use assumes a compact invariant operating
set where the relevant functions are Lipschitz. This assumption does not prove
invariance. No Hessian remainder radius, Lyapunov sublevel or disturbance-budget
radius is computed. `operatingSet.assumedInvariant=true`,
`operatingSet.invarianceVerified=false`, and
`numericalInvariantRadiusCertified=false` express this distinction. There is
also no certificate for actual maps, arbitrary outages, the full lateral
cascade or sampling. Zero speed has defined runtime behavior but remains
outside the positive-speed theory. With every velocity output withdrawn,
pose-only velocity recovery is slower: the regression keeps its 0.02 m
position and 0.05 m/s velocity accuracy targets at 20 s instead of 12 s.
The original braking, GNSS-only heading, standstill-overlap and moving-overlap
accuracy checks remain. None of these sampled examples reinstates an outage
theorem.

## LiDAR runtime

Current dataset inputs use `data.lidarOverlap`. The runner predicts and
applies GNSS/invariant corrections, evaluates the overlap gradient once at
that state, then applies an implicit Lyapunov-matched LiDAR correction.
There is no online pose optimizer or registration callback. The current
runtime uses the common localization clock; it does not propagate delayed
registered poses or select a historical transported implementation.

`overlapGradientConfig` selects the scale ladder and aggregation; its default
is the class-balanced frame overlap at bandwidths `[0,0.5,1,2]` m.
`semanticGeometryConfig` supplies geometry settings. Landmark likelihood and
orientation options are alternative measurements of this same observer,
not alternate observer runtimes. Information availability, supported pose
rank and independent heading strength are distinct diagnostics.

`localizationSupport.updateLocalizationSourceWindow` forms a causal five-scan feature horizon
using uninterrupted valid wheel/lateral/gyro motion. When motion is
unavailable it discards history transport and emits the current scan.
Reference-built frozen maps and one declared initial reference pose are
permitted design inputs. Later reference states must remain outside online
prediction, motion preparation and corrections. Same-pass map/reference
scores do not establish independent survey accuracy.

## Support files

Only two MATLAB files live at this directory level:

| File | Role |
| --- | --- |
| `runFullLocalizationObserver.m` | Public eight-state observer entry and synchronous runtime |
| `localizationSupport.m` | Static methods for design, verification, sensor preparation, geometry, source windows and calibration |

The support class groups methods by responsibility. Current design helpers
include `invariantInputs`, `fusionVertices`, `fusionMatrices`, `fusionBlocks`,
`verifyInvariantDesign`, and `verifyMotionResponse`. Removed solver machinery
is recoverable from Git history.
