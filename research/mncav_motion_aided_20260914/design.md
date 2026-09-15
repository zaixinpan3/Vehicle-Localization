# Motion-aided seven-state localization: structure and gain design

## Material passport

Date: September 14, 2026. Mode: engineering research and experiment execution.
Material: local Bessafa and Gao papers, the current MATLAB implementation,
precomputed recorded MnCAV matching, and explicitly synthetic plant runs.
Decision: adopt a separate, zero-delay motion-aided output structure with
physical gains and its own conditional stability argument. This is not a
claim that changing the old preset alone solves fusion or that the original
delayed-HGO theorem automatically certifies the new observer.

## Source review and design alternatives

Bessafa, Delattre, Belkhatir, Zemouche and Rajamani,
*Generalized multi-output high-gain observer with application to ego vehicle
trajectory and orientation estimation*, Automatica 188, 112915 (2026),
[DOI](https://doi.org/10.1016/j.automatica.2026.112915), was read locally.
Equations (41)--(49) describe pose and auxiliary-output injections and their
gain scaling. Theorem 5 and (55)--(58) establish an ISS bound. Remark 8
explicitly discusses the convergence/noise tradeoff. Section 5.1 imposes an
additional gain-related constraint so the auxiliary measurements influence
the example. These support designing both measurement channels and do not
establish a lower recorded RMSE than arbitrary pose measurements.

Our old LiDAR implementation fixes K and N from JSON and optimizes certificate
matrices. Its recorded N=0 control changes position by at most 0.115 mm.
Its local position-noise transfer has a gain above one. Those are properties
of the old implementation/preset, not a disproof of the paper's method.

The distinction between fast reconstruction and measurement-noise rejection
is also explicit in Ahrens and Khalil, *High-gain observers in the presence
of measurement noise: A switched-gain approach*, Automatica 45, 936--943
(2009), [DOI](https://doi.org/10.1016/j.automatica.2008.11.012).
Switching gains can address that tradeoff, but switching was not selected
here: the position residual would still enter derivative states, and a new
switching proof and thresholds would be necessary.

Gao, Xiong, Xia, Lu, Yu and Khajepour, *Improved Vehicle Localization Using
On-Board Sensors and Vehicle Lateral Velocity*, IEEE Sensors Journal 22(7)
(2022), [DOI](https://doi.org/10.1109/JSEN.2022.3150073), was also read locally.
Its introduction and velocity measurement construction use longitudinal and
estimated lateral velocity to correct localization. That motivates retaining
signed velocity components instead of using only quadratic invariants. The
Kalman filter in that paper is not implemented or claimed as this design.

Alternatives considered:

| Alternative | Decision and evidence |
|---|---|
| Increase the old high-gain scalar | Previous recorded controls do not improve input-relative RMSE and increase derivative excursions. |
| Jointly optimize the old K/N with all invariant-output Jacobians | Plausible future approach, but strong quadratic-output feedback complicates robustness to model errors and scaling. No successful joint solution is claimed here. |
| Switch or adapt the old gains | Requires additional switching/stability design; deferred. |
| Replace localization with a KF or smoother | Would change the requested observer framework and require noise/correlation calibration; not selected. |
| Direct body-motion correction in the existing seven-state model | Selected and executed: a simple cascade admits an explicit common quadratic bound and separates position noise from derivative-state correction. |
| Return or blend the raw pose to pass the baseline | Not used; all reported new outputs are integrated observer states. |

## Equations and measurement roles

Let p=(X,Y), v=(Vx,Vy), a=(Ax,Ay), and psi be the same seven states as
before. Let J=[[0,-1],[1,0]], R(psi) be the planar rotation, and
q=r_m+betaDot_hat. Body measurements are
b_v=(u_m,vy_hat) and b_a=(ax_m,ay_m). Acceleration must represent compensated
inertial acceleration at the same vehicle reference point. Its reference-point
and bias uncertainties remain disturbances, not newly established calibration.

The implemented equations are

\[
\begin{aligned}
\dot{\hat\psi}&=r_m+k_\psi w_\psi(\psi_L-\hat\psi),\\
\dot{\hat p}&=\hat v+k_pW_p(p_L-\hat p),\\
\dot{\hat v}&=\hat a+k_v\{R(\hat\psi)b_v-\hat v\},\\
\dot{\hat a}&=q^2\hat v+2qJ\hat a+k_a\{R(\hat\psi)b_a-\hat a\}.
\end{aligned}
\]

The global prediction q^2*v+2*q*J*a is retained. Variable speed and turn rate
are allowed; omitted jerk/course-rate-derivative terms remain model
disturbances. No vehicle speed or yaw rate is clamped. A rate above the
declared envelope is reported as a certificate-applicability failure.

Pose feedback no longer changes velocity and acceleration directly. Their
signed body measurements provide substantive correction. Heading is estimated
from gyro plus LiDAR yaw and is used for rotation; no GNSS/INS reference yaw
enters the runtime. The old four invariant residuals are replaced by these
direct outputs. This is a changed nonlinear injection structure, not merely
a new numerical value of the old N matrix.

At each knot, the full geometric information matrix gives
W=I_geo*(lambda*I+I_geo)^(-1), with lambda=.001 and physical unit pose scales.
W_p=W(1:2,1:2) and w_psi=W(3,3). Position-heading feedback cross terms are
omitted to preserve the heading-to-motion cascade; translation anisotropy
and its off-diagonal term remain. This does not assume independent statistical
errors and does not turn a registration normal matrix into calibrated
covariance. Qualified knot weights, rather than information matrices, are
linearly interpolated inside this runner. Convex interpolation preserves
w_min*I <= W <= I everywhere, with w_min=.25.

Initial pose is the first LiDAR measurement. Initial v/a are the corresponding
rotated body measurements. A caller may supply a different seven-state initial
estimate. The lifted LiDAR yaw and initial heading must use the same chart;
only displayed output yaw is wrapped. There is no discrete reset, pose
postprocessing, output blend, reference correction or delayed-history replay.

## Conditional stability certificate

Use dimensionless coordinates defined by reference units 1 m and 1 s, so the
numerical state entries match the stored physical units. With errors ordered
as (e_p,e_v,e_a), collect rotated measurement errors, heading coupling and
model error into d. The homogeneous translation system is

\[
\dot e=A(q,W_p)e+d,\quad
A=\begin{bmatrix}
-k_pW_p&I&0\\0&-k_vI&I\\0&q^2I&-k_aI+2qJ
\end{bmatrix}.
\]

For V=e' e, the skew part cancels. Cauchy--Schwarz gives

\[
\dot V\le -\xi^T M(q^2)\xi+2\|e\|\|d\|,
\quad \xi=(\|e_p\|,\|e_v\|,\|e_a\|),
\]

\[
M(s)=\begin{bmatrix}
2k_pw_{min}&-1&0\\
-1&2k_v&-(1+s)\\
0&-(1+s)&2k_a
\end{bmatrix}.
\]

M is affine in s. Positive definiteness at s=0 and s=q_max^2 proves a
uniform bound for arbitrary time-varying q in the interval, without bounding
qDot. If mu is the minimum endpoint eigenvalue, the homogeneous translation
decay rate is at least mu/2. The forced norm obeys

\[
\|e(t)\|\le e^{-\mu t/2}\|e(0)\|
+\int_0^t e^{-\mu(t-\tau)/2}\|d(\tau)\|d\tau.
\]

Lifted heading error satisfies a scalar stable equation with decay at least
k_psi*w_min and gyro/yaw disturbances. Since R is Lipschitz in its angle and
true body motion is bounded, heading error contributes a bounded input to
translation. This yields a conditional ISS cascade. Neither sensor-error
independence nor a stochastic covariance model is required for that bound.
The lateral observer must supply bounded errors for a full physical cascade
claim; its matrix synthesis alone does not verify all physical hypotheses.

For gains [4,4,12,4], q_max=.4 rad/s and w_min=.25:
mu=1.83614987994, translation decay bound=.918074939968 /s,
heading decay bound=1 /s. `designMotionAidedObserverGains` computes both
endpoint matrices and verifies eigenvalues for every supplied gain set.
An independent test checks the actual 6-by-6 symmetrized dynamics at 41
course rates and 11 anisotropy orientations. The previous P/Q/R delay
certificate is neither reused nor labeled applicable to this structure.

## Noise-response change and its limit

For constant W_p=I and fixed external motion outputs, position-noise transfer
is H_L(s)=k_p/(s+k_p). Unlike the prior position-driven triple integrator,
its magnitude never exceeds one. At .3587165 Hz with k_p=4, the ideal
magnitude is .87121399036. The actual runner gives .87117710925; the
4.23e-5 relative difference is consistent with .01 s linear reconstruction
of the input sinusoid. The previous runner's local peak at this frequency
was approximately 1.3546. This is a controlled one-channel frequency result,
not a full nonlinear trajectory transfer or a universal performance bound.

Motion errors still affect position: at zero course rate, for example,
constant velocity measurement bias can produce approximately bias/k_p
position error. A perfectly clean stationary LiDAR signal paired with a
false 1 m/s speed yields .24881 m position RMSE in the explicit 40 s
counterexample, while direct LiDAR error is zero. Universal pointwise or
all-dataset dominance is therefore not claimed. The meaningful acceptance
claim concerns the stated recorded data and synthetic conditions.

## Gain selection and reproducibility

`designMncavMotionAidedGains` actually searches physical gains:
kp in [3,4,5,6,8,10], kv in [1,4,8], ka in [4,12],
kpsi in [.5,1,2,4]. Every one of the 144 candidates is checked for the
new certificate and run on t<=60 s. Feasibility additionally requires
position RMSE, position maximum, P95 and heading RMSE no larger than the
actual continuous LiDAR input on that design interval. Nine candidates pass;
the minimum-position-RMSE winner is [4,4,12,4]. No reserve-segment metric
enters the selector. All candidate metrics are retained.

Before the formal implementation, 12 initial temporary probes and a 144-point
unit-weight design-only probe explored the structure. The selected constrained
winner was evaluated once on the remainder; the production runner then
reproduced it using actual weights and a smaller integration step. This is
exploratory research, not preregistered evidence. The whole drive had been
examined in earlier diagnostic tasks, so the >60 s interval is reserved from
this numerical gain selection but is not a never-seen independent drive.
The mixed-ODOM P95 exception was retained without tuning to remove it.
