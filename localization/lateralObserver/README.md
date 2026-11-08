# Yaw-rate-only lateral observer

The two states are lateral velocity and yaw rate, `x = [vy; r]`. Steering
is the model input. Measured yaw rate is the only innovation:

$$
\dot{\hat x}=A(V_x)\hat x+B\delta+f(\hat x)+L(V_x)(r_m-[0,1]\hat x).
$$

`runLateralVelocityObserver` requires aligned `time`, `steeringAngle`,
`longitudinalSpeed`, `longitudinalAcceleration` and `yawRate`. It does not
read `lateralAcceleration`, even if that extra field is supplied. Longitudinal
specific force is used only to reconstruct speed rate for the algebraic
sideslip-rate interface. Output-point transport, sideslip angle/rate,
`velocityValid`, and the two-state integration interface are retained.

The global observer continues to consume both body-acceleration channels.
This change removes the lateral observer's dependency on the lateral
accelerometer; it does not add a bias estimator to either observer.

## Gain design

Use `designLateralObserverGains(lateralObserverConfig())`. The adopted MnCAV
configuration remains the default. Re-synthesize for each changed vehicle
or measurement model; old two-output gains are rejected.

The matrices `A` and `L` are affine on the scheduling triangle enclosing
`[Vx; 1/Vx]` for `Vx` in the configured interval. Since `C = [0,1]` is
constant, `A-L*C` is affine without any extension or gain constraint.

One common `P >= I` and `Y_i = P*L_i` give three 7-by-7 LMIs:

$$
\begin{bmatrix}
\operatorname{He}(PA_i-Y_iC)+2\kappa P+\tau\gamma_f^2I&S_i&P\\
S_i^\top&-\gamma^2I_3&0\\
P&0&-\tau I_2
\end{bmatrix}\prec0,
\qquad S_i=[PEW_w,-Y_i\sigma_r].
$$

When `gamma_f = 0`, omit the last row and column: the exact linear
certificate has three 5-by-5 LMIs and no Young multiplier (`design.tau = 0`).
Otherwise the positive Young multiplier `tau` is a decision variable. First minimize
`gamma^2`; then minimize gain-norm bounds subject to the relaxed ISS bound.
There are no parameter-dependent metric, scheduling-speed derivative bound,
Finsler slack or scalar candidate lists. The common metric can be conservative;
feasibility is checked by the configured solver rather than assumed.

For the proof, blend the vertex inequalities with the nonnegative barycentric
coordinates of `[Vx; 1/Vx]`. Every block is affine, so the result is negative
definite at every admissible speed. Schur-complementing the last block adds
`P^2/tau` to the first block. The Lipschitz inequality implies

$$
2e^\top P\Delta f\le\tau\gamma_f^2\|e\|^2+\tau^{-1}e^\top P^2e.
$$

Thus, with normalized force/gyro disturbance `d` and `V = e'*P*e`,

$$
\dot V\le-2\kappa V+\gamma^2\|d\|^2,\qquad
\|e(t)\|\le\sqrt{\lambda_{\max}(P)}e^{-\kappa t}\|e(0)\|
+\frac{\gamma}{\sqrt{2\kappa}}\|d\|_{\infty,[0,t]}.
$$

For `gamma_f = 0`, the nonlinearity difference vanishes and the reduced
5-by-5 inequality gives the same dissipation bound directly. Because `P`
is constant, no scheduling derivative appears in either proof.

A bounded measurement bias contributes to the ISS error bound; it is not identified or
cancelled. Sampled integration and operation outside the speed interval are
not covered by the continuous theorem. Low-speed states are held and their
aiding is withdrawn.

## Verification

The class-based tests `tests/lateralObserverTest.m` and
`tests/lateralObserverIntegrationTest.m` check missing/ignored lateral
acceleration, scalar innovation, current-certificate rejection, all three
vertex inequalities, nominal convergence, axle-force mismatch, algebraic
transport and numerical integration. Tests synthesize gains from the current
configuration. The synthetic cascade entry is
`scripts/validateSyntheticLocalizationCascade.m`; its acceleration channels
remain available to the global observer.
