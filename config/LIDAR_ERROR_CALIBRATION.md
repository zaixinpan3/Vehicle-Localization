# Conditional LiDAR error calibration

The calibrated profile uses the existing seven-state observer, continuous
ISS LMIs, and implicit numerical update. It changes the weighting of the
LiDAR pose innovation. It adds no prediction-bias estimate, velocity feedback,
state reset, or second gain-design route. Nominal and fixed-scale profiles
remain reproducible experimental controls.

## Physical meaning and equations

The registration Hessian is a geometric normal matrix, not an inverse pose
covariance. The calibrated channel instead predicts two uncentered error
second moments: horizontal squared error per axis (m^2) and heading squared
error (rad^2). Uncentered moments retain systematic matching error; the
observer never subtracts an estimated bias from the measurement.

For residual clusters belonging to matched distributions, evaluate the
residual and Jacobian at the **fitted measurement anchor**, then calculate

```
H = J' J
u_i = J_i' r_i
R_shape = H^dagger (sum_i u_i u_i') H^dagger
f = [trace(R_shape(1:2,1:2))/2; R_shape(3,3)]
r_hat = intercept + slope .* f
```

`R_shape` is only a predictor. Its clusters can remain correlated; residual
scatter cannot identify every common map/association error. Nonnegative
coefficients are fitted to actual estimate-minus-reference measurement errors.
The positive intercept represents errors invisible to the residual scatter.
Multiplying the complete matching objective by a positive constant leaves
this sandwich predictor unchanged. Evaluating scatter at the observer's
predicted pose would instead confuse a large innovation with noisy matching;
the implementation deliberately evaluates it at the fitted anchor.

Let `r_bar` be the empirical mean squared measurement error in the fit data,
in the same physical units. The bounded inverse-variance weights are

```
w = min(1, r_bar ./ r_hat)
B = U_supported * diag(sqrt(upstream_reliability)) * U_supported'
S = B * diag([w_position, w_position, w_heading]) * B
F = S * H^dagger
```

This is an explicitly chosen bounded inverse-variance weighting, not a
claim of a unique Bayesian optimum. The normalization is measured from the
declared calibration population, rather than an arbitrary Hessian magnitude
such as 16. Average-or-better predicted accuracy receives full supported
authority; worse accuracy receives less. The physical base rates `kp` and
`kpsi` still determine bandwidth and must be selected within the continuous
LMI constraints. The algorithm is not claimed to have no design choices.

`S` is symmetric PSD with eigenvalues at most one, preserves the admitted
nullspace, and satisfies `F H = S` to numerical precision. In general `F` is
not symmetric; symmetrizing it would break this identity when position and
heading have different weights. The continuous injection and existing
implicit contraction construction therefore use `F` and `S` separately.
The usual PSD rank check remains; calibrated rank uses a relative numerical
tolerance, not a new physical information floor. Missing scans withdraw the
channel. A residual model or an explicitly supplied calibrated variance is
required; there is no silent fallback to uncalibrated Hessian weighting.

## Calibration and validation scope

The initial Mississippi experiment fits on 0--60 s of the fixed recording.
Six contiguous 10-second folds select between a constant second moment and
a nonnegative affine sandwich model. The later interval, including frame
847, is excluded from calibration and gain selection. Gain candidates pass
the continuous LMIs before being scored on the fit interval. Position and
heading RMSE are separately normalized by their nominal baseline, combining
the LiDAR-only and GNSS+LiDAR scenarios. A finite candidate search is not a
proof of global optimality.

The map and receiver reference come from the same acquisition. This is a
calibration of consistency with that fixed map/reference, not independently
surveyed absolute accuracy. Sparse pairs on the separate 12-11-24 recording
probe transfer, but their scan-to-scan errors cannot silently substitute for
the production scan-to-map measurement distribution. Production measurements
are also conditional on the matching initialization; recursive rematching
must validate the selected profile.

The initial held-out results predict RMS magnitude reasonably but
under-cover the tails under a Gaussian interpretation. The weights are
second-moment estimates, not certified 90%/95% probabilities. Deployment on
another map, matcher, reference convention, or vehicle requires new calibration
and validation. Both calibration coefficients and the declared continuous
information domain enter the certificate signature.

## ISS scope

The profile declares position and heading strength lower bounds just below
the minimum in the full-support fit data. They are proof-domain assumptions,
not clipping floors. Runtime records violations rather than inflating weak
weights. The same common continuous Lyapunov certificate covers changing
weights within this domain; a new proof matrix is not chosen every frame.
Rank loss, noise outside the calibrated population, sensor outages, and the
sampled nonlinear trajectory are not automatically certified by that result.
The LiDAR implicit jump remains nonexpansive in its fixed injection metric.

Original calibration, protocol, held-out checks, gain search, recursive
experiments, and technical report are stored locally in
`research/calibrated_lidar_gain_20261002/` and
`output/calibrated_lidar_gain_20261002/`.
