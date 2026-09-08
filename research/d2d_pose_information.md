# Physical pose information from semantic D2D

Date: 2026-09-07. State order is `[X,Y,psi]`, in meters and radians.

Status update, 2026-09-08: the production observer now uses the full coupled
matrix, and separately validated directional events are supported. See
[the current directional event contract](directional_geometry_and_assimilation_timing.md)
and [the localization interface](../localization/README.md). The original
implementation account below records the earlier full-pose export stage.

The geometric matcher now supplies its local Gaussian information directly to
`localizeLidarFrame`, which emits `timestamp`, `arrivalTime`, `pose`, and
`information`. `registrationSupport.registrationPoseMeasurement` rejects an
accepted result without a finite symmetric positive definite matrix. Rejected or partially observable
matches emit no full-pose event. The observer's missing-information defaults are
unchanged and are not used to compensate for an incomplete D2D interface.

For a final correspondence, let

\[
d_i=R\mu_i+t-\mu_j,\quad
V_i=C_j+RC_iR^T+\sigma_n^2I,\quad
D_i=\begin{bmatrix}1&0&-(R\mu_i)_y\\0&1&(R\mu_i)_x\end{bmatrix}.
\]

For a point-like feature, the whitening matrix is a Cholesky inverse of
\(V_i\). For a line feature, its only nonzero row is
\(n_j^T/\sqrt{n_j^TV_in_j}\). Thus \(J_i=W_iD_i\) differentiates the
whitened residual with respect to **physical additive map-frame pose**.

The exported local information is

\[
\mathcal I=\sum_i w_i\rho_iJ_i^TJ_i,
\qquad \rho_i=\left(1+\|W_id_i\|^2/c^2\right)^{-1}.
\]

Here \(w_i\) is the existing class-balanced semantic quality multiplied by the
map component's geometric repeatability. The robust multiplier is exactly the
one used in the final solver linearization. This is the information of the
local Gaussian surrogate with frozen correspondences, covariance/normal
orientation, and robust weights. It is positive semidefinite by construction.
No diagonal pose prior, eigenvalue floor, or artificial longitudinal road
constraint is added. Map stability is applied once and is not normalized away.
Duplicating identical source components does not multiply evidence because the
existing class normalization is retained. This is a weighted composite model,
not an assumption that all pillar points are independent measurements.

The optimizer uses `q = diag(1,1,yawLeverArm) * deltaPose`. Its normal matrix
therefore has numerical length-scaled yaw units. The export applies
\(\mathcal I=S^{-T}H_qS^{-1}\), where
\(S=\operatorname{diag}(1,1,1/\texttt{yawLeverArm})\). Translation-heading cross
terms are retained. Changing the numerical yaw lever arm cannot change
physical information at the same final associations.

This construction follows the summed-distribution-covariance interpretation
of GICP, Section III-A. The line-normal projection and class/stability weighting
are this project's choices, not claims to reproduce the complete GICP method.
[Segal, Haehnel and Thrun, *Generalized-ICP* (2009)](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf).

`informationCalibrated=false` is intentional: this local model information is
not a demonstrated inverse frequentist covariance of the registered pose.
Association mistakes, shared map observations, calibration errors and
inter-frame correlations are outside this approximation. In particular, map
scatter is not automatically the independent covariance of a sample mean.
No empirical coverage or absolute accuracy claim follows from inverting this
matrix. The existing observer uses information to form bounded directional
correction weights; it is not a Kalman covariance update. Its current separate
translation and heading channels do not use the off-diagonal XY/yaw block.

The explicit legacy `densityOverlap` method remains available for registration
score diagnostics. Its negative similarity Hessian is not converted to Gaussian
information. The online `localizeLidarFrame` interface requires `geometricD2D`
and fails explicitly if the legacy score-only method is selected.

Seven regression cases verify an analytic isotropic four-landmark matrix,
physical yaw-scale invariance, map-frame rotation including XY/yaw cross terms,
repeatability scaling, duplicate-source invariance, an exactly unconstrained
road tangent, and rejection of missing/singular event information.
