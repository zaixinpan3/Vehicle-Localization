# Continuous anisotropic Gaussian matching

A complete Gaussian matching mode is implemented and verified, but remains
**explicitly opt-in** because the Mississippi experiments worsen the maximum
position discrepancy. `distributionRegistrationConfig` keeps the preceding
validated default. `anisotropicRegistrationConfig` selects the new mode for
both scan registration and the shared pose-graph model. No feature detector,
source-window confirmation, map product or motion configuration is changed.

## Continuous geometry

For a source Gaussian transformed by planar pose `(R,t)` and a target Gaussian,
let

\[
 A=R\Sigma_sR^T+\tfrac12\sigma^2 I,\quad
 B=\Sigma_m+\tfrac12\sigma^2 I,\quad
 S=A+B,\quad \delta=R\mu_s+t-\mu_m.
\]

The same pair metric applies to every semantic class:

\[
 q=\delta^T S^{-1}\delta+
 \log\frac{\det S}{4\sqrt{\det A\det B}}.
\]

This is negative twice the logarithm of the cosine-normalized overlap of two
2D Gaussian densities. Its position term contains both covariance axes. If
`S` has long/short eigenvalues `lambda_long >= lambda_short`, the translation
weights are `1/lambda_long` and `1/lambda_short`. Their relative strength is
`lambda_long/lambda_short`: increasing anisotropy continuously relaxes the long
axis relative to the short axis, with no semantic point/line switch or
anisotropy threshold. Finite scatter gives a finite weak force; it is not an
exactly infinite line. The existing observability projection discards directions
that are too weak for a pose update and retains the prediction there.

When either Gaussian is isotropic, its shape has no intrinsic rotation
information. When both are isotropic, the pose-dependent pair cost reduces to
weighted center distance. Different isotropic scales can contribute a constant
shape mismatch to association/robust quality, but no shape rotation force.
Multiple isotropic feature centers can still constrain vehicle yaw through
their relative positions. Sliding translation and rotating an elongated shape
are separate operations.

The determinant term also supplies continuous orientation evidence. Its
rotation-dependent determinant increment is proportional to

\[
 (a_{\rm long}-a_{\rm short})(b_{\rm long}-b_{\rm short})\sin^2\theta.
\]

The eigenvalue gaps vanish continuously as either shape becomes round. No
separate one-degree direction factor is added. Association and residual cost
use the same Gaussian metric, including isotropic noise on both sides.
Same-class mixture priors, quality balancing, temporal stability, map view
support, height compatibility, coarse/fine search and Cauchy loss remain.
Ambiguous target modes use their moment-matched covariance in the same metric.
A common covariance-aware spatial gate permits matching along extended support.

`gaussianRegistrationResiduals` supplies analytic derivatives of transformed
means, rotated covariance, Cholesky whitening and shape overlap. Line search
freezes associations and quality weights, then recomputes covariance at every
trial rotation. The signed square root of the shape penalty has its analytic
finite derivative at identical elongated shapes. Information remains geometric
and uncalibrated; no sensor-noise or independent-posterior claim is made.

## Experiments and deployment decision

All runs recursively propagate the same 1,170 Mississippi scans from the same
initialization and independent wheel/gyro/lateral motion. They use the frozen
source windows previously verified against the original raw perception replay,
the existing view-conditioned map and the same pose references for scoring.
The new runs operate on cached distributions; they do not rerun raw perception.
Reference poses do not enter matching, map-view queries or state propagation.
The maximum excludes the configured first-frame displacement; RMSE includes it.

| Candidate | Maximum after initialization | RMSE | Frame 178 | Frame 932 |
|---|---:|---:|---:|---:|
| Validated default | 15.986 cm, frame 932 | 6.112 cm | 2.776 cm | 15.986 cm |
| Full Gaussian overlap, all-class soft modes | 58.028 cm, frame 894 | 8.973 cm | 46.658 cm | 7.742 cm |
| Eigenvalue-ratio inflation, exponent 2 | 46.045 cm, frame 599 | 9.267 cm | 23.286 cm | 17.849 cm |
| Eigenvalue-ratio inflation, exponent 3 | 86.360 cm, frame 1110 | 15.189 cm | 16.046 cm | 10.215 cm |
| Soft modes only for point classes | 58.028 cm, frame 894 | 8.934 cm | 47.760 cm | 7.770 cm |
| Point-class soft modes, exponent 2 | 46.045 cm, frame 599 | 9.208 cm | 23.286 cm | 18.060 cm |
| Common support neighborhoods, 4 m | 41.423 cm, frame 178 | 8.699 cm | 41.423 cm | 7.860 cm |
| Common support neighborhoods, 2 m | 46.966 cm, frame 178 | 8.757 cm | 46.966 cm | 7.868 cm |
| Orientation-normalized overlap | 48.056 cm, frame 178 | 8.890 cm | 48.056 cm | 18.524 cm |
| Orientation normalization, exponent 2 | 68.977 cm, frame 899 | 11.117 cm | 22.716 cm | 17.206 cm |

The inflation controls preserve the minor eigenvalue and scale the major
variance by the eigenvalue ratio to a power. The support controls use a common
same-class Gaussian-overlap kernel over nearby component centers to estimate
within-plus-between scatter, leaving association coordinates unchanged. The
orientation controls divide Gaussian overlap by the best aligned overlap of
those same eigenvalues, removing the constant scale-mismatch penalty. All these
controls are rejected research prototypes, not production settings. None passes
the preceding maximum-error result. Runtime columns are local observations
from separate runs with concurrent work, not a controlled speed comparison.

Frame 178 and frame 894 are reproduced in isolated registrations seeded from
the preceding default trajectory. The full-overlap errors remain 46.598 cm
and 58.028 cm respectively, with lower objective cost at the fitted biased pose
than at reference. Consequently, recursive drift alone does not explain these
failures. Frame 932's isolated error is 15.655 cm, whereas the changed recursive
route gives 7.742 cm: its route improvement cannot be credited entirely to a
better local factor. Correspondence tables retain centers, covariances, robust
weights, view priors and source support for these controls.

The known partial sign patches and coarse curb cells do not necessarily have
the same center or sampling extent as a complete map feature. A small point
scatter is not proof of a correspondingly precise feature-center observation.
The preceding map conditioning also includes between-acquisition center drift
in its total scatter. These are representation/observation-model issues. The
controls show that simply increasing anisotropic freedom, adding local support,
removing scale penalties or changing soft associations is insufficient here;
they do not isolate a unique remaining cause or prove an accuracy floor.
The continuous rule is mathematically verified, but its production-quality
adoption is unfinished. Existing defaults therefore remain unchanged.

## Validation and reproduction

The regression export combines the existing selected perception, mapping,
registration, temporal-window, height, observability and graph suites with the
new continuous-mode tests. The latter verify all five semantic classes,
continuous axis weighting, round-shape behavior, mixed-feature pose recovery,
unobservable yaw, graph integration, reciprocal/frame invariance and finite-
difference objective gradients including covariance rotation. The new branch
and test files are checked with factory Code Analyzer.

Two final 1,170-frame replays verify bit-for-bit pose equality: the default
matches the September 29 raw-verified output and the final opt-in mode matches
its first full-overlap experiment. Prototype replays check their previously
recorded poses and reasons. Compact CSV/JSON retain the results; large MAT
products stay under ignored `output/`.

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('tests','research/anisotropic_matching_20260930');
checkAnisotropicImplementation;
replayAnisotropicMatching('default_verified',distributionRegistrationConfig());
replayAnisotropicMatching('continuous_verified',anisotropicRegistrationConfig());
verifyFinalReplays;
inspectFrames;
replayMetricScreens; % Temporary prototype path shadow; never add it to startup.
```

The prototype family is intentionally outside normal module paths. It requires
the stored CSVs for parity checks and overwrites only its own experiment exports.
The map and queries share a recording, the fine map comes from detector output,
and the calibration/reference limitations of the previous studies remain.
No surveyed physical groundtruth, independent-drive validation or new precision/
recall improvement is claimed. No viewer window is required by these experiments.

The final validation consists of 333 passing tests in the selected full run,
followed by 14 passing final continuous-mode tests (three additional public
registration/graph cases). The combined export contains **336 distinct passing
tests** in 24 suites. Factory Code Analyzer reports zero findings in six core
files. An initial test invocation lacked the repository root on MATLAB's path;
that harness error was corrected before the complete run. Initial legacy
expectation failures came from temporarily replacing the default geometry;
the final implementation keeps that default and tests the new contract separately.

The first prototype-reproduction attempt lost its path shadow when the replay
called `setupVehicleLocalization`. Its second run incorrectly used the plain
full-overlap mode and failed parity. The replay now explicitly installs and
asserts its prototype path after setup. The failed run had overwritten the first
`power2` pose export; its original scalar metrics remain in the screen log and
are checked separately by the corrected reproduction. Other original pose
exports are checked directly. The failed shadow outputs/log stay local under
`output/anisotropic_matching_20260930/`; they are not counted as successful
controls or deployed algorithm results.
