# Actual LiDAR input versus the zero-delay observer

## Material passport

Date: September 14, 2026. Mode: experiment validation with a bounded,
exploratory fixed-gain comparison. Material: local recorded MnCAV data and
the already executed zero-delay observer, not independent ground truth.
Decision basis: engineering-critical comparison of the actual continuous
input and output. Verification status: VERIFIED for baseline reproduction
and independent harmonic checks; one candidate certificate construction
fails and has no simulated result. This report does not claim that the
position-fusion accuracy problem has been solved.

## Corrected comparison and finding

The zero-delay observer is worse than its actual continuous LiDAR input
in whole-sequence position RMSE. The previous native-frame comparison
instead used 1,170 raw matching/prediction outputs, which include prediction
on rejection. That result remains numerically valid, but does not show that
the observer improves the continuous curve that it actually receives.

For an identical comparison, use the stored `data.lidar.pose` and observer
output at the same original 11,690 timestamps, with identical reference,
frame acceptance, offline interpolation and zero delay. Additional numerical
knots are not counted as extra samples.

| Whole-sequence metric | Actual continuous LiDAR input | Current observer |
|---|---:|---:|
| Position RMSE, mixed ODOM reference (m) | .251085 | .254625 |
| Position RMSE, INSPVA reference (m) | .188465 | .195742 |
| Position peak, INSPVA reference (m) | .967273 | 1.135061 |
| Position P95, INSPVA reference (m) | .376068 | .348828 |
| Position RMSE after t=5 s, INSPVA (m) | .191892 | .198086 |
| Heading RMSE (degrees) | .584031 | .538750 |

The output position discrepancy is larger at 54.64% of these times.
Its INSPVA RMSE increases by 3.86%, while the position peak increases by
17.35%. P95 and heading improve, so the conclusion is metric-specific:
position RMSE and peak behavior have not improved against this baseline.

At t=81.48 s the input discrepancy is .777010 m and the observer discrepancy
is 1.135061 m. This peak is outside the previously diagnosed ODOM/BESTPOS
substitution intervals, and it persists with the same-receiver INSPVA
reference. It cannot be dismissed as that particular reference-source issue.
The largest .882857 s accepted-pose gap begins at 82.69938 s, after this
peak; that single gap also cannot explain the earlier peak by itself.

For context, descriptive INSPVA RMSE by fixed intervals is:

| Receiver interval (s) | LiDAR input (m) | Observer (m) |
|---|---:|---:|
| 0--5 | .079627 | .132880 |
| 5--20 | .238997 | .246607 |
| 20--80 | .147459 | .138329 |
| 80--85 | .502820 | .577861 |
| 85--end | .147590 | .143925 |

These intervals diagnose where behavior differs; no interval is removed
from headline metrics. The exact vector decomposition is
`MSE_output = MSE_input + MSE(output-input) + cross_term`:
.03831493 = .03551894 + .01187949 - .00908350. The terms are correlated
and do not support additive independent root-cause percentages.

## Why zero delay does not remove the amplification

For a frozen straight-motion translational chain, with auxiliary gain N=0
and information weight W=I, let the physical pose-injection gains be
`[k1;k2;k3]`. Eliminating estimated velocity and acceleration gives the
capture-noise to estimated-position transfer

\[
H(s)=\frac{k_1s^2+k_2s+k_3}{s^3+k_1s^2+k_2s+k_3}.
\]

Consequently,

\[
|H(j\omega)|^2-1
=\frac{\omega^4(2k_2-\omega^2)}
{(k_3-k_1\omega^2)^2+(k_2\omega-\omega^3)^2}.
\]

For a stable chain with k2>0, a nonempty low-frequency band has gain above
one even at zero delay. Increasing theta shifts that band and also changes
motion tracking; it does not eliminate this structural property of the
position-driven triple-integrator chain. This is a local result, not an
exact scalar transfer for the full turning trajectory with cross terms.

For the current physical gains [6;12;12], the maximum gain is 1.3546 near
.3587 Hz. Doubling only position correction to [12;12;12] reduces it to
1.1213 near .2136 Hz. Two independent 45 s simulations of the actual runner
use q=0, N=0, information 1e6*I, zero initial state and 1 cm sinusoidal noise
at the respective peak frequencies. Fits on [30,45] s reproduce the gains;
relative errors are 1.23e-11 and 3.74e-9. Thus the amplification is present
in the implemented zero-delay dynamics, not merely an inferred phase lag.

The pose residual also corrects estimated velocity and acceleration.
A changing map-matching discrepancy can therefore induce a motion-state
transient that continues after the LiDAR discrepancy starts decreasing.
The known near-unit information weights and tiny auxiliary correction make
the scalar mechanism relevant, although it does not uniquely attribute every
nonlinear recorded error sample.

## What the current gain-design function actually optimizes

`localization/designImprovedObserverGains.m` reads the LiDAR K and N from
the stored preset. Its optimization variables are P, Q, R, a disturbance
bound and matrix bounds/margin. The objective maximizes a certificate
margin; it does not optimize K/N using LiDAR, velocity or acceleration
measurement errors. The function's existing header states this fixed-gain
scope, but calling the whole operation optimal fusion-gain design would
be inaccurate.

The normalized nonzero N entries are about 7.04e-5. An additional recorded
control on this exact zero-delay input sets N=0: INSPVA RMSE is .19574405 m
versus .19574200 m, and the largest position-output change is .00011455 m
(0.115 mm). Thus the current
configuration gets little corrective benefit from the motion-output
constraints, despite running the lateral observer. Yaw rate still drives
prediction and the lateral observer still aids upstream matching; neither
is claimed to be absent.

This is a gap between a stability-feasible fixed-gain implementation and
a demonstrated accuracy-improving fusion design. A certificate bounds
errors under stated hypotheses; it does not promise lower RMSE than the
measurement being tracked.

## Fixed-input gain and initialization controls

Nine candidates were attempted. Eight completed recorded runs, each with
a verified zero-delay matrix certificate. The ninth failed its particular
certificate construction and has no trajectory. K/N from failed solves
are never used. The baseline reproduces every saved state exactly.

All completed runs use the same zero-delay inputs, actual saved lateral
output and references. Initial state is also fixed except for the explicitly
named first-measurement initialization control. That control uses the first
real LiDAR pose and rotates the initial measured velocity consistently;
it supplies no reference truth.

| Candidate | INSPVA RMSE (m) | INSPVA peak (m) | Maximum estimated acceleration component (m/s²) |
|---|---:|---:|---:|
| Actual continuous LiDAR input | .188465 | .967273 | Not an observer state |
| Current theta=2 | .195742 | 1.135061 | 2.505 |
| Initialize from first measurement | .194521 | 1.135061 | 2.505 |
| Theta=2.5 | .195811 | 1.155748 | 2.905 |
| Theta=3 | .196324 | 1.159442 | 3.391 |
| Theta=4 | .197726 | 1.130955 | 4.294 |
| Theta=6 | .199820 | 1.056723 | 10.148 |
| Theta=8 | .199648 | 1.017954 | 17.084 |
| Double position correction, keep velocity/acceleration coefficients | .189845 | 1.001905 | 2.269 |

The doubled-position candidate changes normalized K(1,1) and K(4,2) from
3 to 6, with theta=2, N and all other gain entries unchanged. Its independent
zero-delay margin is .03673549. It reduces RMSE and peaking relative to the
current observer, but still does not beat the actual input against INSPVA.
Against mixed ODOM, its RMSE .250456 m is slightly below input .251085 m,
so an improvement claim would depend on the reference selected. It remains
an experimental candidate, not a new production default or an established
fusion improvement.

The normalized position coefficient 9 candidate returns an infeasible SDP
status. This is failure of the tested sufficient certificate formulation,
not proof that every zero-delay design with that gain is unstable or
impossible. Some candidates construct a positive-delay certificate first
(.1, .075, .05 or .0375 s) and reverify it at zero delay to avoid the prior
direct-zero numerical issue. Those values are certificate-construction
parameters only: every recorded and harmonic runtime here has delay zero.
The constructive steps use the same four-vertex, norm-remainder inequality
as the production verifier. The study does not replace its proof.

Larger theta produces larger velocity/acceleration excursions. The .4 rad/s
course-rate and stated true-state bounds remain unchanged, and unverified
physical hypotheses are not inferred from a matrix pass. This study provides
no reason to increase theta globally as an accuracy remedy.

## Engineering decision and limits

Keep the direct continuous LiDAR curve as the mandatory position baseline
for future observer changes, alongside raw native-frame metrics. Judge
position RMSE, peak/P95, heading and motion-state behavior separately.
Neither a matrix pass nor an improvement against a less directly comparable
sampling baseline is sufficient.

The next design work should explicitly optimize the effective pose and
motion correction gains and their noise/model-disturbance response, with
corresponding verification of the chosen structure. The current fixed,
near-zero auxiliary gain does not establish meaningful motion-aided
position improvement. No one-parameter gain tested here solves that gap.
The existing map/reference source audit remains relevant to absolute accuracy,
but it cannot excuse the observer's additional error on an identical input
and reference. Production gains, original map and original data are preserved.

The map and query share one drive. INSPVA remains a same-receiver diagnostic
reference. All candidates were evaluated on this same sequence after the
deficit was observed, so these are exploratory controls, not held-out tuning
or independent replication. Time samples are dependent. The statistical
fallacy check covers significance/null claims, magnitude, multiplicity,
post-hoc selection, dependence, sample selection, association/causation,
measurement validity, model validity, uncertainty and generalization. No
population confidence intervals, universal superiority, or additive causal
percentages are claimed.

## Artifacts and reproduction

From the repository root, with the existing YALMIP/SeDuMi paths:

```matlab
addpath(pwd); setupVehicleLocalization;
addpath(genpath('../RobustVehicleLocalization/external/YALMIP'));
addpath(genpath('../RobustVehicleLocalization/external/sedumi'));
maxNumCompThreads(8);
report = compareMncavObserverWithLidarInput;
harmonic = validateMncavInputAmplification;
```

The two new scripts have zero factory Code Analyzer findings. Relevant
validation consists of eight candidate trajectories plus one recorded
zero-auxiliary control, exact baseline reproduction, two independent harmonic
runs, verified matrices for every
executed case, and a visually inspected four-panel figure. The existing
53-test suite was not rerun because the production implementation is unchanged.
A nonfatal vector-export warning occurred; both figure files were produced.

`output/mncav_input_comparison_20260914/` retains full states, candidate
designs/configurations, failures, input/output error curves and PNG/PDF
figures. Compact metrics and the report are committed. Full generated
artifacts are preserved in the archive as identified exports, not added
to the public repository as recorded datasets or generated binaries.
