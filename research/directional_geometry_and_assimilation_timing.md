# Directional geometry and actual observer assimilation time

Date: 2026-09-08. Inspected baseline:
`82703d4609d7d8e10d600f46cabeb69509aacc17`.

The follow-up source audit correctly identified a frontend/backend gap and
an incomplete delay diagnostic. This change admits carefully validated
partial geometric measurements and records when events reach an online
output. It retains the seven-state cascade, four auxiliary channels,
information fusion, gains, timer certificate, and replay/pulse feedback law.
The earlier [assimilation analysis](observer_proposal_assimilation.md) already
derives all 13 incremental coefficient bounds, the vertex family, weighted
disturbances and conditional timer/replay ISS. Those results remain attached
to the implemented observer.

## A separately accepted directional measurement

The geometric D2D solver already supplied a rank and a supported-subspace
projector. Previously, incomplete rank stopped event export even when that
subspace contained useful lateral/yaw information. The new contract separates
three outcomes:

| Result | Flags | Event |
| --- | --- | --- |
| Valid complete pose | `accepted=true`, `directionalAccepted=false` | `fullPose`, positive definite information |
| Valid supported subspace | `accepted=false`, `directionalAccepted=true` | `directionalPose`, nonzero PSD information, rank one or two |
| Rejected registration | Both acceptance flags false | Empty |

`partialPoseAvailable` remains diagnostic. It cannot authorize export. Both
accepted paths pass the existing correspondence compatibility, sufficient
match count, coverage/similarity, search-boundary and semantic-class
consistency gates. Directional acceptance additionally requires convergence
after the final supported projection: the recomputed supported Newton step
and the unsupported component of the accumulated scaled correction must
both be below ten times the solver step tolerance. This prevents the final
projection/rematching from silently invalidating a convergence flag.

These are local quality gates. They do not guarantee globally correct map
association, calibrate an error covariance, or detect every symmetric scene.
The full-pose acceptance flag retains its meaning for existing consumers.

## Coordinate transformation and the information actually exported

Let physical additive pose correction be \(\delta q=[\delta X,\delta Y,
\delta\psi]^T\), and let the solver use \(\delta q=S q_s\), where

\[
S=\operatorname{diag}(1,1,1/\texttt{yawLeverArm}).
\]

Let \(H_s\succeq0\) be its final robust Gaussian normal matrix and \(\Pi_s\)
the orthogonal projector onto the eigenspace retained by its observability
threshold. The directional event exports

\[
\boxed{\mathcal I_{\rm dir}=S^{-T}\Pi_s H_s\Pi_s S^{-1}},
\qquad \Pi_q=S\Pi_s S^{-1}.
\]

Consequently, \(\Pi_q^2=\Pi_q\), but \(\Pi_q\) need not be symmetric.
It is an oblique physical-coordinate projector, not an ordinary Euclidean
orthogonal projector. Both coordinate conventions are explicitly labelled
in the registration result/event.

The invariant required at the interface is

\[
\mathcal I_{\rm dir}(I-\Pi_q)=0.
\]

Directions excluded by the solver may have small positive raw eigenvalues.
Using the unfiltered normal matrix for such an event would apply corrections
in directions whose pose representative was deliberately left at the prior.
Projecting before the physical congruence removes this leakage. The raw
physical matrix remains in `result.information`; the supported matrix is
`result.directionalInformation` and becomes `event.information`.

Thus the observer's existing spectral map preserves the nullspace of the
**exported** information. With thresholding, this can be larger than the
nullspace of the raw normal matrix. This is a deliberate declaration of
unsupported information, never a positive information floor. Numerical
equalities are checked with relative tolerances. Full-rank events retain
their previous raw physical information and Cholesky validation.

For the existing joint GNSS/LiDAR fusion, multiplication on the left by
\((cI+\mathcal J+\mathcal G)^{-1}\) cannot recover a vector annihilated by
\(\mathcal J\). A supported pose correction therefore remains invariant to
adding a nullspace displacement to its representative, in a consistent
local yaw chart. Wrapping across a different angle branch is outside this
linear representative argument.

The pose/weighted-error contract is still local. In particular,

\[
W_k y_{L,k}=W_k C_pz(t_k)+\epsilon_{L,k}
\]

does not bound the arbitrary unweighted null component. The separate fused
LiDAR and GNSS weights can be nonsymmetric, so the proposal's separate
symmetric-channel dissipation argument is not imported. The stored timer
certificate checks the sector of their actual combined normalized weight.
Valid partial information and a full-state certificate are distinct outcomes.

`localizeLidarFrame` now forwards both accepted event types. Recorded replay
updates only the solver's supported correction and writes the event's
supported information. Its CSV retains `accepted` for full poses and adds
`directionalAccepted`; the observer adapter supports both that schema and
legacy full-pose tables. Benchmark tables also distinguish directional
events from rejected registrations.

## Delivery, incorporation and the settled horizon

An event acquired at \(t_k\) and delivered at \(a_k\) is incorporated when
the outer high-rate loop reaches a sample \(b_k\ge a_k\) and finishes the
associated replay. The diagnostic now records

\[
d_k^{\rm delivery}=a_k-t_k,\qquad
d_k^{\rm wait}=b_k-a_k,\qquad
\boxed{d_k^{\rm assimilation}=b_k-t_k}.
\]

`incorporationTime` is expressed on the sensor/output sample clock. It does
not measure MATLAB wall-clock execution, transport or a real-time deadline.
The delivery placeholder emitted by the frame interface is explicit.
`registrationPoseMeasurement(result,timestamp,arrivalTime)` can receive an
actual delivery timestamp; callers of `localizeLidarFrame` set it on delivery.
The runtime reports whether accepted events supplied this metadata. It does
not silently certify an acquisition-time placeholder as measured latency.

For a configured delivery delay \(\tau\), the configured sample-clock lag is

\[
\bar\tau_{\rm configured}=\tau+\max_i(t_{i+1}^{\rm high}-t_i^{\rm high}).
\]

RK4 may subdivide that interval more finely; those integration substeps do
not shorten the wait for the outer processing loop. The audit uses the
larger of this configured lag and the largest observed accepted-event
assimilation age, considering GNSS as well as LiDAR. It caps the final
settled-history endpoint further at any known qualified acquisition that
has not been incorporated. Events too old for the buffer cannot be treated
as if replay had occurred. The combined-weight interval audit uses this
same endpoint.

This is a finite-record diagnostic, not proof of a future maximum delay or
knowledge of packets absent from the supplied stream. The earlier causal
ISS composition still assumes a valid bound on all relevant events and
sufficient retained history. Real-time execution needs an additional measured
completion budget. Existing delivery-only fields remain separately visible.

## Raw-pose holding remains deterministic forcing

Replay addresses delayed delivery but the current correction compares a
held pose against the current replayed state during its finite pulse. For
perfect tracking and a noiseless acquisition,

\[
r_L(t)=C_pz(t_k)-C_pz(t),
\]

which is nonzero during motion. Under physical bounds, position and lifted
yaw components satisfy

\[
\|b_{L,p}(t)\|\le \bar V(t-t_k),\qquad
|b_{L,\psi}(t)|\le\bar r(t-t_k).
\]

GNSS holding has the analogous position term. These inputs already appear
in the layered ISS disturbance stack. Therefore zero sensor noise alone
does not imply zero error for this implementation. The noiseless fixture
with exact initialization, \(V_X=10\) m/s, zero acceleration/yaw rate and
a pose acquired at zero gives a hypothetical truth innovation of \(-0.2\) m
at 20 ms. With the unchanged gains, the implemented estimate is
0.1696421762 m, a 0.0303578238 m position error. The actual observer innovation
is \(-0.1696421762\) m because the estimate has already departed from truth.
This is a deterministic regression fixture, not a recorded-drive accuracy
result or an asymptotic error measurement.

An output predictor or held acquisition innovation could remove this
particular forcing. Either introduces memory/error variables absent from
the seven-dimensional timer inequalities. A deployment change would need
an explicit augmented flow/reset system, a new verified certificate,
event-order/replay tests, and causal comparisons. We retain the current
feedback and state this limitation rather than reuse an inapplicable proof.
[Ahmed-Ali, Karafyllis and Giri (2019)](https://arxiv.org/abs/1907.06691)
provides primary-source context for intersample predictors and sampling
bounds; its theorem does not certify this repository's implementation.

The proposal's \(\dot\omega Jv\) and signed-speed-jerk terms remain explicit
model residuals. No new derivatives are injected without an input-quality
comparison and enlarged drift certificate. The complete local code also
contains a stationary/crawl/shadow lateral observer and a persistent
sideslip interface, which the supplied audit could not inspect. Those
capabilities are retained. Nonnegative longitudinal speed remains the input
contract; this change makes no reverse-driving guarantee.

## Validation and reproduction

MATLAB R2026a Update 3 passed 150 distinct tests: 51 improved-observer,
12 registration-information, 16 geometric-registration, 12 distribution,
16 height, 17 repeatability, 21 structural-perception and five recorded
motion cases. There were no final failures or incomplete cases. Global
synthesis was excluded because gains, drift/output families and timer
matrices did not change. The stored-reference observer tests include their
existing certificate checks; no new certificate is claimed.

The new/updated cases cover gated directional export, nonconvergence and
inconsistent partial classes, rank-one coupled yaw/translation geometry,
physical projector conversion, suppressed weak eigenvalues, registered curb
information reaching a stationary observer, GNSS loss/nullspace invariance,
delivery metadata, processing wait, pending events and raw-hold forcing.
Earlier full-matrix sector, GPS-expiry and out-of-order tests also pass.

Twenty-one cached raw-frame calls use Mississippi frames 120, 260, 350, 550,
700, 900 and 1050 with three initial offsets. All return full-pose events.
Their local maps contain the subsequent 30 frames and exclude the query.
These calls check compatibility with the existing frontend, not real-world
partial-geometry accuracy. Partial cases use constructed Gaussian geometry.
The recorded sensor/clock adapter was also checked with three legacy full
events and a controlled table containing one full event, one constructed
directional event and one rejection. The directional matrix's nullspace
survives export. This fixture is not presented as a new registration result
from the recorded drive.

All 13 changed MATLAB files have zero factory Code Analyzer findings.
The session's obsolete custom analyzer settings path was unavailable, so
`checkcode(...,'-id','-config=factory')` was used explicitly. The
[validation exports](results/directional_geometry_20260908/) record per-case
results, frame identities and numerical checks.

An initial broad test run exposed the expected rejection-reason change for
insufficient matches; its 12-test class was rerun after updating that
assertion, and other passing results were retained. Two new test assertions
were corrected during development: one compared a floating sample timestamp
with a decimal literal, and one confused the hypothetical truth innovation
with the already corrected observer innovation. The CSV export harness also
required a table-type correction. None required changing gains or loosening
registration gates.

From the repository root:

```matlab
addpath(pwd); setupVehicleLocalization;
suite = testsuite('tests/improvedObserverTest.m');
suite = suite(~contains({suite.Name},'synthesisProduces'));
classes = ["registrationInformationTest","geometricRegistrationTest", ...
    "distributionRegistrationTest","heightProbabilityCloudTest", ...
    "repeatabilityRegistrationTest","structuralSemanticPerceptionTest", ...
    "recordedPlanarMotionTest"];
for name = classes
    suite = [suite, testsuite(fullfile('tests',name+".m"))];
end
results = run(suite);
assertSuccess(results);
assert(~any([results.Incomplete]));
```

Disturbance calibration, registration-domain continuation, numerical-error
bounds and wall-clock deadlines are not established here.
`observer.certified` remains false. The verified conditional matrix
certificate and measured/synthetic interface checks retain separate meanings.
