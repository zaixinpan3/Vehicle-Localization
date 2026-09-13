# Continuous GNSS and fixed-delay LiDAR runtime validation

Date: 2026-09-13. MATLAB R2026a Update 3; YALMIP and SeDuMi already installed
outside this repository. This is an implementation and numerical-validation
record for [the two continuous ISS theorems](../../improved_observer_derivation.md).
It supersedes the executable pulse/transport paths at Git revision
`f14e9a5cab326798d6c247ce3f887ba4a0dff1ae`; their dated research findings and
original outputs remain preserved.

## Implemented equations and interfaces

`runImprovedVehicleObserver` now integrates one fixed continuous measurement
mode for the seven states `[X,Vx,Ax,Y,Vy,Ay,psi]`:

* GNSS position uses the triangular gains in theorem G: position and three
  invariant innovations drive the first six states; the raw estimated
  velocity/heading coupling drives yaw. There is no independent heading
  measurement. The seven-state ISS conclusion requires sustained motion and
  the theorem's admitted local heading-error sector.
* LiDAR uses `y(t) - C*hatZ(t-d)` with a declared fixed delay and continuous
  uniformly informative pose output. All four extended invariant outputs
  remain active. A supplied continuous estimate history on `[t0-d,t0]`
  initializes the retarded differential equation. The current state cannot
  silently be initialized from an old LiDAR pose.

RK4 and the method of steps approximate these continuous equations. Steps are
no longer than the delay; cubic Hermite history uses accepted states and their
actual derivatives. Cuts respect propagated history derivative breaks. The
buffer retains a delay-length bracket; outputs are immutable and yaw stays
lifted internally. No physical state or model coefficient is clipped.
The bounded extension applies only to the auxiliary maps required by the
proof. GNSS uses the raw fourth coupling in its separate triangular yaw row.

Full physical information is normalized using fixed pose units before
constructing `W=J/(lambda*I+J)`. Cross terms remain; no eigenvalue floor creates
information in a missing direction. Invalid or insufficient LiDAR output is
rejected. Function outputs are required to be continuous and valid throughout
an interval; point evaluations cannot establish that hypothesis between calls.

The event queue, correction pulse, source fusion, old timer certificate,
input-flow transport, replay and associated synthesis/outage diagnostics were
removed from the active implementation. The existing lateral observer runs
once, or its aligned continuous interface is supplied explicitly. Perception
and registration geometry were not changed. Registration sample metadata
remains a data product, not an input to the new runner.

See [the full API and initialization examples](../../localization/README.md).

## Constant-matrix certificates

The verifier recomputes the inequalities from actual `K,N,P` or `K,N,P,Q,R`;
saved success flags cannot authorize changed matrices. GNSS synthesis uses
observable chains and the triangular yaw row. LiDAR synthesis fixes `K,N`
and solves for constant functional matrices with a norm enclosure of the
whole declared uncertainty family. Joint gain/functional synthesis is not
asserted to be convex.

The reference GNSS design has `theta=20`, `|q|<=0.6` rad/s, true velocity and
acceleration component bounds 16 m/s and 5 m/s², nominal margin 20, and
uniform margin **8.207630719**. The runtime uses the exact scaled model bound
`qMax*sqrt(qMax^2/theta^2+4)`, improving on the looser theta-independent bound
used in the preceding theory experiment. Its local yaw proof uses
`kPsi=0.5`, minimum true speed 1 m/s, and heading sector `pi/3`; initial-error
and input admission are still necessary. The reported rate is
0.206748336 s^-1 under those hypotheses.

The reference LiDAR design has `theta=1`, delay 0.15 s, rate 0.05 s^-1,
`|q|<=0.002150319819` rad/s and `0.9985200484 I <= W <= I`.
Its recomputed uniform margin is **0.1185900177**. All four auxiliary columns
are nonzero. Fresh constant-matrix synthesis at the same settings succeeds,
with independently recomputed uniform margin **0.124532**; the resulting
matrices are in [certificates.json](certificates.json).
This is a conservative existence example, not a certificate for ordinary
vehicle turns at 0.6 rad/s. Arbitrary positive information and arbitrary fixed
delay do not guarantee stability for a chosen high gain.

`observer.certificateVerified` reports the matrix check. `observer.certified`
remains false: sensor/true-state bounds, the yaw chart, initial-error admission,
upstream disturbances and a numerical error bound are not established by the
runner. Observed coefficient and motion-proxy checks are reported separately.

## Regression and independent numerical checks

**73/73 pass:** 46 global-observer tests, 15 lateral-observer tests including
fresh synthesis, and 12 registration-information tests. This is a focused
suite, not the full perception/mapping regression. [tests.csv](tests.csv)
contains the executed names and outcomes.

The global tests include:

* Exact moving truth and zero innovation in both modes; nonlinear GNSS yaw
  agrees with `2*atan(tan(e0/2)*exp(-kPsi*v*t))` within 1e-9 rad.
* Delayed scalar yaw agrees with the independent MATLAB `dde23` solution
  within 2e-7 rad over 3 s (`RelTol=1e-10`, `AbsTol=1e-12`). A nonconstant
  initial history has an independently derived polynomial solution on the
  first delay interval.
* Non-grid delay, delay shorter than the integration step, zero delay,
  refinement with an initially incompatible derivative, bounded history,
  immutable prefixes, and lifted yaw across pi.
* Standstill does not invent GNSS heading observability; unavailable output,
  old event metadata, mixed modes, stale certificate flags, wrong clocks,
  insufficient directions and wrong history are rejected.
* Information unit normalization and cross terms, exact one-time delay in
  reconstructed signals, original-data gap rejection, and the real upstream
  lateral observer's interface.

Factory Code Analyzer reports zero findings in all **22 changed MATLAB
files**; see [code_analyzer.csv](code_analyzer.csv). An initial test-only
relative path to the lateral reference fixture was fixed before the passing
suite. `runImprovedObserverDesign` also executes successful fresh LiDAR
synthesis. The plotting entry was exercised with a completed GNSS trial and
an explicitly rejected LiDAR trial; exported PNG/PDF were inspected.

## Analytic continuous-signal experiments

Four 20 s trials use analytic Cartesian circular motion, speed 8 m/s,
course rate 0.001 rad/s, initial heading 0.2 rad, input/output grid 0.01 s,
and maximum integration step 0.005 s. Each coarse run takes 4,000 steps.
The initial state error is `[0.1,0,0,-0.1,0,0,2*pi/180]`; LiDAR prehistory
is analytic truth plus that same error. The known current lateral interface
is supplied exactly in these four experiments.

Noisy outputs are continuous bounded sine functions: position noise
`0.01*[sin(1.3*t);cos(0.9*t)]` m and LiDAR yaw noise
`(0.1*pi/180)*sin(0.7*t)` rad. LiDAR information is `1e6*I`. There is no random
sampling or seed. RMSE below covers **10--20 s** and uses physical units:

| Mode | Measurement noise | Position (m) | Velocity (m/s) | Acceleration (m/s²) | Heading (rad) |
|---|---|---:|---:|---:|---:|
| gnss | zero | 1.35675e-10 | 3.65417e-09 | 2.75603e-08 | 3.58758e-11 |
| gnss | bounded sine | 0.00994532 | 0.0113537 | 0.0130564 | 0.000814783 |
| lidar | zero | 4.63222e-05 | 0.000120871 | 6.45107e-05 | 3.10982e-08 |
| lidar | bounded sine | 0.0133047 | 0.0119369 | 0.00383334 | 0.0010402 |

All four trials are finite and pass the observed coefficient bounds. The
zero-noise final full-state error norms are 2.7797e-8 (GNSS) and 2.6512e-7
(LiDAR). These mixed-unit norms are diagnostic values, not a physical error
unit or a new ISS gain. Halving the noisy LiDAR integration step to 0.0025 s
changes the largest reported state coordinate by at most **3.612e-12** in
that coordinate's units. This is a smooth-scenario convergence check, not a
uniform numerical-error proof.

GNSS high gain still causes startup peaking: the ideal and noisy acceleration
error peaks are 13.042 and 13.693 m/s², respectively, even though the true
acceleration is only 0.008 m/s². The estimated state is not artificially
clipped; the auxiliary extension remains applicable. These results do not
justify a claim that startup peaking has been eliminated.

Two additional noisy 20 s trials run the **actual lateral observer**, loading
its stored reference design and the current `hybrid` configuration. Both
complete; maximum supplied track rate is 0.0014307 rad/s and the observed
coefficient conditions pass. GNSS position RMSE is 0.0099453 m and heading
RMSE is 0.0008032 rad; LiDAR gives 0.013305 m and 0.0010402 rad. The upstream
bicycle model and the analytic circular truth are not asserted identical;
their discrepancy belongs to the disturbance interface. See
[cascade_metrics.csv](cascade_metrics.csv).

## Recorded-data entry validation

The explicit offline adapter was exercised on the stored Mississippi drive
`raw_data_2024-06-07-12-09-31_0`, using the previously saved `sensors/`,
`vehicle_parameters.json` and `recursive_8threads/calls.csv` under
`output/mississippi_20240607_120931_20260907/`. Perception and map construction
were not rerun. Both mode designs were freshly synthesized using the
existing independently calibrated input corrections and declared nominal
vehicle parameters (dynamics factor 1).

Input arrays now use linear reconstruction. All covered original GNSS
positions are retained, without artificial downsampling or delivery clocks.
Pose reconstruction uses physical timestamps and applies the LiDAR delay
exactly once. Reconstructed signals do not establish physical continuous
sensor delivery or online causality.

* GNSS completes **11,686 samples**, position RMSE **0.04137938 m**, yaw RMSE
  **2.88800665 deg**, measured runtime 7.538964 s. The reference and GNSS input
  share NovAtel odometry, so position agreement is consistency with the input,
  not independent localization accuracy. The initialization deliberately uses
  a biased reference pose. Maximum track rate is 0.3549752 rad/s, within the
  GNSS bound, but 246 samples have measured speed below 1 m/s, including zero.
  The motion-proxy condition is false and the seven-state local ISS theorem
  is not claimed over this whole drive.
* LiDAR is **rejected before observer integration** with
  `VehicleLocalization:ReconstructionGap`: the largest accepted-pose gap is
  1.2987 s, exceeding the declared 0.12 s reconstruction limit. No missing
  measurement is silently bridged. Uniform information for that full record
  is not established by this rejection; the adapter encounters the gap first.

The `validatePoseObserverImplementation(...,RerunPerception=false)` wrapper
was also executed against these saved inputs. It freshly synthesizes both
mode designs, completes GNSS and preserves the same LiDAR gap rejection;
[the wrapper summaries](entrypoint_summaries.json) retain both outcomes.

The original traces, trajectory, overview figures and MAT files remain local
under `output/continuous_observer_runtime_20260913/`. Small result exports are
committed here: [GNSS summary](recorded_gnss_summary.json),
[LiDAR rejection](recorded_lidar_summary.json), and
[observed conditions](recorded_conditions.json). The failed LiDAR data contract
does not negate the separate successful continuous-output LiDAR simulations.

## Reproduction

From the repository root, with the external YALMIP/SeDuMi paths already added:

```matlab
setupVehicleLocalization;
results = run([testsuite('tests/improvedObserverTest.m'), ...
               testsuite('tests/lateralObserverTest.m'), ...
               testsuite('tests/registrationInformationTest.m')]);
assertSuccess(results);
report = validateStandaloneObserver;
cfg = improvedObserverConfig("lidar");
newDesign = designImprovedObserverGains(cfg);
assert(verifyImprovedObserverDesign(newDesign,cfg).certified);

stored = load(fullfile(pwd,'tests','reference','lateralObserverDesign.mat'));
current = lateralObserverConfig;
stored.design.cfg.hybrid = current.hybrid;
for mode = ["gnss","lidar"]
    cfg = improvedObserverConfig(mode);
    cascade = simulateImprovedObserverScenario( ...
        improvedObserverReferenceDesign(cfg),stored.design,cfg);
    disp(cascade.metrics);
end

base = "output/mississippi_20240607_120931_20260907";
out = "output/continuous_observer_runtime_20260913";
for mode = ["gnss","lidar"]
    designFile = fullfile(out,mode+"_recorded_design.mat");
    parameters = fullfile(base,'vehicle_parameters.json');
    designMncavReplayObserver(parameters,designFile,1,mode);
    trial = runMncavObserverReplay(fullfile(base,'recursive_8threads'), ...
        designFile,parameters,fullfile(out,"continuous_"+mode),mode, ...
        SensorFolder=fullfile(base,'sensors'));
    disp(trial.summary);
end
plotMississippiExperiment(out);
```

The delay comparison follows MATLAB's documented constant-delay/history
interface: [dde23](https://www.mathworks.com/help/matlab/ref/dde23.html) and
[delay differential equations](https://www.mathworks.com/help/matlab/math/delay-differential-equations.html).
The new observer's equations, certificate enclosures and numerical-history
implementation are project work; the MATLAB reference solver is used as an
independent check, not as evidence of the nonlinear ISS theorem.
