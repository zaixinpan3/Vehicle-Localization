# Observer framework and Bessafa comparison

Audit date: 2026-09-06. Scope: the current working-tree implementation,
its continuous certificate, and its actual interface to planar D2D registration.
This is an assessment; no estimator implementation or gain is changed.

## Assessment

The implementation is a mathematically motivated, application-specific extension
of the generalized multi-output high-gain framework. Its rotational invariants,
known-input prediction model, explicit heading state, and information-shaped
translation injection are defensible design choices. The stored numerical gain
passes the stated continuous-flow vertex inequalities. These facts do not prove
superiority to Bessafa et al., or establish an ISS theorem for the complete
sampled, delayed, switching implementation.

Two practical findings affect interpretation of existing results:

1. Connecting `localizeLidarFrame` events directly to the observer with default
   configuration gives **zero LiDAR translation correction**, because those
   events omit information and the observer's missing-information translation
   weight is zero.
2. Existing simulation RMSE uses replay-revised history. The separately retained
   `onlineZ` is the causal output and must be used for real-time accuracy claims.

## Sources and inspected version

Bessafa, H., Delattre, C., Belkhatir, Z., Zemouche, A., and Rajamani, R.
(2026). *Generalized multi-output high-gain observer with application to ego
vehicle trajectory and orientation estimation*. Automatica, 188, 112915.
[Publisher record](https://doi.org/10.1016/j.automatica.2026.112915);
[author institution record](https://experts.umn.edu/en/publications/generalized-multi-output-high-gain-observer-with-application-to-e/).
Publication metadata agrees with the supplied 13-page PDF. Its full text was
read locally, especially Sections 3, 4, and 5.3; no paper is redistributed.
This is a targeted comparison, not an exhaustive novelty search.

The code baseline is `8cc97b1d3498a08de24d69b640ce05a6a5fd15f5`.
The user is relocating nine observer files from `localization/improvedObserver/`
to `localization/`; all nine current files were byte-compared with their old
committed paths and match. Those relocation changes, the supplied reference
PDFs, and `improved_observer_derivation.md` remain untouched and uncommitted
by this audit. The latter is an inspected design draft, not verified evidence
that every theorem applies to the present runtime. Source hashes and comparison
results are in `results/observer_framework_audit_20260906/provenance.json`.

## Executable architecture

The first stage is now more elaborate than a standalone two-state bicycle
observer. `runLateralVelocityObserver` integrates 12 internal states: a common
`[vy, lateral-accelerometer bias]` state, a two-state LPV bicycle observer,
six persistent correction states, and the two-state slip-angle interface.
Stationary, crawl, and dynamic information modify continuous correction
channels. The reciprocal-speed model is evaluated only within its speed
certificate. No global pose estimate feeds back into this lateral stage.

The global observer has seven states, in map coordinates:

\[
z=[X,V_X,A_X,Y,V_Y,A_Y,\psi]^T.
\]

The localization pose and D2D registration remain exactly `[X,Y,psi]`.
Neither map height nor retained point-cloud Z statistics adds a vertical,
roll, or pitch pose state. Auxiliary velocities and accelerations support
prediction and observation; seven global states do not mean seven pose DOFs.

The lateral interface provides `vy`, `beta`, and `betaDot`. Define
`q = measuredYawRate + betaDot`. Prediction uses two length-three chains and
one heading integrator:

\[
\dot V_X=A_X,\quad \dot A_X=q^2V_X-2qA_Y,\qquad
\dot V_Y=A_Y,\quad \dot A_Y=q^2V_Y+2qA_X,\quad
\dot\psi=r_m.
\]

The three correction branches in `runImprovedVehicleObserver` are:

- GPS XY plus LiDAR heading: `T*K*Omega*(yb-Cb*z)`.
- LiDAR XY: `T*(P\Cl')*W*(yl-Cl*z)`.
- Four invariant outputs: `T*N/theta^3*(yh-h(z))`.

Here `T=diag(theta,theta^2,theta^3,theta,theta^2,theta^3,theta)`.
The invariant predictions are squared speed, velocity/acceleration dot
product, their planar cross product, and
`Vy*cos(psi+beta)-Vx*sin(psi+beta)`. Measurements use wheel speed,
estimated body lateral velocity, compensated body inertial accelerations,
and zero for the directional consistency constraint.

The runtime integrates with RK4 by default, gives low-rate poses short
correction pulses, and replays accepted delayed events at acquisition times.
`onlineZ` retains what was available at each online sample; `z`, `position`,
and `heading` can subsequently be revised by replay.

## What is inherited and what is extended

| Aspect | Bessafa et al. | Current implementation and interpretation |
|---|---|---|
| Core observer | Triangular coordinates, high-gain scaling, additional nonlinear outputs, LPV/LMI treatment and ISS result; Theorem 5 | Preserves this methodological core; it is not an independently invented observer family. |
| Heading | Six transformed states; heading reconstructed from velocity direction and slip, Eq. (38) | Adds heading as a measured state and gyro-driven integrator, avoiding division by speed in this computation. At rest, heading still requires a reliable absolute observation to correct drift. |
| Vehicle prediction | Eqs. (30)-(35) use restricted motion assumptions and a state-dependent yaw-rate ratio | Uses measured gyro plus slip-rate input. The modeled state dependence is linear and nonsingular, but omitted jerk, track-rate derivative, and cascade errors remain disturbances. |
| Additional outputs | Speed and a dot-product constraint; later adds gyro yaw rate in Eqs. (72)-(73) | Squared speed, measured dot/cross products, and heading-direction consistency avoid inverse-speed ratios. Adding gyro or extra outputs alone is not a new contribution. |
| LiDAR | Not part of that vehicle application | Introduces a map-derived pose and a Lyapunov-shaped translation correction. Actual accuracy depends on map matching and its uncertainty interface. |
| Known-input model certificate | Nonlinear model error is bounded in the high-gain argument | Encloses q and q-squared in a four-vertex box and incorporates their linear error dynamics directly. The box is conservative because those quantities are physically related. |
| Real-time timing | Experiments include sampled KITTI data and GPS dropout | Explicit delayed-event replay is implemented, but its stability is not established by the continuous LMI. |

It would misrepresent the original paper to say it never considers variable
speed or gyro measurements. Section 5.3 explicitly evaluates changing speed
and steering, and introduces a gyro output. The narrower defensible claim is
that the present model makes the departures from the restrictive triangular
vehicle model explicit and avoids its inverse-speed expression.

The current model agrees with the original nominal motion equations on the
constant-speed, constant-track-rate consistency manifold. The complete
seven-state observer is not literally the old six-state observer as a special
case: state dimension, measurements, and off-manifold error dynamics differ.

## Mathematical basis and guarantee boundaries

**Rotational invariants are valid.** Orthogonal planar rotation preserves
squared norm, dot product, and oriented cross product. IMU accelerations
must represent inertial acceleration at the vehicle center of gravity in body
coordinates; raw specific force or body velocity derivatives do not satisfy
the same identities without compensation. At zero speed the expressions
remain finite, but several lose sensitivity and do not restore observability.

**The shaped LiDAR injection has a useful proof.** With error
`epsilon=T^-1*(z-zHat)` and `V=epsilon'*P*epsilon`, its noise-free contribution is

\[
\dot V\big|_l=-2\theta(C_l\epsilon)^T W(t)(C_l\epsilon)\leq0
\quad\text{for }W=W^T\succeq0.
\]

No derivative of W is needed because the Lyapunov matrix is fixed. This supports
arbitrary changing nonnegative translation weights within the stated noise
bounds. It does not ensure observability when other measurements disappear,
nor protect against an unbounded biased/outlier pose. A positive heading
weight floor is an assumption on usable heading evidence, not a source of it.

**The numerical certificate is real but conditional.** The verification covers
8,192 invariant-output corners, two heading-weight endpoints and four model
vertices: 65,536 combinations. It assumes continuously available GPS position
and heading weight at least 0.15. Current default bounds are 8 m/s for each
global velocity component, 5 m/s^2 for each acceleration component, and
0.6 rad/s for q; `theta=3.5`, `sigma=3`. The code fixes a nonzero invariant gain
N before synthesis, rather than jointly optimizing the full gain asserted in
parts of the draft. Feasibility for this gain is numerically demonstrated;
feasibility for every proposed nonzero gain does not follow.

The following gaps prevent a full runtime theorem:

1. **Sampling and dropout:** pose pulses last 0.03 s, whereas nominal GPS and
   LiDAR periods are 0.2 and 0.1 s. Between pulses, the runtime uses zero base
   weights, outside the certified Omega family. In the default experiment,
   only 14.2857% of saved sample times have both active GPS and heading weights
   required by that family. This is a diagnostic of the replayed schedule, not
   a formal continuous-time duty-cycle bound. A sampled/hybrid proof needs
   explicit gap, delay and observability conditions.
2. **Operating region:** the invariant-output derivative bounds involve both
   true and estimated velocity/acceleration. Bounding only the true trajectory
   is insufficient. The runtime reports estimated envelope violations but
   does not establish invariant containment of all estimates or RK stages.
   Heading wrapping also requires a consistent local angular error chart;
   the Euclidean argument is not a global theorem on the circle.
3. **Lateral cascade:** a certificate for the LPV bicycle branch does not prove
   ISS of the 12-state switched/injection/filter implementation. A bounded
   slip-angle error alone also does not bound its derivative. The downstream
   theorem needs explicit bounds on the supplied slip-rate error and model
   disturbances. Finite stop/go outputs are useful evidence but not that proof.
4. **Information fidelity:** the weight helper uses Hxy and Hpsi-psi separately,
   discarding their cross terms. These blocks are not automatically marginal
   information for a coupled `[X,Y,psi]` fit. Raw registration curvature is
   not calibrated sensor covariance. Its scale, nuisance coupling and
   quality/acceptance rules need validation before interpreting the weights
   statistically.

The draft's Eq. (14) also has the opposite LiDAR measurement-noise sign from
the convention `yl=Cl*z+noise` and `error=z-zHat`. Symmetric norm bounds are
unaffected, but the displayed dynamics should be corrected in a future draft
revision. Its old low-speed reset description no longer matches the code.

## Executed checks and interface counterexample

MATLAB R2026a Update 3 was used through the MATLAB MCP. The existing
`improvedObserverTest` and `lateralObserverTest` produced 26 passes, no failed
tests, and two incomplete synthesis tests because YALMIP/SDP solvers were not
on the MATLAB path. This audit did not regenerate gains. The known obsolete
setup path from the ongoing file relocation emitted a warning.

The independent numerical verification returned minimum P eigenvalue
0.1400, worst block-LMI eigenvalue -0.0981237544, and worst Schur-complement
eigenvalue -0.0981247836. These values support the finite matrix inequalities,
not a claim that every runtime assumption holds.

Default 24 s simulation, seed 2026, evaluation on [6,24] s:

| Quantity | Replay-revised history | Causal online output |
|---|---:|---:|
| XY position RMSE | 0.151119 m | 0.169618 m |
| Heading RMSE | 0.00335181 rad | 0.00348460 rad |

The scenario has varying speed/steering, 0.10 s GPS and 0.08 s LiDAR nominal
delays, an extra 0.14 s out-of-order delay, GPS dropout on [8,12] s, and LiDAR
translation degeneracy on [14,18] s. Its truth uses the same nominal vehicle
model/parameters as the observer branch; it is not independent field validation
or a comparison against Bessafa on equal sensor inputs.

For the interface check, remove `sensorData.lidar.information` from this
scenario, retaining its acquisition/arrival times and heading. Run once, then
add `[100,-50]` m to every LiDAR XY pose and run again with identical settings.
The largest absolute translation weight is zero and the largest difference
between the two complete `onlineZ` arrays is exactly zero. This confirms the
default missing-information semantics. `localizeLidarFrame` intentionally omits
information pending calibration, so direct connection currently exhibits those
semantics unless a caller supplies another policy or calibrated information.
The synthetic nominal scenario supplies information and therefore does not
expose this integration gap by itself.

Outputs: `results/observer_framework_audit_20260906/tests.csv` and
`validation.json`. Reproduction entry points:

```matlab
setupVehicleLocalization();
results = runtests({'tests/improvedObserverTest.m','tests/lateralObserverTest.m'});
design = improvedObserverReferenceDesign();
lateral = load('tests/reference/lateralObserverDesign.mat');
sim = simulateImprovedObserverScenario(design,lateral.design,improvedObserverConfig());
settled = sim.truth.time >= 6;
onlineError = sim.estimate.onlineZ(:,[1,4]) - sim.truth.position;
onlinePositionRmse = sqrt(mean(sum(onlineError(settled,:).^2,2)));
lateral.design.cfg.observer.initialState = [0.10;0.02];
sensors = sim.sensorData;
sensors.lidar = rmfield(sensors.lidar,'information');
first = runImprovedVehicleObserver(sensors,lateral.design,design,sim.cfg);
sensors.lidar.pose(:,1:2) = sensors.lidar.pose(:,1:2) + [100,-50];
second = runImprovedVehicleObserver(sensors,lateral.design,design,sim.cfg);
maxOnlineDifference = max(abs(first.onlineZ-second.onlineZ),[],'all');
```

## Research decision

Retain the architecture as a defensible extension under development. Prioritize
the accepted D2D pose/uncertainty contract and causal evaluation before claiming
end-to-end map-aided localization. Then align a sampled/cascade stability result
with the executable schedule and operating region, and compare against a
faithful Bessafa baseline and a suitable conventional estimator using equal
sensors, timestamps, initial conditions and calibration. Include ablations for
invariants, explicit heading, information shaping and replay. Until then, use
the claim "application-specific extension with a verified conditional
continuous-flow certificate and synthetic runtime validation"; empirical
superiority and broader scholarly novelty remain unestablished.
