# Improved cascaded vehicle observer

This folder implements the localization architecture in which an independent
2-DOF lateral observer feeds a seven-state global high-gain observer. The
cascade has no feedback from the global observer to the lateral observer.

## State and known inputs

The global state is

```text
z = [X, Vx, Ax, Y, Vy, Ay, phi]'
```

where the velocity and acceleration components are expressed in the map frame.
The lateral observer supplies the body-frame lateral velocity `vy`, side-slip
angle `beta`, and analytic side-slip rate `betaDot`. The gyro supplies `r_m`,
and the known track-angle rate is

```text
q = r_m + betaDot.
```

The prediction model is nonsingular at zero vehicle speed:

```text
z1Dot = z2                  z4Dot = z5
z2Dot = z3                  z5Dot = z6
z3Dot = q^2 z2 - 2 q z6     z6Dot = q^2 z5 + 2 q z3
phiDot = r_m
```

`runLateralVelocityObserver` holds its exported `vy`, `beta`, and `betaDot` at
zero below the minimum scheduling speed, so the cascade remains finite at
standstill while the internal lateral-observer state remains continuous.
The runtime preserves `q = r_m + betaDot` exactly and flags samples outside the
certified track-rate envelope; it does not silently clip the physical input.
It also reports when the estimated velocity or acceleration components leave
the corresponding certificate box.

## Measurement channels

The base output is GPS position plus lidar heading:

```text
Cb z = [X, Y, phi]'.
```

The base correction is `T K Omega(t) (yb - Cb z)`. Heading innovations are
wrapped to the shortest angular arc. The lidar position correction is shaped
directly by scan-matching information:

```text
T P^-1 Cl' W(t) (yl - Cl z),    Cl z = [X, Y]'.
```

`computeLidarInformationWeights` symmetrizes the scan-matching Hessian,
projects negative translation eigenvalues to zero, maps each eigenvalue through
`lambda/(lambda + lambda0)`, and preserves its eigenvector. The heading weight
uses the same rational map and a configured positive floor.

Four invariant outputs couple the two kinematic chains and heading block:

```text
h1 = z2^2 + z5^2
h2 = z2 z3 + z5 z6
h3 = z2 z6 - z5 z3
hc = z5 cos(phi + beta) - z2 sin(phi + beta)
```

Their measurements are

```text
[vx^2 + vy^2,
 vx ax + vy ay,
 vx ay - vy ax,
 0]'.
```

The invariant correction is `T N/theta^3 (ym - h)`, with a fixed, explicitly
nonzero `N`. The high-gain scaling is

```text
T = diag(theta, theta^2, theta^3, theta, theta^2, theta^3, theta).
```

## Robust gain certificate

`designImprovedObserverGains` uses YALMIP and SeDuMi to synthesize `P` and `K`
with a cutting-plane loop. Every candidate is searched over the complete
Cartesian product of:

- 8,192 corners of the structural 13-coefficient mean-value box for the four
  invariant outputs;
- two endpoint matrices for the lidar-heading weight; and
- four vertices that enclose `q` and `q^2` in the known-input model exactly.

`verifyImprovedObserverDesign` then checks the full ten-order block LMI and its
Schur complement at all 65,536 combinations without requiring an optimizer.
`improvedObserverReferenceDesign` stores the numerical matrices as reviewable
MATLAB text and re-runs that exhaustive check whenever it is loaded.

The continuous certificate assumes that GPS position remains in the base
output and that the lidar-heading weight stays above its configured floor. The
runtime also handles GPS dropout by setting the missing GPS channel weight to
zero and retaining information-shaped lidar corrections. The deterministic
dropout test validates that behavior empirically, but it is not a substitute
for the optional mode-dependent/common-Lyapunov GPS-dropout LMI.

Likewise, the LMI is a continuous-flow certificate while the runtime represents
low-rate absolute poses as finite-duration correction pulses. Replay and the
pulse implementation are regression-tested, but no separate hybrid jump/flow
certificate is claimed for the sampled sensor schedule.

## Runtime data interface

`runImprovedVehicleObserver(sensorData, lateralDesign, observerDesign, cfg)`
expects these fields:

```matlab
sensorData.highRate.time
sensorData.highRate.steeringAngle
sensorData.highRate.longitudinalSpeed
sensorData.highRate.longitudinalAcceleration
sensorData.highRate.lateralAcceleration
sensorData.highRate.yawRate

sensorData.gps.timestamp
sensorData.gps.arrivalTime       % optional; defaults to timestamp
sensorData.gps.pose              % N-by-2 [X, Y]

sensorData.lidar.timestamp
sensorData.lidar.arrivalTime     % optional; defaults to timestamp
sensorData.lidar.pose            % N-by-3 [X, Y, heading]
sensorData.lidar.information     % 3-by-3-by-N, N-by-9, or omitted
```

GPS and lidar streams are optional. Delayed and out-of-order events are
accepted only inside the configured replay horizon, inserted at their physical
timestamps, and propagated forward again. Absolute poses act as short
zero-order-held correction pulses rather than stale measurements held until the
next event.

The acceleration inputs must already be planar inertial accelerations at the
vehicle center of gravity, after gravity, attitude, and IMU lever-arm
compensation. The observer deliberately does not infer those calibration terms
from the localization state.

## Typical use

```matlab
setupVehicleLocalization();
lateral = load("tests/reference/lateralObserverDesign.mat");
improved = improvedObserverReferenceDesign();
result = simulateImprovedObserverScenario( ...
    improved, lateral.design, improvedObserverConfig());
```

Run `scripts/runImprovedObserverDesign.m` to regenerate a certified design in
the MATLAB workspace after placing YALMIP and SeDuMi on the MATLAB path.
