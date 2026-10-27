# Current localization pipeline

The maintained estimator is the eight-state invariant global observer driven
by the lateral LPV observer, wheel-speed fusion, GNSS position and semantic
LiDAR overlap gradients. State order is
`[X, Vx, Ax, Y, Vy, Ay, psi, r]`.

| Stage | Entry point |
| --- | --- |
| Vehicle and replay parameters | `mncavVehicleConfig`, `mncavReplayConfig` |
| Wheel/IMU/steering preparation for MnCAV | `prepareMncavMotionInputs` |
| Lateral design and execution | `designLateralObserverGains`, `runLateralVelocityObserver` |
| Global configuration and fresh synthesis | `mncavFullObserverConfig`, `synthesizeMncavInvariantDesign` |
| Frame-clock alignment | `fullObserverSupport.synchronizeLocalizationInputs` |
| Global execution | `runFullLocalizationObserver` |
| LiDAR overlap and geometry | `overlapGradientConfig`, `semanticGeometryConfig`, `lidarInjectionSupport.evaluateOverlapGradient` |

The explicit `mncavSmallOffsetObserverConfig` scenario is maintained. It
changes the assumed GNSS geometry and requires its own fresh gains; see
[the vehicle configuration](../config/MNCAV_CONFIGURATION.md).

Current dataset entry points are `runMncavFullObserverExperiment`,
`replayDowntownGnssFreeLocalization`, `runNuscenesOverlapTrial`,
`runKittiOverlapTrial`, and `runMncavVdbLocalization`. They use raw or prepared
coarse semantic clouds and measured-motion source windows. GNSS-only and
dead-reckoning ablations remain available where declared by the entry point.

The common L/G/M certificate retains translation-heading coupling and is
checked against freshly synthesized gains. The continuous ISS result remains
conditional on its stated sensor, motion and local-domain assumptions. Every
L/G/M claim uses the same strictly positive course-speed interval: [10,15] m/s
in the ordinary configuration and [1,30] m/s in the small-offset scenario.
All three families use the same coupled true-course dynamics enclosure.
The packet has five outputs: squared speed, velocity-acceleration dot/cross,
course direction and yaw rate. No cross-minus-rate, body-acceleration projection
or projected-speed replacement remains. Missing velocity withdraws all four
non-gyro outputs; there is no acceleration-only correction. The G-family
Bernstein enclosure retains the shared speed dependence of squared speed,
direction and the new dot/cross acceleration lower bound. Zero-speed execution
has no speed denominator, but is outside this ISS guarantee. See the
[current derivation](../improved_observer_derivation.md#08a-current-five-output-observer-and-positive-speed-iss-domain).
Removing retired implementations does not establish actual-map or sampled
stability certification.

The generic pose/residual input and `validateSyntheticLocalizationCascade`
exercise the current observer with synthetic measurements. They do not run
an online pose optimizer. The removed observer families and registration
experiments are described in [the retirement record](REGISTRATION_ARCHIVE.md).
