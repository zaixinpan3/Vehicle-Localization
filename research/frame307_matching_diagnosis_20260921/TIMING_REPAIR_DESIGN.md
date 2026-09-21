# Measurement-time synchronization repair design

Date: 2026-09-21. Status: design and source/data inventory review, not an
implemented clock repair or a newly executed localization experiment.

## Decision

Use native measurement timestamps on a declared receiver-time axis whenever
their clock relationship is established. Recover Ouster packet timestamps
before treating a robust fit of ROS publication times as the final repair.
For sources with only host timestamps, use a shared, validated clock model
and retain explicit uncertainty about transport latency. Apply the resulting
time convention consistently to mapping, sensor replay and evaluation.

The previous frame-307 experiment supports removing isolated publication-time
excursions: seven timestamp-only controls explain most of its 57.5065 cm
apparent error. It does not establish the absolute LiDAR acquisition epoch,
constant inter-sensor latency, or corrected full-sequence localization accuracy.

## Evidence reviewed

The existing extraction manifest at
`output/mississippi_20240607_120931_20260907/sensors/manifest.json` lists these
topics in the Mississippi bag:

| Topic | Messages | Potential use |
| --- | ---: | --- |
| `/vehicle/lidar/front_ouster/lidar_packets` | 74843 | Recover scan IDs, column acquisition timestamps and packet profile |
| `/vehicle/lidar/front_ouster/imu_packets` | 11693 | Check the sensor clock and independently compare motion timing |
| `/vehicle/lidar/front_ouster/points` | 1170 | Associate exported scans with raw packets |
| `/novatel/oem7/inspva` | 5847 | Native receiver timestamps and evaluation trajectory |
| `/novatel/oem7/corrimu` | 11695 | Receiver-timed inertial measurements, subject to unit/interval validation |
| `/novatel/oem7/time` | 117 | Inspect time status and GPS/UTC relationship |

This inventory does not prove that the sensors were hardware-synchronized.
Packet timestamps and TIME payloads were not decoded in this design task.
The exact sensor timestamp mode, firmware, recorded driver build and any
timestamp rewriting during data capture/replay still need to be established.

NovAtel's driver timing FAQ identifies ROS header time as publication time
and recommends embedded GPS timestamps for fusion. This documented behavior
is consistent with the frame-307 header/receiver discrepancy; the particular
recorded driver version is not known from the inspected files.
Ouster documents both measurement timestamps in packet columns and internal,
external pulse, and PTP clock modes. Those modes have different epoch
semantics and cannot be inferred solely from a large timestamp value.

## Clock model and limits

Choose relative receiver seconds as the canonical computational axis, with
GPS week and seconds retained in provenance. Handle week rollover explicitly
and distinguish GPS, UTC, TAI, sensor uptime and host time.

For sources requiring a clock conversion, a centered affine model is the
first candidate:

```text
receiver_time = a * (source_time - source_origin) + b
```

Fit it from timestamp pairs across many messages, using robust residual
weighting/rejection. Estimate local/segmented models only when held-out
timestamp residuals show that a single rate is inadequate. Segment real
restarts or jumps; enforce positive rate, monotonic corrected time and
explicit validity intervals. Do not interpolate exactly through every
publication-time sample, force a 10 Hz frame grid, or assume a unit rate.
The observed approximately 1.091 receiver/ROS rate ratio is large enough to
require checking capture/replay clock behavior, rather than calling it
ordinary oscillator drift without evidence.

A publication timestamp contains both a clock relationship and delivery
latency. A robust affine intercept can absorb typical latency; it does not
identify the physical clock offset separately. Sparse long delays can be
suppressed, but an unknown constant delay remains unobservable from these
timestamp pairs alone. A low residual is therefore not a certificate of
millisecond acquisition-time accuracy. Even a lower-envelope fit assumes
something about minimum delivery latency and must not be declared exact.

INSPVA, BESTPOS and other messages carrying native GPS timestamps should
retain those timestamps directly. Do not re-time them through publication
headers. For LiDAR, first associate valid raw packet columns and scans with
the exported point clouds, checking counts, frame IDs, missing columns,
scan-start convention and timestamp mode. Validate GPS/UTC/TAI conversions
and synchronization status before assigning a common absolute epoch.
If the sensor is free-running and no absolute anchor exists, record the
remaining offset ambiguity instead of concealing it with a fitted delay.

If a residual sensor offset must be estimated from motion, use a separate
calibration interval with sufficient changing motion, consistent axes and
lever-arm treatment, then freeze and validate on held-out intervals.
Do not tune that offset to minimize map-matching error against the same
reference/map used for evaluation. Constant-speed data alone is insufficient
to separate some time, spatial-offset and extrinsic effects.

## Shared implementation boundary

Export one versioned clock artifact and corrected event-time tables; MATLAB
and Python must consume the same artifact rather than independently fitting
slightly different clock models. Include origins, units, epochs, segment
boundaries, scale/offset, fit support, rejected timestamp pairs, uncertainty,
latency assumptions, source hashes and validity limits. Keep arrival time
separate from measurement time.

Replace the repeated timestamp bridges at these inspected entry points:

| Entry point | Required integration |
| --- | --- |
| `scripts/prepareInspvaMappingPoses.py` | Evaluate native INSPVA pose at validated LiDAR measurement time |
| `scripts/replayMississippiLocalization.m` | Use corrected scan times and a common origin |
| `scripts/prepareMncavObserverReplay.m` | Use the same event times throughout sensor/observer export |
| `scripts/prepareWheelMotionInputs.m` | Convert host-only wheel, steering and IMU times through the shared clock artifact; retain source-specific latency uncertainty |
| `scripts/calibrateMncavWheelSpeed.py` | Audit/revalidate timing-dependent calibration on its designated calibration drive |
| `scripts/prepareMncavBestpos.py` | Preserve native BESTPOS GPS time; replace only the old first-LiDAR origin conversion |

Audit additional derived calibration/export consumers before migration.
The corrected observation trajectory used for evaluation must not become a
runtime reference-pose input to localization. Localization remains coarse
perception only; its feature classifier continues to use whole-pillar
statistics. No ring-dependent classification or new reference observer is
part of this design.

Offline clock reconstruction may use both earlier and later timestamp
samples, but must be labeled offline. A claimed real-time implementation
requires hardware synchronization or a separately validated causal clock
estimator using only available samples; a centered future-data fit cannot
be presented as an online result.

## Scan timing and replay

A scan has a time span. Once packet/point timing is verified, choose and
document one scan reference instant. Deskew points to that instant using
appropriately timed motion, if the incoming clouds have not already been
deskewed. Preserve point identities and support unorganized point clouds.
Do not apply compensation twice or infer per-point units from a field name
without checking the recorded packet/driver convention.

Keep the two changes separable: first validate common scan/event times;
then evaluate deskew independently. A clock-only mapping repair can recover
the saved local feature points by inverting their original mapping pose and
projecting with the corrected pose. A deskew change alters point geometry
and requires regenerating affected perception products and distributions.
Rebuild the map, precomputed measurements and replay inputs consistently.
Preserve old experiments as labeled records rather than replacing their
reported metrics with results from an altered evaluation clock.

## Verification and acceptance

Check synthetic known-rate clocks with delayed publications, gaps, week
rollover and restarts. Verify MATLAB/Python conversion equality using shared
fixtures, correct invalid/out-of-range handling, and measurement/arrival
time separation. For the recorded data, validate scan-to-packet identities,
clock residuals across the full drive, physically explained rate changes,
held-out synchronization evidence and unchanged physical source conventions.

At the previously measured 14.5593 m/s, a 5 cm longitudinal timing budget
permits about 3.4 ms; a 2 cm budget permits about 1.4 ms. These are error
budgets, not achieved clock accuracy. Include turning and lever-arm effects
when propagating timing uncertainty. A matching Hessian alone does not
capture timing uncertainty; adding it to measurement covariance requires
a calibrated statistical error model and consistent coordinates.

After timing validation, regenerate consistently timed mapping and replay
artifacts, then report all-frame and accepted-frame RMSE, P95, maximum,
acceptance coverage, along/cross-track and heading errors, with frame 307
as a regression case. Compare clock-only and additional-deskew changes;
use independent map/evaluation data where available. Do not infer overall
improvement from the single-frame diagnostic.

For future data capture, prefer a verified common hardware time source
(PTP or appropriate PPS plus absolute epoch information) and record sensor
configuration, lock status, packet times and host arrival times. The Ouster
documentation confirms these supported modes; availability in this
historical recording remains to be determined.

## Sources and work actually completed

- [NovAtel driver timing FAQ](https://docs.ros.org/en/humble/p/novatel_oem7_driver/doc/FAQ/timing_synchronization.html).
  Official search-indexed content was available on 2026-09-21; direct retrieval
  was access-blocked. This is driver guidance, not proof of the recording's build.
- [Ouster time synchronization](https://docs.ouster.com/sensor-docs/firmware/time-synchronization).
- [Ouster packet column timestamp layout](https://docs.ouster.com/sensor-docs/firmware/3.2/lidar-data).
- [Ouster historical measurement-block timestamp layout](https://docs.ouster.com/sensor-docs/firmware/3.2/lidar-data-legacy).
- [Executed frame-307 diagnosis](README.md), retained without modification.

Completed here: review of these source paths, previously exported topic
inventory and primary vendor documentation; the integration design above.
Not completed here: packet decoding, sensor clock-mode identification,
production implementation, synchronization calibration, map rebuild,
deskew validation or a new localization experiment.
