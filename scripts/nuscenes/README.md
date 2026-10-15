# nuScenes development trial

`runNuscenesOverlapTrial` runs the existing overlap-gradient observer with the
upstream lateral observer on nuScenes mini plus its CAN expansion. It solves
both sets of gains afresh in memory on every call. It does not load saved gains,
perform registration, or use navigation velocity as a wheel-speed substitute.

Download the official [mini split](https://www.nuscenes.org/data/v1.0-mini.tgz)
and [CAN expansion](https://d36yt3mvayqw5m.cloudfront.net/public/v1.0/can_bus.zip).
Extract the metadata, LiDAR and relevant CAN files under one directory:

```text
data/raw/nuscenes/
    v1.0-mini/             # scene/sample/sample_data/ego_pose/calibrated_sensor
    samples/LIDAR_TOP/
    sweeps/LIDAR_TOP/
    can_bus/              # scene-XXXX_ms_imu/zoe_veh_info/steeranglefeedback.json
```

Run from the repository root with MATLAB and the project's YALMIP/SeDuMi
dependencies installed:

```matlab
setupVehicleLocalization;
reports = runNuscenesOverlapTrial("data/raw/nuscenes", "output/nuscenes_trial");
```

The default `RandomSeed=20261014` fixes the stochastic perception fitting.
The entry restores the caller's random-number state when it finishes.

The default evaluation scenes are `scene-0061` and `scene-0103`. The other eight
mini scenes calibrate an approximate steering-wheel to road-wheel ratio using
wheel speeds and gyros only. Calibration and evaluation scenes must be disjoint.
All selection thresholds are fixed in the calibration script; reference pose,
velocity and localization error are excluded from that calibration.

The runtime uses rear-wheel mean RPM with the official tutorial's 0.305 m wheel
radius, corrected steering feedback, and native IMU specific force, angular
rate and tilt. CAN values use previous packets on the native IMU clock; queries
use previous motion/lateral outputs. Excessively old packets are rejected. CAN
receipt timestamps can still contain uncalibrated delay. IMU quaternion yaw is
discarded. Acceleration is leveled and transported from an **assumed** CG IMU
location to the rear-axle output point. Query-cloud tilt uses IMU tilt changes
aligned to one initial reference tilt. Main prediction uses its internal rate
state and upstream sideslip derivative; IMU enters the auxiliary outputs and
the lateral observer.

Every tenth LiDAR sweep builds a fixed map with reference poses; alternating,
disjoint sweeps query it at approximately 10 Hz. One initial reference pose is
perturbed by `[0.5 m, -0.4 m, 2 degrees]`. Later reference poses are used only
for scoring. `pose.json`, its velocity, and the route are never loaded. No
independent raw GNSS channel is supplied. Point clouds are not deskewed and
use existing Downtown perception thresholds, with the ground-height prior
translated to the rear-axle coordinate origin. Intensity and labels are unused.

This is a same-pass, interleaved known-map development experiment, not an
independent mapping traversal or an official nuScenes localization benchmark.
nuScenes `ego_pose` itself is a fused localization product. Its difference from
our trajectory is not an independent survey accuracy measurement. See the
[official schema](https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/schema_nuscenes.md#ego_pose)
and [CAN documentation](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/can_bus/README.md).

`nuscenesTrialConfig` explicitly overrides the MnCAV vehicle for this trial.
Stock geometry and curb mass come from a Renault-authored brochure; the actual
instrumented vehicle variant, payload, IMU lever arm and tire parameters are
unknown. Inertia uses a uniform-body approximation and axle stiffnesses are
declared assumptions. The lateral LMI verifies that assumed model. It does not
identify or certify the real nuScenes vehicle, the actual overlap operator,
its attraction region, or the complete sampled implementation. Main observer
noise scales are declared transfer priors, not fitted nuScenes covariances.
As in the KITTI transfer entry, this trial uses the balanced angular metric
without the nominal MnCAV feedback-authority caps. Its noise objective contains
the LiDAR and weak-LiDAR points; the common sufficient-condition LMIs still
include the moving-GNSS case because velocity/acceleration outputs are present.
No GNSS correction is actually supplied in this trial.

The entry retains summary JSON, metrics CSV and trajectory figures only. It
does not persist point clouds, features, state arrays or gain caches. Raw
downloaded dataset inputs remain inputs and are kept outside Git.

To test the reference boundary with an active perturbation, run:

```bash
python scripts/nuscenes/auditNuscenesReferenceIsolation.py \
    data/raw/nuscenes output/nuscenes_reference_audit
```

This runs two fresh trials, preserving every mapping pose and each first query
pose. All later query reference positions are displaced by `[1000,-700,100]` m
and their orientations are replaced. The eight-state histories, lateral outputs,
intrinsic predictions and overlap gradients must remain exactly equal while
scoring changes substantially. The optional second MATLAB output exposes the
estimates in memory for this check. Source inputs are read-only; temporary
metadata, state arrays and trial figures are removed after the audit. Only the
mutation recipe, small comparison report and run log are retained. This checks
software dependence on the reference fields; it does not establish statistical
independence between the vendor's sensor and reference products.
