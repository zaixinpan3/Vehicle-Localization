# Frame 895 after the continuous-support repair

At implementation `e5531a9ee86aa22f1e31510a755898a386a9da88`, the recursive
Mississippi replay maximum after initialization is frame 895:
**15.871750 cm position discrepancy and +0.693379 degrees yaw discrepancy**.
In reference vehicle axes the position discrepancy is -5.92047 cm longitudinal
and -14.72618 cm lateral. This is relative to the recorded evaluation trajectory,
not a surveyed absolute localization error.

The prediction already has 15.45213 cm error: +2.78877 cm longitudinal,
-15.19839 cm lateral and +0.61832 degrees yaw. Matching moves the vehicle
origin -8.70924 cm longitudinal and +0.47221 cm lateral. It does not remove
the lateral/yaw bias. However, the bias cannot be attributed only to the seed:
starting exactly at the reference converges to essentially the same solution.

## Controlled results

The current raw coarse clouds, saved five-scan source, fixed map and production
solver are reused. The source window reconstructs exactly. Production output
reproduces within 1e-7 in pose coordinates. Map view conditioning is frozen at
the causal prediction except the explicitly dynamic reference-seed control.
Reference-motion and reference-seed controls are offline oracles, never runtime
inputs. The interventions are not an additive error budget.

| Intervention | Position discrepancy | Yaw discrepancy |
| --- | ---: | ---: |
| Reproduced production | 15.8718 cm | +0.69338 deg |
| Reference seed, frozen map | 15.8715 cm | +0.69340 deg |
| Reference seed, recondition map | 15.8759 cm | +0.69323 deg |
| Increase iteration budget to 400 | 15.8718 cm | +0.69338 deg |
| Reference translation for historical transport only | 9.9886 cm | +0.49461 deg |
| Reference yaw for historical transport only | 15.5482 cm | +0.68643 deg |
| Reference full historical motion | 9.6852 cm | +0.48767 deg |
| Suppress sources assigned to map curb 685 | 9.1886 cm | +0.45568 deg |
| Negligible angular factor | 23.4970 cm | +1.08527 deg |
| Current scan alone | 17.6743 cm | +0.86415 deg |
| Remove pole | 16.3360 cm, directional rank 2 | +0.67020 deg |

There are **one current pole and ten current curb distributions**. The confirmed
matching window has **one pole and fourteen curbs**, with no signs. Thus this
maximum is not explained by a missing pole. Removing the available pole loses
a pose direction. The pole's center at the reference pose differs from its
map target by only [2.6151, -2.4708] cm.

## Supported sources of the discrepancy

**Historical translation compensation contributes.** The window spans 0.40089 s.
The oldest scan's motion-based displacement is [-4.2489, +0.07545] m in current
vehicle axes, compared with reference displacement [-4.2311, +0.00809] m.
The lateral disagreement is about 6.74 cm. Correcting only this historical
translation reduces the final error from 15.87 to 9.99 cm; correcting only yaw
has little effect. This establishes a motion/reference translation inconsistency,
without determining whether wheel/lateral calibration, the motion estimate or
the recorded reference is physically correct.

**A group of curb constraints contributes a consistent angular bias.** At
reference, six source clouds assigned to target 685 disagree in direction by
approximately -0.66 to -1.18 degrees. They use a strong angular scale of about
50 (roughly a 1.13 degree residual scale). That group's scaled yaw gradient is
-0.49876, including -0.73397 from the angular factor. The position gradient
part partly opposes the angular factor. Suppressing the seven sources assigned
to 685 at the production solution improves error to 9.19 cm. Other retained
class weights are preserved, but neighborhood geometry and correspondences can
change, so this is a group intervention rather than an isolated fixed-factor
ablation. Removing all angular information worsens the result. No current fine
point inspection was performed here, so this does not establish false curb
detections or an incorrect physical map.

**A well-aligned distant pole does not eliminate yaw/origin coupling.** The pole
is approximately 12.31 m ahead of the vehicle. The +0.69338 degree yaw error
alone shifts its transformed location about +14.93 cm laterally. The vehicle's
-14.73 cm lateral origin error almost cancels that displacement. A visually
well-aligned pole can therefore coexist with a 15 cm vehicle-origin error when
curb direction evidence biases yaw. The pole itself cannot independently fix
all three planar pose coordinates.

The current objective is 0.10176 at the biased solution versus 0.23627 at
reference. All three scaled directions are observable (production eigenvalues
approximately 3.36, 7.19 and 38.55); a larger iteration budget changes nothing.
This supports biased/inconsistent input geometry rather than a failure to
iterate or simply an absent feature. It is consistent with the earlier frame
932 transport/geometry diagnosis, but the measurements above are fresh frame
895 controls.

The user subsequently requested a full observer replay. That separate experiment
is recorded in `research/support_full_localization_20260930/`. This diagnostic
does not itself modify perception, motion, map matching or map data.

## Reproduction

```matlab
setupVehicleLocalization;
addpath('research/frame895_diagnosis_20260930');
diagnoseFrame895;
```

Tables retain all 21 controls, reference geometry, per-target forces, objectives
and relative motion. Large diagnostic state remains in
`output/frame895_diagnosis_20260930/diagnostic.mat`. No GUI was opened.
