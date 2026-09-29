# Root-cause investigation of Mississippi matching residuals

This investigation starts from production commit
`04c9f405f2ba3494d216ce690b2327ce54809244`. That implementation has a
post-initialization maximum discrepancy of 0.3006076906 m at frame 178 on the
reused 1,170-frame Mississippi route. The configured first-frame displacement
of 0.6403124240 m is excluded only from the operational maximum; all-frame
RMSE is 0.1260342681 m and P95 is 0.2239794619 m. No independent-drive accuracy
claim is made. INSPVA is the existing comparison reference, not surveyed truth.

## Identified mechanisms

1. **Selected pillars do not provide unbiased feature locations.** At frame
   178 the original fine curb points lie, on average, 12.8 cm inward of the
   selected left curb pillars' complete-point means. A continuous curb
   surface estimator was implemented without adding a finer grid or changing
   selected pillars. It models different road/shoulder slopes and a vertical
   connecting face. A simpler equal-slope height step was inadequate: when
   the true boundary lay outside the selected pillar it could force a false
   inside-pillar optimum, and gutter slopes moved its inferred edge.
   Diagnostic transverse mean absolute errors against original fine-point
   centers change from 12.26/11.74/9.06/10.41 cm to 2.48/2.63/1.89/2.65 cm
   at frames 178/806/854/1096. These are development diagnostics, not held-out
   physical edge annotations. Combining the estimator with a local boundary
   normal-scatter model improves route RMSE/P95; the maximum remains near
   30 cm. This geometry change is adopted for coarse Mississippi only.
   Candidate membership and original whole-pillar diagnostic statistics remain
   unchanged. The new matching centers/covariances are explicitly tagged as
   model geometry, not empirical complete-point moments or calibrated pose
   uncertainty. Batched small linear systems replace per-candidate QR solves;
   fourteen diagnostic frames select exactly the same edges. A subsequent
   full-route parity check found an equal-cost boundary tie at frame 367:
   direct and batched solvers differ by 1.33 cm in one pillar. That check
   stopped rather than reusing the cached result. The final implementation
   is evaluated independently over all raw frames, with any further geometry
   differences recorded and unchanged detection membership asserted.

2. **Map association can manufacture displaced anchors.** At frame 806 the
   current detected pole is about 4 cm from one map mode, but mixing three
   nearby modes moves its effective anchor about 22 cm longitudinally.
   Alternative point/surface geometry, precision-weighted assignments,
   hard/soft class separation, identity priors and point-object map priors
   were implemented and replayed. Their worst-route errors remain worse
   than the baseline. Frozen-reference-seed controls and oracle feature
   centers do not eliminate the bias. A merely sharper optimizer cannot
   resolve an inconsistent map reliably.

3. **The apparent map aliases include acquisition-motion distortion.**
   Original fine observations of the same narrow pole near frame 806 drift
   approximately 0.7 m across mapping frames, although each scan has compact
   XY support. The exact current and mapping poses/calibration agree.
   Ouster PointCloud2 point-relative times were recovered from the original
   bag and checked against all 1,170 stored scans, including their documented
   axis rotation and organized point order. Maximum XYZ reconstruction
   disagreement is 15.3 micrometers; header stamps agree exactly. Each scan
   spans about 100 ms. No absolute GPS/hardware phase is inferred from that
   audit alone.

   Reference-twist deskew used only as a diagnostic reduces this pole's
   longitudinal cross-frame standard deviation from 29.3 cm to 13.6–15.4 cm
   over the explicitly tested 0/50/100 ms reference phases. A separate
   fine-pole-track consistency fit uses independent wheel/gyro increments
   and only frames 1–585 to estimate an effective phase of 94.64 ms.
   Applying it unchanged to frames 586–1170 reduces within-track XY RMS
   scatter from 23.16 cm to 18.35 cm (20.8%). This is a same-recording held-out
   consistency check; it does not establish surveyed extrinsics, an absolute
   sensor latency, or independent-drive localization accuracy.

4. **Temporal pooling and coarse-basin protection can retain bias.** With
   the improved curb geometry, frame 829 has a 29.86 cm coarse solution and
   a 3.54 cm fine solution. The fixed 15 cm refinement guard retains the
   coarse solution. Disabling that guard lowers some errors but moves the
   route maximum to frame 1096 at 29.79 cm, only 2.66 mm below baseline.
   An information-weighted trust metric also helps this frame, but regresses
   the original geometry to a 49.90 cm maximum at frame 837. No reference pose
   or frame-specific override is used for selection. Both broad replacements
   remain research controls.

   At frame 1096, pooled-source geometry gives 29.79 cm while current-scan
   geometry gives 17.00 cm; hard association gives 14.64 cm. Applying either
   globally regresses other route frames. Current-center variants retain
   two-frame confirmation and pooled scatter, but peak at 33.84–37.54 cm.
   This is evidence of representation/association inconsistency rather than
   a failure to iterate the optimizer longer.

5. **An isolated conflicting class can pull the solution.** With the 50 ms
   deskew map, frame 154 peaks at 42.59 cm. A historical sign distribution is
   about 1.7 m from its map target. Removing signs diagnostically reduces this
   frame to 4.42 cm. A bounded redescending loss and globally normalized class
   disagreement gate were implemented with finite-difference tests. Applied
   only to signs with boundary geometry, the maximum is 29.79 cm, an immaterial
   peak improvement. These branches are preserved as a reproduction patch,
   not installed as default solver policy.

## Experiment boundaries

`rootCauseReplay` performs a causal recursive 1,170-frame replay with the
unchanged independent wheel/gyro trajectory, initial displacement and
acceptance policy. Reference XY/yaw are used only after each estimate for
scoring. Existing known INS tilt remains an input. Original fine labels are
never online detector inputs. All original raw source variants use the shared
0.6 m pillar lattice. `oracle*` experiments intentionally use unavailable
reference geometry and are diagnostic only. `jointMatching*` outputs fuse
wheel/gyro constraints with map factors and are explicitly not independent
LiDAR measurements; neither graph prototype improves the baseline.

The graph investigation exposed a dimension bug: adding curb-direction
residuals changes each correspondence from two to three rows, but the pose
graph repeated its weights only twice. The weight-row replication now follows
the actual residual dimension and a regression test exercises the direction
factor. This fix is independent of the rejected graph accuracy experiments.

`runDeskewExperiment` compensates raw scan geometry before perception using
independent causal wheel/gyro increments and an explicit reference phase.
It also rebuilds the complete map using the original offline fine annotations
on the compensated geometry. All 399,086 map point identities are recovered
by inverse SE(3) and raw-coordinate lookup, with maximum reconstruction error
1.39 nanometers. The separate original fine-reference index files differ by
a few points from the final mapping annotation files; they are not silently
substituted for the map's original membership. Frame timestamps, mapping
poses, calibration and online motion inputs remain fixed. Phase sensitivity
and full-route replay are evaluated together with this coherent map rebuild.

## Reproduction and status

All MATLAB research entry points require `setupVehicleLocalization` and this
folder on the MATLAB path. Local large inputs/intermediates are retained in
`output/root_cause_matching_20260929/`; raw recordings and MAT files are not
published. Source and result CSVs in this folder describe actual executed
variants. A summary CSV is written only after a complete route replay.

The rejected association prototypes are preserved in
`rejected_association_prototypes.patch` against the baseline. Their optional
configuration branches were removed from production after evaluation. Apply
that patch only in an isolated reproduction checkout for the precision,
point-surface and class-specific soft-assignment experiments. Do not infer
that changing those unused options affects the final production solver.

```bash
uv run --offline --with h5py --with scipy --with numpy --with rosbags python research/root_cause_matching_20260929/extract_point_timing.py
uv run --offline --with numpy --with pandas python research/root_cause_matching_20260929/fit_timing_origin.py
```

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/root_cause_matching_20260929');
recoverMapPointIndices;
exportCalibrationTracks;
diagnosePoleDeskew;
runDeskewExperiment(0);
runDeskewExperiment(.05);
runDeskewExperiment(.1);
```

## Final production result

All 1,170 frames were recomputed from raw scans using both configurations.
Candidate structs and original ground/off-ground diagnostics agree exactly.
The final batched implementation was replayed independently after the
near-degenerate frame-367 geometry difference; it was not credited with a
cached QR result.

| Metric | Previous production | Final production |
|---|---:|---:|
| Maximum after configured initialization | 30.06 cm (178) | 29.86 cm (829) |
| All-frame RMSE | 12.60 cm | 12.13 cm |
| All-frame P95 | 22.40 cm | 20.44 cm |
| Full / directional updates | 1153 / 10 | 1149 / 10 |
| Perception median in paired raw replay | 126.97 ms | 169.79 ms |

The median paired perception difference is +40.92 ms. The maximum improves
by only 2.00 mm; P95 improves by 8.72%. All 235 selected tests pass, including
the actual-data regressions; 11 changed MATLAB files have zero factory Code
Analyzer findings. Detection membership is unchanged for every class/frame,
so reference-empty rates and reference-point coverage remain unchanged for
any fixed reference labels. There is no claim of improved detection recall.

The final frame-829 viewer and `maximum_pole_829.csv` expose a remaining
reference-agreement issue. Original fine pole points occupy pillars 5433/5533;
none of their 80 points is covered. Their supported shaft is 2.73 m tall with
2.69 m continuous span and 7.47 cm radial RMS, but context scores 0.528/0.690
fall below 0.875. Tilt 5.14 degrees and isolation 0.908 also fail the current
split-shaft recovery bounds (5 degrees / 0.95). The selected pillar 6846 has
score 0.946, 3.09 m continuous support and isolation 0.984, but contains zero
original fine-reference points. This is reference disagreement, not proof that
the physical shaft is false. The original fine detector is imperfect.

Removing the matched pole class diagnostically gives 1.00 cm at frame 829;
allowing fine refinement gives 3.54 cm. Replacing whole-pillar pole centers
with the fitted axes together with the final curb geometry instead peaks at
30.56 cm on the full route and is rejected. These controls distinguish
missing reference poles from the biased association of an existing anchor.
The reference labels and frame identifiers are never used by online policy.
The final viewer is opened in niri with all 65,536 original points, point size
4, no feature overlay points and a nearby shared 0.6 m pillar grid.

## Adoption and limitations

The production change adds `estimateCurbBoundaryGeometry` and
`applyCurbBoundaryScatter`, enabled through `curbBoundaryGeometryConfig` for
coarse Mississippi. Offline fine mapping and the Downtown profile retain their
existing geometry. A fitted boundary must lie inside the originally selected
0.6 m pillar; no new semantic point labels, fine grid or candidates are added.
Normal scatter is estimated from neighboring selected boundary centers with
a 0.01 m^2 floor, while tangential scatter and XY/Z cross terms are propagated
by a congruence transform. Whole-pillar diagnostics remain available.

The final fresh-raw replay and validation are recorded in `finalSurface.csv`,
`finalSurface_summary.csv`, `final_raw_verification.csv`, `final_tests.csv` and
`code_analysis.csv`. `summarizeStudy.py` aggregates completed replay labels,
checks the fresh/cached result, and exports `final_summary.json` and the route
comparison figure. The first configured frame is excluded only from the
operational maximum. Runtime values are local measurements, not real-time
certification.

Coherent deskew/map rebuilding at phases 0/50/100 ms peaks at
39.43/42.59/37.06 cm. Adding the new curb surface model at 100 ms gives
36.27 cm. These results reject deskew as an immediate drop-in peak-error fix,
although the independent pole-consistency diagnostic establishes that motion
compensation addresses a real source of map spread. Absolute timestamp phase,
extrinsics and independent-drive accuracy remain unresolved.

No tested algorithm makes a substantial, stable reduction of the remaining
30 cm peak. This is a measured development plateau, not a physical accuracy
limit. The report separates deployed bulk-error improvement from rejected
peak-error experiments; it does not attribute all residuals to parameters.

`rejected_robust_trust_prototypes.patch` preserves bounded-loss/global-gate and
information-trust experiments against the starting production commit. Apply
it in an isolated baseline checkout to run those variants and the tests under
`prototype_tests/`. The replay harness fails explicitly when such options are
requested without the prototype. `rootCausePerceptionConfig` freezes empirical
geometry for historical source experiments; the final runner uses current
production defaults. Large timing sidecars, map builds and scan data remain
in the local output/data folders.

The first expanded test invocation lacked the data-root environment variable;
two data regressions were filtered rather than passed. The final invocation
explicitly supplies the local data root. No skipped test is counted as passed.
