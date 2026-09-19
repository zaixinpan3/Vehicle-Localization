# Existing-map-weight NDT matching: implementation and evaluation

Date: 2026-09-19. Control commit: `cd5b91440f7009890ffded069692e7ffb7a00a76`.

## Decision

Use the map's existing `components.mixtureWeight` when evaluating the NDT
candidate. These stored weights already encode temporal feature stability.
Do not reconstruct a second stability score, renormalize map mass separately
within each class, or multiply by `repeatability` again.

The candidate is implemented and tested, but **is not promoted to the default**:
two complete 1170-frame experiments found worse errors and fewer accepted
measurements. The existing geometric matcher remains the active control and
default; the new matcher is an explicitly selectable research method. Neither
perception, the whole-pillar representation, map contents nor observer gains
were changed. This work does not demonstrate an improvement in localization.

## Objective and interfaces

For source pillar distribution `i` and same-class map component `j`, define

\[
K_{ij}(T)=\mathcal N(R\mu_i+t;\nu_j,
  \Sigma_j+R\Sigma_iR^\top+\sigma^2 I),\qquad
J(T)=-\sum_c\sum_{i\in c}\alpha_i\sum_{j\in c}\pi_j K_{ij}(T).
\]

`pi_j` is exactly the stored map `mixtureWeight`. `alpha_i` is source mass
normalized within its semantic class and divided by the number of shared
classes. This bounds source sampling influence without renormalizing target
weights. All same-class Gaussian pairs enter the objective. A point index or
shared grid is unnecessary: the scan grid produces Gaussian statistics, and
the map remains a continuous, weighted Gaussian field. No target rasterization
or subdivision of source pillars is performed.

- `config/weightedNdtRegistrationConfig.m` defines the opt-in configuration.
- `localization/registerWeightedNdtProbabilityCloud.m` minimizes the objective
  over map `[X,Y,yaw]` using bounded SQP. It recenters map coordinates before
  optimization, avoiding loss of numerical precision at UTM magnitudes.
- `localization/semanticNdtSupport.m` evaluates the field and exact gradient,
  including rotation of covariance. Blocks limit temporary pair-array storage.
- `registerSemanticProbabilityCloud` dispatches an explicit `weightedNdt`
  configuration to the candidate. Its default geometric path is preserved.
- `localizeLidarFrame` still calls only `perceiveCoarseProbabilityCloud`.
  Replay and the map-matching experiment now accept `RegistrationConfig` and
  record the selected method. With no option they retain the prior behavior.

The `1000` objective multiplier conditions the optimizer only; exported cost,
gradient and information are unscaled. The background density is used for
inlier/coverage diagnostics, not in this overlap objective. The diagnostic
correspondence table contains the largest-contribution target per source; it
does not replace the all-component objective with hard assignments.

Information is the symmetric finite-difference Hessian of the analytic gradient
of `J`, in physical `[m,m,rad]` coordinates. It retains cross terms. It is
**uncalibrated objective curvature**, not validated inverse pose-error
covariance. Weak directions can be projected out; a candidate that fails the
supported-convergence check is rejected. No new fusion-gain calibration or
GNSS/LiDAR observer experiment was performed. Full XYZ kernels are available
only with an explicit vertical reference; the sequence evaluation uses XY.

The distribution-overlap approach is related to D2D-NDT registration:
[Stoyanov et al., 2012, DOI 10.1177/0278364912460895](https://doi.org/10.1177/0278364912460895).
This implementation adds the existing semantic map weights and the stated
source balancing and acceptance rules; it is not a reproduction claim for
that paper's full algorithm.

## Full-sequence results

Both candidates recomputed coarse perception from all 1170 raw scans in each
of two modes. The frozen map contains 1320 components. The recursive mode
initializes once at reference plus `[0.5 m,-0.4 m,2 deg]`, then uses four-wheel
speed, corrected gyro and lateral-observer velocity for propagation. The
reference-seeded diagnostic resets to that offset every frame. Neither uses
the global GNSS/LiDAR fusion observer.

| Recursive result | Accepted / 1170 | Accepted position RMSE | All-output position RMSE | Accepted yaw RMSE |
| --- | ---: | ---: | ---: | ---: |
| Existing geometric control | 1072 (91.62%) | 0.197132 m | 0.222628 m | 0.589264 deg |
| Log-mixture candidate | 873 (74.62%) | 0.317936 m | 0.381723 m | 0.627819 deg |
| Direct weighted-overlap candidate | 810 (69.23%) | 0.377028 m | 0.388423 m | 0.963380 deg |

The earlier log-mixture trial minimized
`-sum_i alpha_i log(background + sum_j pi_j K_ij)`. It used the same existing
map weights, but differed in influence: for a well-matched isolated component,
the logarithm approximately cancels the multiplicative map weight in the pose
gradient. Its rejected experimental source snapshot remains under ignored local
output and in the archive, not as another production algorithm.

On the **same 597 accepted recursive frames**, position RMSE is 0.200412 m for
the control, 0.275642 m for log-mixture, and 0.281346 m for direct overlap.
This confirms that the regression is not merely a change in which frames enter
the primary metric. All accepted errors are included; no outlier clipping or
post-hoc trajectory alignment is used. Neither trial emits directional events.

In the reference-seeded diagnostic, accepted position RMSE is respectively
0.205773, 0.259138 and 0.281117 m. On 587 commonly accepted diagnostic frames,
the values are 0.214118, 0.246094 and 0.251813 m. Thus removing accumulated
prediction drift does not make the tested objective outperform the control.

`all-output` metrics include propagated predictions following rejection, not
additional LiDAR measurements. The reference is interpolated INSPVA and the
map includes observations from this drive. These are same-drive consistency
results, not independent-ground-truth or held-out-drive accuracy. Timing is
not a controlled performance comparison because diagnostic jobs overlapped.

## Diagnostic findings and remaining limitations

The weighted-overlap recursive run rejects 354 frames for class inconsistency,
compared with 96 for the control. The candidate's per-class local Newton-step
check is conservative and is not equivalent to the control's geometric check.
For example, on frame 214 the raw candidate is 0.009044 m from the reference,
but the curb class proposes a 0.51132 scaled correction, slightly exceeding
the 0.50 threshold. Rejection is therefore not synonymous with inaccurate pose.
The common-frame comparison nevertheless shows that changing rejection alone
would not establish an improvement.

Unlike the control's normal-only curb residual, a finite Gaussian probability
field can constrain motion along a curb through its density and endpoints.
Thus changing from geometric residuals to density overlap changes the optimum,
not just its weight. Sampling-density mismatch, covariance shape, class
influence, optimization basins and rejection calibration remain possible causes;
this experiment does not isolate their causal contributions. The user-provided
frame examples helped motivate the work but were not turned into per-frame
rules or modified labels. No successful accuracy claim is based on a small
subset that improved.

## Validation and reproducibility

- All **74 existing tests** across distribution, geometric, repeatability,
  information and height registration suites pass with the default preserved.
- All **15 weighted-NDT tests** pass. They check XY/XYZ analytic gradients,
  curvature, one-time use of stored weights, unchanged behavior under redundant
  repeatability metadata, weight-dependent alignment, map-component splitting,
  and explicit candidate dispatch while retaining the default.
- Final dispatch reproduces all decisions on 17 cached source-cloud checks
  against the completed weighted-overlap recursive run. Maximum exported-pose
  difference is below `5e-9` and information difference below `9e-12`, consistent
  with CSV coordinate roundoff. These are dispatch checks, not a fresh full run.
- Factory MATLAB Code Analyzer has no findings in the nine modified/new
  MATLAB files; `git diff --check` passes. MATLAB is R2026a Update 3.
- The standalone Python audit verifies coverage and compares identical accepted
  frame populations. Results are in `comparison.json`; source is
  `analyze_results.py`. `artifact_hashes.json` identifies local full outputs.

Reproduce the retained candidate through the final, explicit interface:

```matlab
setupVehicleLocalization;
maxNumCompThreads(8);
report = runMississippiMapMatchingExperiment( ...
    'output/weighted_ndt_repeat', ...
    RegistrationConfig=weightedNdtRegistrationConfig());
```

The full trials were executed before the candidate was given its dedicated
name, while its equations and configuration temporarily occupied the default
entry points. The final separation preserves those equations and configuration;
the 17 dispatch checks verify that separation against the completed run.

```bash
python research/weighted_ndt_20260919/analyze_results.py
```

Local full outputs: `output/stability_ndt_20260919/full/` (log-mixture) and
`output/stability_ndt_20260919/overlap_full/` (weighted overlap), each containing
recursive/control calls, MAT results, summaries and logs. Control outputs are
`output/mississippi_matching_only_20260919/`. Generated MAT files, recorded
point clouds, binaries, abandoned probe implementations and unrelated
`AGENTS.md`/`reference/` changes are excluded from the source commit.
