# Historical 7.85 cm replay without INSPVA matching seeds

## Result

The historical 7.8474 cm fused-position result is reproducible. Removing every
recorded INSPVA XY/yaw seed, including startup, increases full-sequence fusion
RMSE to **9.4292 cm with fused-state prediction** or **10.9238 cm with matching-state
prediction**. The unchanged GNSS-only observer gives **8.8041 cm** with either
new startup state. Thus the old 7.85 cm result benefited from reference seeds;
its advantage over GNSS does not persist in these reference-free seed replays.

All fused statistics use the same 1,169 synchronized outputs, raw frames
1--1169, without trajectory alignment, error clipping, or dropping failed-match
frames. Raw frame 1170 lies beyond the original motion-input coverage and was
not part of the 7.85 cm population. Errors are planar position discrepancies
against recorded INSPVA, not independently established absolute accuracy.

| Matching initialization / observer | Fused RMSE (cm) | Median (cm) | P95 (cm) | Maximum (cm) | Accepted LiDAR frames |
|---|---:|---:|---:|---:|---:|
| Historical per-frame INSPVA seed | 7.8474 | 4.6151 | 17.1913 | 26.8705 | 1083 |
| Previous matching result + wheel/IMU prediction | 10.9238 | 6.3847 | 22.4736 | 38.9337 | 1081 |
| Previous fused result + wheel/IMU prediction | **9.4292** | 5.8676 | 19.5558 | 30.4931 | 1087 |
| GNSS-only observer, same new startup | 8.8041 | 5.5902 | 19.1085 | 26.0834 | Not used |

No fused output exceeds 0.5 m in any arm. Without GNSS feedback into matching,
the separate matching/prediction trajectory does reach **1.5191 m** during a
rejected-match interval (frame 837); this is a propagated prediction, not an
accepted LiDAR measurement. There are 14 such prediction-inclusive outputs
above 1 m. Omitting those frames would hide an important tracking limitation.

Raw matching has a different population from fusion:

| Initialization | Accepted-pose RMSE (cm) | Maximum (cm) | Samples |
|---|---:|---:|---:|
| Historical INSPVA seeds | 14.8629 | 57.5065 | 1083 |
| Matching-state prediction | 19.3682 | 81.0034 | 1081 |
| Fused-state prediction | 16.9174 | 57.5059 | 1087 |

On common accepted frames, the conclusions persist: historical versus
matching-state prediction is 14.7676 versus 19.2218 cm (1068 frames), and
historical versus fused-state prediction is 14.7735 versus 16.8750 cm
(1078 frames). Every population is explicit in `metrics.csv`.

## Frozen historical implementation

The exact runtime is Git revision
`d6502080b392e6d368696eeb0b7fd755f556752b` (the 7.85 cm output-point/gain change),
extracted unmodified into ignored experimental output. No historical solver
is installed as an alternate active production path. The experiment preserves:

- Historical single-frame geometric distribution-to-distribution matching,
  its map stability weights, gates, information matrices and configuration.
- The saved local Gaussian clouds built from the then-confirmed fine perception,
  and the same 1,320-component probability map. This specifically tests the
  historical fine-input version; it does not change online coarse perception.
- Wheel-derived forward speed, estimated lateral speed, IMU yaw rate and
  accelerations, the seven-state observer and its gains, the independently
  calibrated BESTPOS output-point offset, and all recorded source validity.
- Historical offline real-sample synchronization (up to 0.09996725 s of GNSS
  bracketing wait) and zero modeled LiDAR processing delay.

The old observer reproduces its saved trajectory exactly. Re-running matching
with the old reference seeds reproduces all 1083 acceptance decisions; the
maximum pose-entry difference is 4.66e-9 and information-entry difference is
4.92e-11. Timestamp association uses a 1 ns absolute tolerance because saved
CSV/MAT representations differ by up to 4.98e-13 s; frame order is asserted.

## Initialization and recursion

The recording begins stationary. Startup uses the first valid BESTPOS XY
packet and a fixed full-circle heading search, **-180:10:170 degrees**. For
each heading hypothesis, apply the unchanged output-point correction and run
the historical matcher. Select the accepted hypothesis with highest matcher
similarity. No reference position, heading, reference error, or future course
enters candidate generation or selection.

Candidate 7 (initial heading -120 degrees) wins. Only after selection, scoring
shows its converged pose has 3.3716 cm position discrepancy and -0.0872 degrees
heading discrepancy. The whole candidate table, including failures, is saved.
Velocity and acceleration initialization uses this estimated heading and the
original motion inputs. All old LiDAR measurements and the old initial state
are cleared before each reference-free replay.

After startup, trapezoidal forward/lateral velocity and yaw-rate integration
advances the previous estimated pose. A failed full-pose match retains the
prediction but supplies **no** LiDAR event to fusion, matching the historical
full-pose acceptance policy. Directional candidates remain diagnostic only.

`matching_prediction` advances the previous accepted match or fallback
prediction. `fusion_prediction` advances the previous fused observer pose.
The latter runs prefixes of the unchanged historical observer to expose its
previous state without reimplementing the observer. Final full replay agrees
with every saved prefix pose within 1e-9. Prefix replay is an experimental
correctness mechanism; its runtime is not a deployment timing benchmark.

## Validation and limits

The independent Python audit reconstructs every seed from GNSS, the fixed
heading grid, previous estimates and sensor motion, with maximum numeric
difference 9.32e-9 at absolute UTM coordinates. It also checks startup
output-point correction, exact frame populations, accepted-measurement versus
fusion-mode consistency, positive-definite accepted information, rejection
fallbacks and the historical RMSE. All **56** related historical MATLAB tests
pass (GNSS output point, geometric registration, registration information and
repeatability weighting); all four harness MATLAB files have zero factory
Code Analyzer findings. No random perturbations or result-driven gain sweep
is used.

This removes INSPVA **XY/yaw from matching and observer initialization**, not
all reference assistance in the experiment. The same-drive map was built
using reference poses; frozen local fine-feature caches retain their existing
reference-assisted preparation and gravity alignment/recorded tilt. BESTPOS
and INSPVA share receiver/INS processing. Upstream sensor preparation remains
offline. No independent mapping drive, raw causal tilt estimator, GNSS-denied
global startup, or multiple-drive generalization is claimed.

The experiment establishes seed sensitivity under the frozen old system.
It does not isolate all causes of current-versus-historical performance:
source clouds, matching gates and fusion implementation changed subsequently.
Production parameters are unchanged by this research replay.

## Reproduction and artifacts

With the original ignored inputs present, prepare a fresh historical runtime:

```bash
python research/reference_free_785_20260920/prepare_runtime.py
```

The preparer deliberately refuses to overwrite an existing snapshot. Run the
following from `output/reference_free_785_20260920/historical_runtime`, replacing
`REPO` with the repository's absolute path:

```matlab
addpath('REPO/research/reference_free_785_20260920');
runReferenceFree785('REPO');
```

From the project MATLAB session, run `validateReferenceFree785('REPO')` via
MATLAB MCP, then run the independent audit from the repository root:

```bash
uv run --offline --with numpy --with pandas --with matplotlib --with h5py \
  python research/reference_free_785_20260920/analyze.py
```

Original replay MAT files, per-frame CSVs, `comparison.png` and `comparison.pdf`
are under `output/reference_free_785_20260920`. The compact result tables,
bootstrap candidates, verification records and SHA-256 artifact manifest are
retained here. Large recorded/generated data and the historical runtime
snapshot remain uncommitted. The active production runtime is unchanged.
