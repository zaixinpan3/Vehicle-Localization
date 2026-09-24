# Observer estimation with 20% GNSS loss

Date: 2026-09-23. Code revision `1a85091f6f59d1c2616cdcf1161c2be34482f3f7`.
No algorithm, configuration or map change. The offline map uses the same-drive
INSPVA poses. This is accepted by the project and not revisited here.

## Protocol

Each run starts from the current production closed loop: whole-pillar coarse
sources, the five-scan window, GNSS-aided LiDAR hypothesis selection and the
synchronous seven-state observer. It uses the source cache and inputs of
[the production evaluation](../localization_evaluation_20260923/README.md).
A run withdraws a mask of GNSS frames. On a withdrawn frame the observer
receives no BESTPOS correction and the LiDAR matcher receives **no position aid**.
Every run rematches all scans with its own fused seed, so GNSS loss can change
which LiDAR hypothesis is selected.

Withdrawals are drawn only from the 1,149 frames after the first 2 s. This
keeps the declared initialization transient common to every run. Exactly
230 frames (20.0%) are withdrawn in each lossy pattern:

| Pattern | Definition | Runs |
|---|---|---:|
| `none` | no withdrawal (control) | 1 |
| `iid_frames` | 230 frames drawn uniformly without replacement, seeds 1–10 | 10 |
| `burst_1s` | 23 random non-overlapping 10-frame slots (adjacent slots can merge), seeds 1–10 | 10 |
| `burst_5s` | 5 random non-overlapping 50-frame slots, seeds 1–10 | 10 |
| `block_23s` | one contiguous 230-frame block starting at 5, 25, 45, 65 or 85 s | 5 |
| `all_lost` | GNSS withdrawn for the whole run (LiDAR-only control) | 1 |

All metrics use the 1,149 frames after 2 s. Errors are against INSPVA.

## Results

| Pattern | Mean RMSE (cm) | RMSE range | Mean P95 | Worst max | Mean RMSE on GNSS-lost frames | Heading RMSE (deg) |
|---|---:|---:|---:|---:|---:|---:|
| none (0%) | 7.88 | — | 16.49 | 23.58 | — | 0.386 |
| iid_frames | 9.02 | 8.80–9.31 | 18.88 | 42.06 | 10.48 | 0.386 |
| burst_1s | 11.63 | 9.88–13.03 | 24.24 | 68.33 | 19.77 | 0.401 |
| burst_5s | 13.69 | 10.35–16.09 | 28.69 | 75.90 | 25.17 | 0.395 |
| block_23s | 12.55 | 8.71–16.64 | 28.15 | 75.93 | 22.27 | 0.406 |
| all_lost (100%) | 24.09 | — | 51.89 | 75.93 | 24.09 | 0.479 |

The `none` control reproduces the production after-2-s RMSE of 7.8754 cm.
Per-run values, including LiDAR matching statistics, are in `runs.csv`.

1. **The same 20% loss costs very different amounts depending on its temporal
   structure.** Independent frame dropout adds 1.1 cm RMSE. Gaps of at most
   0.5 s let the observer bridge on motion and LiDAR, and the next GNSS frame
   corrects the result. Bursts of 1 s and 5 s add 3.8 cm and 5.8 cm on average.
2. **The loss is spatially concentrated.** Every run whose post-start peak exceeds 64 cm
   has it at 85.0–85.2 s (frames 851–853) or at 16.7 s (frame 168), and GNSS is
   withdrawn at that frame in every case. Accepted LiDAR matching errors reach
   68.3 cm there, against a 33.3 cm maximum with GNSS available. With position
   aid, these segments select the reference-consistent LiDAR mode. Without it,
   the matcher can stay in a displaced mode, and the fused error grows until
   GNSS returns (see `gnss_loss_traces.png`). The `all_lost` run shows the
   same 75.9 cm peak at 85.2 s.
3. **A 23 s contiguous outage outside those segments is almost free.** A block
   at 25 s or 45 s gives 10.13 and 8.71 cm RMSE, with post-start maxima
   of 25.4 and 26.5 cm. The same block at 5 s or 65 s gives 16.64 and 15.28 cm.
   There, 110 and 84 fused frames exceed 30 cm.
4. **Heading is essentially unaffected.** Mean heading RMSE stays at 0.39–0.41 deg
   in every lossy pattern, against 1.46 deg for GNSS only. LiDAR carries heading;
   GNSS mainly prevents position-mode errors.
5. **LiDAR availability is unchanged.** Accepted matches stay at 1,147–1,148 of
   1,149 in every run. The degradation comes from wrong accepted modes, not
   from fewer measurements. These displaced modes pass the existing
   acceptance checks.

## Interpretation and limits

With 20% GNSS loss the fused position RMSE is 9–17 cm, compared with 7.9 cm
without loss and 24.1 cm without GNSS. It stays below the LiDAR-only error in
every pattern tested. The spread comes from two known ambiguous segments of
this single drive. Their location, not the loss fraction, dominates the result.
Improving robustness to longer GNSS gaps therefore requires LiDAR
mode-ambiguity handling in those segments, for example association-aware
matching or multi-hypothesis tracking. Observer gain changes are not the
lever. The evidence is one 117 s drive, with a same-drive map and
reference-assisted initialization and tilt. The seeds show the variability
over withdrawal placement, not over independent drives. No statistical
generalization is claimed.

## Reproduction

From the repository root, after
[the production evaluation](../localization_evaluation_20260923/README.md)
has produced `output/localization_evaluation_20260923/sources.mat` and
`observer/experiment.mat`:

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/gnss_loss_20260923');
runGnssLossExperiment;   % 37 closed-loop replays; writes runs.csv, summary.csv and the figure
plotGnssLoss;            % regenerate the figure from output/gnss_loss_20260923/traces.mat
```

Random withdrawals use `rng(seed,'twister')` with seeds 1–10 (`iid_frames`)
and `1000*slotLength+seed` (bursts). MATLAB R2026a.
