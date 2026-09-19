# Why additional LiDAR measurements increased position error

Date: 2026-09-18. Input experiment: `output/mncav_coarse_localization_20260918/observer/experiment.mat`, produced with project commit `c925ffe0c17da3001041df0c3749982bfde93dce`.

## Finding

The current fusion gives the less accurate coarse LiDAR position measurements
more correction gain than the aligned receiver positions. This is a direct
cause of the observed position RMSE increase. A controlled replay reducing only
the LiDAR position gain from 4/s to 1/s reduces fused position RMSE from
**10.3445 cm to 7.7114 cm**, below the existing BESTPOS-only result of 8.8252 cm.
The full heading and velocity histories remain exactly identical in that
control. Production gains, measurement inputs and the map were not changed.

| Diagnostic variant | Position RMSE | Heading RMSE |
| --- | ---: | ---: |
| Current fusion | 10.3445 cm | 0.70616 degrees |
| LiDAR position gain 1/s; all other settings fixed | 7.7114 cm | 0.70616 degrees |
| Current gains; LiDAR lateral-velocity bias adaptation disabled | 11.173 cm | 0.69043 degrees |
| Existing BESTPOS-only observer | 8.8252 cm | 1.4038 degrees |

The first three rows are fresh diagnostic replays of the frozen inputs. The
last row is the already recorded ablation. Disabling the LiDAR-derived lateral
velocity bias correction increases position error, so removing that mechanism
does not explain or resolve the position deficit in this test.

## Weight calculation and measured errors

`runSynchronousLocalizationObserver` uses

\[
W_L=I_L(I_L+0.001 I)^{-1},\qquad
W_G=I_G(I_G+16 I)^{-1}.
\]

Both position gains are nominally 4/s. The position equation uses
`4*W_G` and the XY block of `4*W_L` to correct toward their respective
measurements. These are bounded observer gain weights, not a joint Bayesian
fusion using empirically calibrated measurement error covariances.

On the 1071 observer frames with accepted LiDAR measurements:

- LiDAR XY weight eigenvalues lie between **0.998180 and 0.999941**.
  Their medians are 0.999892 and 0.999920, effectively full weight.
- GNSS weight eigenvalue medians are **0.665620 and 0.691120**.
  Its corresponding median correction gains are 2.6625/s and 2.7645/s,
  versus approximately 3.9996/s for LiDAR.
- Raw LiDAR position RMSE is **19.7148 cm**. The GNSS positions corrected
  to the observer output point using the fused heading have RMSE **6.3977 cm**
  on those same frames. This last quantity is a measurement diagnostic with
  fused-heading point correction, not the GNSS-only observer's 8.8252 cm RMSE.
- In **82.73%** of those frames, LiDAR position lies farther from the
  evaluation reference than the aligned GNSS position.
- LiDAR mean error in reference body coordinates is approximately
  `[+0.35, -3.03] cm` (forward, left). Lag-one error correlations in map X/Y
  are 0.622/0.507. The samples do not behave like independent white errors.

The registration information matrix describes local geometric constraint
strength under the selected correspondences and noise model. A positive-
definite matrix verifies local full rank; it does not certify low bias,
correct associations, map accuracy or an empirically calibrated covariance.
The very small LiDAR gain scale saturates the observer weights and largely
removes their ability to distinguish degrees of measurement reliability.

Increasing the number of measurement channels does not ensure a lower realized
trajectory error under this fixed-gain nonlinear observer. The new channel can
contribute useful heading while its noisier or biased position correction pulls
the estimate away from the more accurate position channel. Shared reference,
map and temporally correlated errors also limit an independence interpretation.

## Scope and next design decision

The gain control establishes a causal sensitivity within this replay. It does
not establish 1/s as a general optimum. No production parameter is changed.
A next implementation should calibrate position and heading uncertainty
separately on held-out drives, retain directional observability, and validate
how innovation consistency and temporal correlation affect the source weights.
Matching information should not be equated with inverse actual pose-error
covariance without that calibration. Validate both nominal operation and
GNSS-degraded/outage intervals before adopting a lower LiDAR position gain.

All reported errors use the same-drive map and shared INSPVA reference;
they do not establish independent absolute accuracy. The extra control did
not rerun perception, mapping or D2D, and does not use fine perception.

## Reproduction

From the repository root, with the completed input experiment available:

```bash
uv run --offline --with numpy --with h5py python research/coarse_fusion_diagnosis_20260918/analyze_weights.py
matlab -batch "run('research/coarse_fusion_diagnosis_20260918/ablate.m')"
```

`diagnostics.json` records the paired measurement and weight statistics;
`ablation.csv` records the three observer controls. The MATLAB driver asserts
that the position-only gain change leaves every heading and velocity value
unchanged. Full replay states remain in the ignored
`output/coarse_fusion_diagnosis_20260918/ablation.mat` artifact. No randomness
is introduced. Results were produced by the equivalent drivers in that output
folder; the archived source copies replace absolute workspace discovery with
repository-relative discovery.
