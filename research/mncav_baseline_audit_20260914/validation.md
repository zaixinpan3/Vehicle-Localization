# Audit of centimeter-level LiDAR accuracy claims

Date: September 14, 2026. This is an executed analysis of saved results and
an audit of the current conversation's user-facing history. No new matching,
observer tuning, trajectory alignment or frame rejection was performed.

## What was actually reported in the current conversation

The original conversation was located using the opening request about the
Bessafa observer and the later MnCAV vehicle-parameter request. User-facing
assistant messages were inspected in their preceding-question context.
The following are English paraphrases of the relevant claims, with original
message times converted to America/Chicago. Private conversation logs are
not included in the repository or evidence exports.

| Message time | Reported quantity | Actual experiment and source |
|---|---|---|
| Sep 13, 11:12 | Virtual LiDAR position noise scale 0.05 m | Synthetic 10 Hz pose, 150 ms delay; a chosen noise scale, not a measured matching RMSE. [Standalone validation](../standalone_observer_validation.md). |
| Sep 14, 11:33 | New position RMSE 0.020 m, previously 0.168 m | Nominal MnCAV independent global-observer simulation with virtual LiDAR and exact lateral input. Saved value 0.01975 m; evaluation uses seconds 20--40. [Envelope validation](../mncav_lidar_envelope_20260914/validation.md). |
| Sep 14, 13:01 and 13:14 | Position RMSE 4.58 cm | Actual motion inputs, but pose measurement replaced with INSPVA position/reference yaw. A reference-injection diagnostic, not LiDAR matching. Saved value 0.045789 m. [Error diagnosis](../mncav_error_diagnosis_20260914/diagnosis.md). |

The 0.020 m claim explicitly stated the virtual-LiDAR/exact-lateral scope.
The 4.58 cm claim explicitly excluded a LiDAR-accuracy interpretation.
Nevertheless, reusing the same sensor name across these different
experiments can obscure the comparison. The user correctly recalled
centimeter quantities in this conversation. Which particular value they
remembered has not been confirmed. No actual all-frame map-matching RMSE
of a few centimeters was found in this conversation's reported results.

After the actual lateral-observer/map-matching experiment began, the
conversation reported about 0.288 m matching RMSE against the old mixed
ODOM reference; subsequent matched-time INSPVA comparisons were about
0.207 m at native frame times and 0.1885 m for the actual continuous input.
These populations and references must not be interchanged.

The 0.01975 m simulation used synthetic sinusoidal LiDAR errors of 0.01 m
per position axis and 0.1 degree heading. This is a different synthetic
measurement configuration from the initial 0.05 m-noise experiment.
Neither experiment executes point-cloud registration.

## Historical stored matching results

The audit script recomputes the following values from saved CSV tables;
`historical_baselines.csv` retains meters and exact counts. Accepted
multi-start trials are not independent drives or a complete time sequence.

| Result population | Count | Median (cm) | RMSE (cm) | Maximum (cm) |
|---|---:|---:|---:|---:|
| Sep 5, geometric XY, frame 900, start 1 | 1 | 4.10 | 4.10 | 4.10 |
| Sep 5, geometric XY, accepted starts, 7 frames | 18 | 7.64 | 10.36 | 16.59 |
| Sep 14, saved fine perception, accepted starts, 7 frames | 20 | 9.53 | 12.06 | 24.22 |
| Sep 14, additional saved fine perception, 38 frames | 103 | 7.61 | 13.92 | 73.07 |
| Sep 7, full recursive matching, prediction on rejection | 1170 | 10.45 | 28.14 | 156.32 |
| Sep 14, full lateral-aided matching, prediction on rejection | 1170 | 11.03 | 28.83 | 158.47 |

These historical errors use stored mapping/reference poses, including the
previously identified mixed ODOM source. Maps, perception products,
initialization and populations differ. Their table is a provenance audit,
not a controlled ranking of algorithm versions. The centimeter medians and
single-frame result are real, but not full-sequence centimeter RMSE.

## Current identical-input distribution

Use the original 11,690 uniform timestamps, continuous INSPVA reference,
unchanged zero-delay LiDAR reconstruction and saved motion-aided states.
The script checks identical integration clocks, unique increasing evaluation
samples and finite errors. No sample is removed from the primary statistics.
INSPVA is a same-receiver diagnostic reference, not independent truth.

| Position discrepancy metric | Direct continuous LiDAR | Motion-aided observer |
|---|---:|---:|
| RMSE (cm) | 18.8465 | 17.6934 |
| Mean (cm) | 13.9218 | 13.5029 |
| Median (cm) | 10.3945 | 10.5119 |
| P95 (cm) | 37.6068 | 33.7687 |
| Maximum (cm) | 96.7273 | 86.3912 |
| Samples at most 5 cm (%) | 17.6561 | 12.6005 |
| Samples at most 10 cm (%) | 47.5791 | 46.6039 |
| Samples at least 30 cm (%) | 7.8015 | 5.9281 |

The observer reduces position error at only 45.4577% of paired times.
Its reduced RMSE, P95 and peak do not imply improved median precision or
more samples within a centimeter threshold. In particular, the fraction
within 5 cm loses 5.0556 percentage points. The earlier four-metric
improvement remains numerically correct but is incomplete as a description
of typical precision. The prior validation report now links this finding.

The largest ceil(0.05*N) errors account for about 48.65% of LiDAR squared
error and 49.18% of observer squared error. Removing each method's own
largest 5% would leave RMSE 13.86 and 12.94 cm, respectively. Those are
explicitly diagnostic, differently selected subsets, not valid replacement
benchmarks or a paired performance comparison. Even this diagnostic does
not support explaining the entire accuracy gap by a few outliers.

The exported CDF shows the empirical distribution only up to 50 cm; the
full arrays and maximum-error statistics retain larger errors. Samples
are temporally dependent. No confidence interval, significance test or
independent-sample claim is made.

## Reproduction and technical conclusion

```matlab
setupVehicleLocalization;
baselineAudit = auditLidarAccuracyBaselines;
```

Inputs are the five matching CSV tables identified in the script and the
saved zero-delay/motion-aided `experiment.mat` files. Compact results are
versioned here; the full arrays and CDF PNG/vector PDF remain in
`output/mncav_baseline_audit_20260914`. No RNG is used. The execution checks
both full RMSE values against the previously saved comparison to 1e-12 m.

The recorded experiment has not demonstrated a few-centimeter localization
RMSE, nor improvement at most timestamps. Further design should assess
median, paired errors and centimeter-threshold coverage alongside RMSE and
large errors. Existing data also warrant the previously proposed controlled
map/reference-consistency experiment. This audit neither claims that such a
map rebuild has run nor attributes the remaining bias to an untested cause.

Validation: the analysis completed in MATLAB R2026a Update 3; all embedded
clock, sample and RMSE consistency assertions passed. Factory Code Analyzer
reported zero findings. The exported CDF was visually inspected. Input hashes
and actual checks are retained in `input_manifest.json` and `checks.json`.
