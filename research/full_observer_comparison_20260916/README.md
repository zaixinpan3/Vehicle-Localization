# Raw LiDAR matching versus the complete observer at identical timestamps

## Material Passport

Date: September 16, 2026. Status: VERIFIED by executed MATLAB replay and
independent Python aggregation. This audit answers whether the complete
observer's 11.74 cm full-trajectory RMSE is worse than raw map matching.
No deployed estimator, gain, measurement, map or reference was changed.

## Correct comparison

The 11.7403 cm figure scores the complete observer on 11,690 uniform 100 Hz
samples, including periods without an accepted LiDAR pose. Raw map matching
has no valid full-pose measurement during those periods. The previously
reported **23.4267 cm "LiDAR only"** is the output of the new motion-aided
observer with GNSS removed; it is **not raw LiDAR matching**. These names
must not be interchanged.

The direct comparison below uses exactly the same **1,083 accepted LiDAR
frames**, their original timestamps and the same stored INSPVA frame poses.
The last original accepted frame at 116.89954 s lies beyond the preserved
116.89 s uniform experiment; it is excluded from both sides. The earlier
1,084-frame raw baseline is 14.8634 cm; the common 1,083-frame baseline is
14.8629 cm. This one-frame difference does not explain the improvement.

| Same accepted frames | Position RMSE (cm) | Median (cm) | P95 (cm) | Maximum (cm) | Heading RMSE (deg) |
|---|---:|---:|---:|---:|---:|
| Raw LiDAR map-matching measurements | 14.8629 | 7.8758 | 33.0655 | 57.5065 | 0.4245 |
| **Complete GNSS + LiDAR + motion observer** | **9.7613** | **6.6178** | **19.4116** | **40.1027** | **0.3755** |
| New observer with GNSS removed | 13.8691 | 7.5611 | 23.9217 | 181.3939 | 0.3141 |
| Previous offline LiDAR/motion observer | 9.3392 | 6.5553 | 18.7979 | 32.8317 | 0.3141 |

The complete observer's paired position RMSE is **34.32% lower** than raw
matching; its median, P95, maximum and heading RMSE also improve. The fraction
within 5 cm increases from 25.30% to 34.16%, and within 10 cm from 61.68% to
74.33%. This is aggregate improvement, not a guarantee at every frame.

The older 9.3392 cm row is already an observer result using future-endpoint
offline reconstruction, not a pure LiDAR measurement baseline. It remains
slightly better than the new causal-relative-to-inputs full runtime on these
frames. Its full uniform score is the earlier video's 10.14 cm. The audit
does not hide this remaining performance difference or claim a matched
causality comparison with that older pipeline.

## Why full-trajectory RMSE is larger

For the complete observer on the original 100 Hz grid:

| Population | Samples | Position RMSE (cm) | Share of total squared position error |
|---|---:|---:|---:|
| All samples | 11,690 | 11.7403 | 100% |
| Both position sources active | 10,824 | 9.5650 | 61.46% |
| GNSS active, LiDAR unavailable | 865 | 26.7937 | 38.54% |
| 80--86 s | 600 | 32.7423 | 39.92% |
| Outside 80--86 s | 11,090 | 9.3429 | 60.08% |

There is also one initial LiDAR-only sample. The source-based rows and
time-based rows are overlapping partitions and must not be added together.
LiDAR-unavailable samples are only 7.40% of the trajectory but contribute
38.54% of the squared error. Raw accepted-only matching does not score those
times. This explains much of the difference between paired 9.76 cm and
whole-trajectory 11.74 cm without changing the underlying estimator.

There is a real local weakness: within 80--86 s, the 22 accepted poses have
raw LiDAR RMSE 18.3390 cm versus complete-observer 21.3400 cm. The observer
retains accumulated state error when valid poses return; it does not reset
to each measurement. A global improvement does not imply pointwise dominance.
Removing GNSS entirely raises that interval's accepted-frame observer RMSE
to 63.1347 cm, so the GNSS channel is providing useful recovery information.

## Diagnostic heading control

A diagnostic replay preserves GNSS XY, every LiDAR packet and all other
settings, while reducing only the derived GNSS heading gain from 0.5 to
`1e-12` (effectively disabled). Full-trajectory position RMSE becomes
**14.0684 cm**, maximum 79.1881 cm, while heading RMSE falls from 0.5468 to
0.3338 degrees. Thus this gain reduction improves the heading metric but
worsens position; it is not selected for production. The control does not
support simply blaming/removing the derived GNSS heading correction.
It is not a gain search, a new independent ISS result or parameter tuning.

## Exact timing and verification

`auditFullObserverLidarComparison` adds output rows at existing LiDAR event
integration boundaries. It repeats the same left-held motion and lateral
inputs on the union grid, with unchanged source packets and gains. All
original 100 Hz states are **bit-for-bit identical** for both the complete
and GNSS-removed observers. Native-frame scores therefore do not rely on
interpolating the estimated trajectory between 100 Hz outputs.

An initial exploratory linear interpolation gave about 9.73 cm. It is
superseded by the exact-boundary value **9.7613 cm**. The first native-time
lookup used exact floating-point equality and stopped on CSV timestamp
roundoff. Matching by preserved frame index and explicitly checking the
maximum discrepancy (`4.9738e-13 s`) resolves this without fitting a clock,
changing timestamps or moving measurements. The final audit completes.

Executed in MATLAB R2026a through MCP:

```matlab
setupVehicleLocalization;
report = auditFullObserverLidarComparison;
checkcode('scripts/auditFullObserverLidarComparison.m','-config=factory','-id')
```

The final script has zero Code Analyzer findings. Independent Python checks
recompute all 16 paired metric rows, check native exported errors against
saved states, verify unchanged original uniform states, reproduce all five
error-concentration rows and verify the diagnostic heading-control score:

```bash
uv run --offline --with numpy --with h5py python research/full_observer_comparison_20260916/verify_results.py
python -m py_compile research/full_observer_comparison_20260916/verify_results.py
```

Detailed frame errors and replay states remain in
`output/full_observer_comparison_20260916/`. Compact scores, findings and
artifact identities accompany this report. Existing experiments are read
without modification. No new unit tests are required for this audit-only
script; the earlier 105-test runtime result is not represented as rerun here.

All reference limitations persist: same-drive INSPVA map, reference-relative
matching initialization, GNSS/INS ODOM input with shared receiver information,
and unmeasured physical installation geometry. These are agreement metrics
against the stipulated reference, not independent absolute-accuracy evidence.
