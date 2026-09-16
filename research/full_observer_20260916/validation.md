# Complete localization runtime: executed comparison

## Material Passport

Date: September 16, 2026. Status: VERIFIED for implementation, executed
regressions, recorded replay, deterministic reproduction and independent
metric/channel/matrix checks. Physical disturbance budgets, installation
geometry and an unconditional all-outage ISS theorem are not verified.

Question: Can the localization runtime accept GNSS and LiDAR together, keep
the other channel working when one is unavailable, and preserve explicit
limits when neither is available? The implemented answer is yes, with the
conditional stability scope in [design.md](design.md). This is a software and
recorded-data experiment, not independent field validation.

## Scope and executed commands

New public entries:

- `fullObserverConfig`, `designFullObserverGains`.
- `runFullLocalizationObserver`: both sources, sampled validity/expiry,
  causal anchor transport, admitted GNSS displacement heading, and causal
  slow LiDAR lateral-velocity correction in one persistent state trajectory.
- `runMncavFullObserverExperiment`: matched recorded ablations.
- `validateFullLocalizationObserver`: regressions and numerical checks.

The legacy independent continuous GNSS/delayed-LiDAR analyses and the
LiDAR-only motion-aided runner are retained. Documentation now distinguishes
those analysis cases from the complete runtime. No perception, registration,
mapping, raw data or vehicle physical parameters were changed.

MATLAB R2026a via the MATLAB MCP executed:

```matlab
setupVehicleLocalization;
report = runMncavFullObserverExperiment;
validation = validateFullLocalizationObserver;
```

Independent Python verification:

```bash
uv run --offline --with numpy --with h5py python research/full_observer_20260916/verify_results.py
python -m py_compile research/full_observer_20260916/verify_results.py
```

## Data and design

Drive: Mississippi `raw_data_2024-06-07-12-09-31_0`, user-confirmed UMN MnCAV.
All seven runs use the same 11,690 uniform 100 Hz times, 0--116.89 receiver
seconds, the same frozen per-frame-zero LiDAR poses/information, corrected
recorded motion, MnCAV lateral observer/gains and common initial state.

This grid covers 1,169 native LiDAR frames, including 1,083 accepted full
poses, and 5,844 original ODOM position samples. The original 1,170th LiDAR
frame is at 116.89954 s, beyond this pre-existing uniform grid, and is
explicitly excluded from this comparison rather than resampled or discarded
silently. Failed/directional frames are represented as invalid full poses.

The GNSS channel is `/novatel/oem7/odom` XY with its positive diagonal
position covariance, exactly the older GNSS input source. It is labeled
GNSS/INS aiding, not pure GNSS; ODOM yaw and velocity are not injected.
Covariance-derived weights are bounded tuning weights, not a Kalman
independence/covariance claim. INSPVA is used for the evaluation reference and
timestamp bridging; its position does not enter the runtime's measurement
interface. The existing map and per-frame matching seeds still use INSPVA.

Default gains are `[4,4,12,4]` for the existing motion-aided structure,
GNSS position gain 1, GNSS displacement-heading gain 0.5. Both maximum
measurement ages are 0.2 s. There was no recorded-error gain optimization
for this change. All scenarios initialize from the same previous
LiDAR/motion initial state; GNSS-only is consequently a measurement ablation,
not a demonstrated unaided cold-start test. No random values are used.

## Results

For a direct raw map-matching comparison, see the subsequent
[identical-timestamp audit](../full_observer_comparison_20260916/README.md):
raw matching has 14.8629 cm position RMSE versus this complete observer's
9.7613 cm on the same 1,083 accepted frames. The "LiDAR only" row below is
an observer with GNSS removed, not the raw matching measurement baseline.

Every row includes all 11,690 samples, with no outage/error trimming.
"Only" below still includes the measured motion and lateral observer.

| Available measurements / stress case | Position RMSE (cm) | Median (cm) | P95 (cm) | Maximum (cm) | Heading RMSE (deg) |
|---|---:|---:|---:|---:|---:|
| GNSS + LiDAR | **11.740** | **6.740** | 23.414 | 46.111 | 0.547 |
| LiDAR only | 23.427 | 7.766 | 30.312 | 180.903 | 0.334 |
| GNSS only | 27.421 | 17.574 | 55.301 | 77.357 | 1.439 |
| GNSS packets invalid during [40,60) s | 11.824 | 6.882 | 23.455 | 46.111 | 0.547 |
| LiDAR packets invalid during [40,60) s | 12.230 | 7.145 | 23.369 | 46.183 | 0.571 |
| Both invalid during [40,60) s | 66.774 | 8.234 | 188.736 | 262.104 | 0.542 |
| Alternating GNSS/LiDAR availability in one-second blocks | 21.462 | 15.485 | 36.935 | 87.418 | 0.338 |

The combined source result improves position RMSE by about 49.9% versus
LiDAR only in this same sampled runtime. It does not improve every metric:
heading RMSE is larger than LiDAR only, because GNSS-derived heading is
active during the natural LiDAR gaps and brings its own position/motion
reconstruction error. This is not a statistical significance claim; the
ablations are correlated runs of one drive.

With both channels enabled, 10,824 output samples have both active, 865 have
GNSS only and one has LiDAR only; none has neither. There are 5,619 admitted
GNSS heading-window updates and 558 admitted LiDAR bias-window updates.
All 11,690 sampled translation dissipation matrices have positive margins;
heading feedback is available at all output samples in this particular run.
The maximum measured course rate is about 0.355 rad/s, inside the configured
0.4 rad/s envelope. Physical disturbance and heading-region conditions are
still unverified and are not upgraded by these sampled observations.

Within the deliberately shared 20-second outage, position RMSE is 1.579 m
and maximum is 2.621 m. States stay finite and converge back after reception
resumes; this is not continued accurate absolute localization without either
source. Invalidation takes effect at each source's first corresponding
packet timestamp, not an invented synchronous packet at exactly 40 seconds.
Natural missing packets are retained in every scenario.

The earlier **10.140 cm** result used offline future-endpoint interpolation
and motion-assisted gap reconstruction, as well as a different lateral-bias
reconstruction. The new 11.740 cm result therefore must not be described as
an accuracy improvement over that earlier offline result or as a clean
single-factor GNSS ablation against it. The fair channel ablations are the
first three rows above, which share the same new runtime and sampling rules.

The operational plot is
`output/full_observer_20260916/comparison.png`; complete states, diagnostics
and inputs are in `experiment.mat`. The compact metrics and verification
records accompany this report; raw/generated binaries remain outside Git.

## Validation evidence

- **105/105 MATLAB tests pass** across the new full-runtime test class and
  existing motion-aided, continuous GNSS/LiDAR, lateral, LiDAR velocity
  consistency and MnCAV parameter classes. The new class has 20 cases.
- Synthetic tests cover both position residuals acting together, each source
  alone, both absent, invalid NaN payloads, source expiry, alternate-frame
  availability, common outages and recovery, exact straight/turning motion,
  moving GNSS heading convergence, curved displacement reconstruction,
  stationary heading nonobservability, and interface/certificate rejection.
- Repeated recorded integration has exactly zero state difference.
- Halving maximum step from 0.005 to 0.0025 s changes position by at most
  `1.6969e-7 m` and heading by `1.7567e-10 rad`.
- Replacing every future GNSS/LiDAR measurement after 60 s leaves the
  trajectory through 60 s bit-for-bit unchanged. The synthetic suite also
  checks this property through 2 s. This checks the observer relative to
  fixed supplied inputs, not upstream map/clock independence.
- Six MATLAB implementation/test files have zero factory Code Analyzer
  findings. The Python verifier compiles.
- Independent Python/HDF5 checks recompute all 21 metric rows with maximum
  discrepancy `4.44e-15`, reproduce all 81,830 source-availability decisions,
  verify zero injection for unavailable sources and bounded nonnegative
  source ages, and confirm unchanged native LiDAR packet poses.
- Independent matrix checks recover the GNSS/LiDAR/both translation margins
  `0.3679880574`, `1.8361498799`, `2.3219370365`. The common Lyapunov matrix is
  `I6`; these are dissipation margins, not expected physical accuracy.

The run/validate loop completed without a failed observer test or experiment.
A plot legend was subsequently repositioned and its outage markers removed
from the legend; this changes presentation only. Sensor/physical limitations
are retained rather than selecting lower-error reference intervals.

## Interpretation limits

The complete module now implements the requested simultaneous channels and
explicit missing-data behavior. Independent-mode ISS reasoning is preserved,
and the new structure has a separately derived conditional common bound.
It does not follow that arbitrary joint outages, stationary GNSS-only
heading, unbounded sensor error, or unmodeled delay satisfy seven-state ISS.
The data remain a same-drive, reference-seeded consistency experiment, with
shared GNSS/INS information and unmeasured installation geometry. The result
establishes implemented behavior and these measured discrepancies, not
independent centimeter-level absolute accuracy or field deployment readiness.
