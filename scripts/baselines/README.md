# Paired Mississippi baselines

These experiment-only runners leave the production observer unchanged. They
compare the current fresh-LMI observer with covariance filters using its same
eight-state intrinsic predictor, five outputs and overlap evaluator. These are
project implementations of EKF/iterated quadratic likelihood updates, not the
authors' code for another localization system. An overlap information matrix is
an empirical curvature surrogate; the likelihood temperature is not a certified
sensor covariance.

Run from the repository root. Use a temporary directory for generated arrays.

1. Extract drive 2 with `prepareMississippiBaseline.py`, using `uv run --with
   rosbags --with numpy --with scipy --with pyproj`. Supply the June 7 12:11:24
   bag and `--output /tmp/pdvl_mississippi_baselines/drive2`.
2. Export vehicle motion with `scripts/extractVehicleReplaySensors.py` into
   that directory's `sensors/`, and raw Ouster IMU with
   `scripts/extractOusterImuFromBag.py` into `front_imu.csv`.
3. In MATLAB add `scripts/baselines`. Prepare the three independent offline
   feature blocks with `prepareMississippiBaselineChunk(folder,1,260)`,
   `prepareMississippiBaselineChunk(folder,261,520)` and
   `prepareMississippiBaselineChunk(folder,521,779)`, where `folder` is the
   temporary drive-2 directory. These calls can use independent MATLAB
   processes. Every frame checks that stored distance and XYZ agree in metres.
4. Run `runMississippiBaselines` with
   `Drive=2,PrepareOnly=true`, then `tuneMississippiEkf`. The frozen tuning uses
   drive 2 before 40 s only. Run drive 2 and drive 1 comparisons afterwards.

The first drive uses the existing corrected receiver reference and all-frame
map; the second builds its all-frame map from original INSPVA poses and scores
after 40 s. Both are same-pass consistency tests. BESTPOS is INS aided and
correlated with the reference. Prior calibration/development already used these
recordings; the present tuning split is not a new blind benchmark.

The measured receiver lever is moved to the small-lever design point with one
initialized gyro-integrated heading, shared by every estimator. Its error remains
an observation disturbance. Each main experiment freshly solves global and
lateral gains. Reused temporary inputs never load gain fields. The moving-start
second recording uses one initial reference tilt and subsequent raw IMU/wheel
updates. No later reference attitude enters that runtime tilt update.

The eight variants are proposed, overlap EKF, overlap iterated EKF (at most five
iterations), GNSS-only EKF, proposed without LiDAR, proposed without acceleration
outputs, proposed with zero lateral velocity/sideslip, and proposed without
optional overlap evidence factors. The latter withdraws view/source evidence
factors; it does not refit or equalize canonical map mixture masses. All
ablations are empirical controls and do not inherit the complete ISS theorem.

Availability scenarios include both sensors, a middle 30% GNSS blackout, the
same LiDAR blackout, alternating two-second availability, no GNSS, and three
80% GNSS dropout masks (seeds 20261031--20261033). Primary scoring includes every
query frame with one exact initial reference pose and sensor-derived velocity;
there is no trajectory alignment. Secondary windows expose initialization,
withdrawal and recovery. Failure frames are retained and nonfinite runs score
infinite error. Runtime includes each estimator entry call and its own validation, and excludes gain synthesis and common offline perception/map construction. Per-frame numbers are entry-call time divided by query count, not isolated kernel timings.

Keep compact metrics, logs and figures. Delete temporary frame/map/state/gain
arrays after collection. Historical Bessafa/registration results are not reused
as current comparisons.

Drive-2 calibration preparation also builds the same causal five-scan source
horizons with uninterrupted valid wheel/lateral/gyro motion. Its motion is
integrated on the aligned frame clock; drive 1 retains the existing native-rate
preparation. No map correction or reference pose enters those horizons.

The final calibration grid uses six overlap temperatures (0.001 to 100) and
five process scales (0.1 to 1000). An initial smaller grid reached its boundary;
only calibration-prefix scores informed this extension. The final grid version
is required before test scoring. GNSS EKF tunes five process scales alone.

Before test scoring, the best nominal process candidate at each temperature is
also scored under middle-30% and complete GNSS loss on the same calibration
prefix. Equal mean squared position errors across these three regimes select
the final version-3 tuning. The GNSS-only process candidate is included.

After the main replay, run `runMississippiNominalSensitivity`,
`runBessafaMississippiBaseline`, and `runMississippiNdtBaseline` in MATLAB,
then `python scripts/baselines/summarizeMississippiBaselines.py`.

The Bessafa control re-solves the source-family continuous LMI at sigma 160,
with an explicit globally Lipschitz extension and a verified high-gain
threshold. It uses the same GNSS/speed/sideslip/gyro and no LiDAR. Compare it
with the proposed no-LiDAR control to avoid attributing a modality advantage
to observer structure. Sign/dominance/conditioning restrictions select one
conservative source-family member; they are not necessary source-theorem
conditions or a claim of the authors' best attainable implementation. A speed
above 100 m/s stops a source replay and retains a failed result. Tightened
integration checks share the same fresh gains. All unavailable-source and
low-speed source runs are outside that source's continuous guarantee.

NDT uses MATLAB R2026a `matchScans` on five deterministic sigma points per
shared Gaussian component. This preserves each mean/covariance but ignores
semantic labels and masses. It is a planar feature-front-end adaptation,
not raw-scan NDT. Cell sizes 0.5/1/2 m are calibrated on twenty drive-2 prefix
scans with supervised 0.3/-0.2 m and 2 degree seed offsets. The selected
candidate must admit every calibration scan; uncentered pose-error second
moments determine its fixed covariance. During evaluation only the current
EKF prediction seeds NDT. No test-reference pose enters registration. NDT's
positive Hessian eigenspace admits the corresponding pose update; unsupported
measurements withdraw it. Its EKF process scale equals the robust overlap-EKF
scale. These controls do not rank complete external localization systems.
