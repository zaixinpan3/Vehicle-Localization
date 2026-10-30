# Official FAST-LIO2 computational-core baseline

This experiment-only adapter runs the pinned upstream IMU propagation,
undistortion, IKFoM iterated filter, plane residual, and ikd-Tree functions.
It replaces ROS transport/publication with sequential binary sensor input.
It does not modify the project's production observer.

Upstream: https://github.com/hku-mars/FAST_LIO at
`7cc4175de6f8ba2edf34bab02a42195b141027e9`; ikd-Tree submodule at
`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`. Upstream is GPL-2.0.
Generated upstream source and dependencies remain outside this repository.
The build rejects tracked upstream changes and unexpected revisions.

## Controls and boundaries

- `native`: upstream online map insertion, no prior map and no GNSS.
- `prior`: install a reference-built dense map at the declared initialization
  pose and disable map insertion. This is our **prior-map adapter to FAST-LIO2
  core**, not the unmodified authors' localization package. It retains native
  IMU/filter/residual equations and is the closer known-map comparison.
- Native LIO and known-map localization have different priors. A difference
  between them cannot by itself establish observer superiority.
- Runtime sensor binaries contain raw XYZ/intensity/point times and raw Ouster
  accelerometer/gyro only. No reference orientation, velocity, or GNSS is
  exported. The raw vehicle IMU orientation is invalid and is not synthesized.
- References enter the separate map builder and scorer. One reference pose
  defines the initial coordinate transform: frame 4 in drive 1, frame 401 in
  drive 2. The transform uses the *pre-update* runtime state, so it does not fit
  the first map correction away. There is no whole-trajectory alignment.
  Drive 2 warms the native filter on its first 40 seconds before this transform.
- The prior uses every raw scan, reference deskew, and 0.5 m voxel centroids.
  It is same-pass reference mapping, not independent survey ground truth.
  The project uses a different semantic probability map; priors have the same
  acquisition/reference provenance but not identical representations or points.
- Existing calibration converts LiDAR outputs to the receiver reference point.
  Runtime internal LiDAR/IMU extrinsics use the official Ouster profile's
  identity/zero nominal values; exact internal factory translation is not
  available in these bags. No extrinsic is fitted to evaluation poses.
- Our observer additionally uses wheel/steering/vehicle IMU signals. FAST-LIO
  uses raw 3D returns and the built-in Ouster IMU. These are full-system
  comparisons with different sensor interfaces, not isolated observer ablations.

## Time handling

`exportHardwareSensors.py` verifies each cloud's native origin using raw Ouster
column timestamps minus its per-point `t` field. Both available bags have zero
nanosecond disagreement for every column. Packet delivery may lag cloud
publication, so clock metadata is decoded in a first pass. Measurement values
remain causal and are processed sequentially on the shared native Ouster clock.
ROS receipt/header intervals are not used for IMU propagation.

Organized clouds are ordered by ring, not globally by firing time. Scan end is
therefore the maximum full-cloud point time, including the epoch of invalid
returns. Taking the last retained valid point produced up to 8.68 ms of epoch
error in a diagnostic precursor and is rejected in the final runs. Timing in
float32 point offsets introduces less than 4 ns disagreement after correction.
The existing receiver-clock association links each scan to its reference epoch;
absolute header-to-scan latency remains uncalibrated, as in the project replay.

## Reproduce

Install PCL, Eigen, Boost and a C++17/OpenMP compiler outside the repository.
The recorded environment used conda-forge PCL 1.15.1, Eigen 5.0.1, Boost 1.90
and GCC 16.2. Build with:

```bash
python scripts/baselines/fastlio/buildOffline.py --source /path/to/FAST_LIO \
  --deps /path/to/deps --build /path/to/build
```

Export each raw bag to a temporary `drive1` or `drive2` directory:

```bash
uv run --with rosbags --with numpy python \
  scripts/baselines/fastlio/exportHardwareSensors.py \
  --bag data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag \
  --output /var/tmp/pdvl_fastlio/drive1
```

Drive 1 scoring uses the existing corrected receiver reference. For drive 2,
run `extractGnssFromBag.py` into `drive2/gnss`, then
`prepareInspvaMappingPoses.py` to `drive2/reference_poses.csv` using its extracted
front-LiDAR clock and INSPVA streams. These files are reference-only inputs.
Build each prior with `buildPriorMap.py --drive N --temporary /var/tmp/pdvl_fastlio`
using `uv run --with numpy --with scipy --with pyproj python`.
Run each mode with the same Python dependencies:

```bash
python scripts/baselines/fastlio/runTrials.py --drive 1 --mode prior \
  --temporary /var/tmp/pdvl_fastlio --executable /path/to/build/fastlio_offline \
  --deps /path/to/deps --output output/fast_lio_baseline
```

Primary parameters are frozen from the official Ouster configuration: four
iterations, thinning 2, blind radius 4 m, 0.5 m scan/map voxels, gyro/accel noise
0.1, bias noise 0.0001, no online extrinsic estimation. A 10 km map cube avoids
pruning the fixed route prior; maximum detection range is 150 m. Voxel sizes
0.25 and 1 m are reported as sensitivity runs, never selected by test error.
Two OpenMP workers are used. Timings include preprocessing, IMU/deskew,
voxelization, matching/update and native insertion; exclude binary I/O and
prior creation. Initial prior installation is included if it lies in the score
window. Concurrent replay wall times are descriptive, not a hardware benchmark,
and cannot be compared directly with project observer-only entry times.

Exact comparison frames are 22:1169 for drive 1 (1148, the stored `after_2s`
window), and 401:778 for drive 2 (378). Missing frames retain failed scores.
`testScoring.py` verifies lever/rotation conversion, initialization-only
alignment, and missing-frame failure handling. `summarizeResults.py` exports
compact tables and figures; all per-frame arrays and map binaries are temporary.
Delete the temporary directory after retaining these compact artifacts.
