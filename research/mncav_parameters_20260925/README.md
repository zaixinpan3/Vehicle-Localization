# MnCAV vehicle and sensor parameter review

Date: 2026-09-25. Scope: source review, recorded sensor identity and rate audit,
explicit engineering priors, and validation of the current lateral observer.
The user authorized reasonable nominal assignments where physical parameters
cannot be found. No evaluation-drive fitting was performed.

## Adopted vehicle parameters

UMN identifies MnCAV as a **2021 Chrysler Pacifica Hybrid**. The manufacturer
specification was retrieved from its media asset host after the fleet URL
returned HTTP 403. The existing nominal dynamics agree with the published
stock data and are retained.

| Quantity | Adopted value | Basis |
|---|---:|---|
| Mass | 2,273 kg | Published curb mass; nominal anchor, actual loaded mass unknown |
| Wheelbase | 3.089 m | Manufacturer |
| Front/rear track | 1.734 / 1.735 m | Manufacturer |
| Length / width without mirrors | 5.189 / 2.022 m | Manufacturer |
| Front/rear static load fraction | 0.555 / 0.445 | Published stock distribution |
| CG to front axle, lf | 1.374605 m | Wheelbase times rear load fraction |
| CG to rear axle, lr | 1.714395 m | Wheelbase times front load fraction |
| Steering ratio | 16.2 | Steering-wheel angle divided by road-wheel angle |
| Yaw inertia | 5,874.607 kg m² | Uniform-planform engineering approximation |
| Front axle cornering stiffness | 108,238.095 N/rad | Retained unmeasured engineering prior |
| Rear axle cornering stiffness | 80,817.778 N/rad | Retained unmeasured engineering prior |

Sources: [UMN vehicle introduction](https://www.cts.umn.edu/news-pubs/news/2021/august/mncav)
and [manufacturer specifications, pp. 2–5](https://s3.amazonaws.com/chryslermedia.iconicweb.com/mediasite/specs/2021_CH_PacificaHybrid_Specificationsp2i5bgb1foc74s1scirptivlb5.pdf).

The inertia approximation is `m*(length^2+width^2)/12`. Front and rear stiffness
retain the earlier 75,000/56,000 N/rad priors scaled by `2273/1575`. They are
**complete-axle** stiffnesses, not individual-tire stiffnesses. They were not
found in a MnCAV identification report or tire manufacturer data. Keeping them
provides a reproducible starting point without presenting a different guessed
number as a measurement. Neither model mismatch nor loaded CG is identified.

The JSON now makes the assumptions and sensitivity scenarios explicit:
mass 2,273/2,473/2,673/2,858 kg; inertia and each axle stiffness at 0.7/1/1.3
times nominal; longitudinal CG shift -0.15/0/+0.15 m with wheelbase fixed.
The manufacturer lists GVWR as 2,858 kg, which supplies the last mass scenario.
These are illustrative scenarios, not recorded payloads or confidence bounds.
Previously measured sensitivity remains significant: a true -30% tire-stiffness
perturbation gave approximately 0.293 m/s lateral RMSE in the earlier campaign.

## Two distinct inertial data sources

The bag connection and first message confirm that the lateral observer input
`/vehicle/imu/data_raw` is published by `/vehicle/dbw_node`, with frame
`base_footprint`. Its physical OEM sensor model is unknown. Angular and linear
covariance fields are all zero; according to the [ROS message definition](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/Imu.msg),
this means covariance is **unknown**, not noiseless. Orientation is unavailable
(`orientation_covariance[0] = -1`). NovAtel specifications must not be assigned
to this DBW stream.

The recorded NovAtel `INSCONFIG` contains `imu_type=41`, which the
[manufacturer enumeration](https://docs.novatel.com/OEM7/Content/SPAN_Commands/CONNECTIMU.htm)
identifies as **Epson G320N**. The exact receiver enclosure is not established by
this code; PwrPak7D-E1 is consistent with it. The UMN report
[CTS 25-14, Table 1 and Figure 2(a), printed p. 5](https://conservancy.umn.edu/server/api/core/bitstreams/dc64cb11-24de-46d6-ad17-ba34d96f6f01/content)
is internally inconsistent: its table names an E2 receiver and E2-class noise,
while the adjacent figure names E1. For these June 2024 recordings, the logged
IMU identity takes precedence over that ambiguous later table.

The [NovAtel E1 specifications](https://docs.novatel.com/OEM7/Content/Technical_Specs_Receiver/PwrPak7_IMU_Specifications.htm)
give gyro range 150 deg/s, bias instability 3.5 deg/hour and angle random walk
0.1 deg/sqrt(hour); accelerometer range 5 g, bias instability 0.1 mg and velocity
random walk 0.05 m/s/sqrt(hour). These describe the E1-class reference IMU.
Random walk is not a per-sample standard deviation; bias instability is not
turn-on bias or a guaranteed maximum error.

## Rates, noise priors and geometry

| Stream | Recorded rate in receiver time | Parameter choice |
|---|---:|---|
| DBW IMU | 50 Hz | sigma(ay)=0.05 m/s², sigma(r)=0.002 rad/s, engineering simulation priors |
| Four wheel speeds | 50 Hz | Existing separate-drive effective radii and lag coefficients |
| Steering report | 100 Hz | Existing 1.5-degree steering-wheel offset, then divide by 16.2 |
| NovAtel CORRIMU | 100 Hz | Reference integrated inertial channel, not lateral-observer input |
| NovAtel INSPVA | 50 Hz | Scoring reference; not injected into lateral state |
| Front Ouster point cloud | About 10 Hz | OS1-64 reported by UMN; bag confirms 64 rows by 1,024 columns |

The clock scale is approximately 1.0909 receiver seconds per ROS-header second.
Ignoring it would misreport IMU rate as approximately 54.5 Hz. `native_sensor_rates.csv`
contains both clocks for both drives; mean rate is preferable to median interval
for the bursty CORRIMU timestamps. The original 100 Hz replay grid is an offline
interpolation grid, not the native DBW sensor rate.

On seconds 1–40 of the calibration drive, `1.4826*MAD(diff(x,2))/sqrt(6)`
gives scatter proxies of 0.04842 m/s² for acceleration and 0.0008474 rad/s
for yaw rate. These support retaining the approximate 0.05/0.002 simulation
scales, but do not identify white-noise covariance: quantization, motion,
vibration and temporal correlation remain mixed. The evaluation drive's lateral
proxy is 0.09684 m/s², motivating explicit 2x and 5x stress factors. It was not
used to select the nominal prior. The synthetic noise remains independent at
the 100 Hz simulation grid; native-sensor interpolation correlations are not modeled.

The sensor configuration also records an optional 0.1 m nominal RTK position
error scale and 0.05 m front-LiDAR range-error scale, consistent with the UMN
report's nominal sensor table. They are scenario priors, not guarantees or
replacement covariances for recorded GNSS and scan matching. GNSS degradation,
multipath and outages require separate conditions. LiDAR hardware generation
was not identified.

Existing independent-drive calibrations are retained: effective wheel radii
approximately 0.3617–0.3621 m; lateral-output effective forward offset 2.3598 m;
BESTPOS-to-output relative offset approximately [1.7249, 0.2644] m. Their
canonical JSON paths are in the sensor configuration. These are empirical
equivalents, not surveyed mounting geometry. Recorded INSCONFIG exports no
translations/rotations; that absence does not establish zero physical offsets.
Actual latency remains unknown. Zero added delay is only a nominal simulation
assumption; 20/50/100 ms are declared stress scenarios, not measured delays.

## Implementation and validation

- `config/mncavVehicleParameters.json` retains all existing model values and
  adds confirmed stock fields, refreshed sources and explicit assumptions.
- `config/mncavSensorParameters.json` separates DBW input, NovAtel reference,
  wheel/steering/LiDAR parameters, timing, calibration and assumptions.
- `mncavSensorConfig()` loads that catalog. `lateralObserverConfig("mncav")`
  now obtains its two simulation-noise values from it and carries the sensor
  provenance. The reference profile and numerical MnCAV defaults are unchanged.

43/43 tests passed across vehicle configuration, lateral observer, wheel speed
and motion-input suites, including the existing gain-resynthesis regression.
Current MnCAV model parameters equal those in the stored design, so no new
gain synthesis or binary design replacement was needed. Factory Code Analyzer
reported zero findings in all three affected/new MATLAB files.

60 matched-model simulations used 24 seconds, 100 Hz, seeds 2026–2045 and
noise factors 1/2/5. Mean post-6-second output-point lateral RMSE was
0.005461/0.010776/0.026826 m/s; all remained finite. Replaying both saved
recorded-input trajectories yielded exactly the prior lateral outputs:
12:11:24 RMSE 0.055173 m/s and 12:09:31 RMSE 0.250170 m/s, with the latter
still carrying -0.227813 m/s bias. Source qualification does not eliminate this
bias, and no improved real-world accuracy is claimed.

```bash
uv run --offline --with rosbags python research/mncav_parameters_20260925/auditSensorSources.py
```

```matlab
setupVehicleLocalization;
addpath(genpath('../RobustVehicleLocalization/external/YALMIP'));
addpath(genpath('../RobustVehicleLocalization/external/sedumi'));
addpath('research/mncav_parameters_20260925');
summary = validateMncavParameterSet();
```

`source_hashes.csv` records the inputs and locally downloaded source PDFs;
`output/mncav_parameters_20260925/validation.mat` retains full validation data.
The validation depends on existing local datasets and the prior performance
experiment. Pre-existing uncommitted observer/synthesis/design edits remain
in the working tree; their versions are identified by the preceding evaluation
manifest and this review's source manifest. Only this task's source-catalog,
configuration integration, audit and compact validation artifacts are committed.
