# MnCAV parameter basis and observer validation scope

Date: 2026-09-14. The project vehicle is UMN MnCAV. Future vehicle-dependent
experiments should use the explicit MnCAV parameter entry, with identified
parameters replacing nominal priors when available. The prior mass-free
global-observer tuning was a generic kinematic experiment, not a calibrated
MnCAV experiment. The earlier 1575 kg synthetic cascade used a generic sedan.

## Source-qualified vehicle entry

UMN identifies its MnCAV research vehicle as a **2021 Chrysler Pacifica Hybrid**:
[UMN CTS, 2021-08-24](https://www.cts.umn.edu/news-pubs/news/2021/august/mncav).
The manufacturer's stock specifications list 2273 kg curb mass, 3.089 m
wheelbase, 55.5/44.5 percent front/rear static loading and 16.2 steering ratio:
[2021 Pacifica Hybrid specifications, pp. 2--4](https://www.stellantisfleet.com/content/dam/fca-fleet/na/fleet/en_us/chrysler/2021/Pacifica/specifications/2021_CH_PacificaHybrid_Specifications.pdf).
Both sources were checked on 2026-09-14.

| Parameter | Nominal value | Basis |
|---|---:|---|
| Mass | 2273 kg | Published stock curb mass; actual instrumented/occupied mass unknown |
| Wheelbase | 3.089 m | Published stock specification |
| CG to front axle, lf | 1.374605 m | Derived from stock rear-load fraction times wheelbase |
| CG to rear axle, lr | 1.714395 m | Derived from stock front-load fraction times wheelbase |
| Steering ratio | 16.2 | Published overall steering-wheel/road-wheel ratio |
| Yaw inertia | 5874.60733 kg m^2 | Unmeasured rectangular-planform approximation |
| Front axle cornering stiffness | 108238.09524 N/rad | Unidentified mass-scaled prior |
| Rear axle cornering stiffness | 80817.77778 N/rad | Unidentified mass-scaled prior |

Inertia uses `m*(5.189^2+2.022^2)/12`, with stock length and width excluding
mirrors. Stiffness uses the historical 75000/56000 N/rad generic values scaled
by 2273/1575. These dynamic priors are retained consistently with the existing
MnCAV calibration export; they are **not manufacturer values or measured
MnCAV parameters**. No payload, sensor-rack mass, tire identification, or CG
shift is fabricated. Existing direct identification with negative inertia/rear
stiffness remains rejected and was not rerun or relabeled as successful.

The single reusable source is `config/mncavVehicleParameters.json`, exposed by
`mncavVehicleConfig()`. `lateralObserverConfig("mncav")` selects this vehicle.
The calibration exporter now reads the same source rather than retyping its
vehicle numbers. Its numerical vehicle/steering values match the prior local
export exactly. Sensor-axis/offset calibration remains sequence-specific and
separate from vehicle parameters. The Python edit passes syntax parsing; no
new data calibration is claimed.

```matlab
setupVehicleLocalization;
parameters = mncavVehicleConfig;
lateralCfg = lateralObserverConfig("mncav");
lateralDesign = designLateralObserverGains(lateralCfg); % requires SDP tools
report = validateMncavObserverParameters;
```

Fresh lateral gain synthesis for this nominal MnCAV model was executed and
passed its existing verifier. Its saved design is in the local output folder.
Never attach generic-sedan stored gains to a changed bicycle model. Historical
no-argument/reference lateral configuration is retained for archived regression
fixtures; it is explicitly separate from the target vehicle entry. The prior
generic demonstration remains historical and should not be cited as MnCAV
validation.

## Global observer versus vehicle model parameters

The seven-state global prediction/correction does not directly use mass,
wheelbase, yaw inertia or stiffness. It uses motion measurements, the lateral
interface, pose information and timing. Vehicle parameters affect the physical
motion and the upstream lateral estimator; gain choice must also reflect the
actual motion/disturbance and sensor envelope. Merely changing a mass field
cannot establish that a global observer certificate covers MnCAV operation.

To check the parameter basis without invoking other system modules, this
experiment generates true motion from the source-qualified 2-DOF MnCAV bicycle
plant and supplies its exact lateral velocity/sideslip/rate to the production
global observer. It does not run perception, mapping, registration or the
lateral estimator. The nominal plant is integrated with ode45 at relative
1e-10 and absolute 1e-12 tolerances, from -0.15 through 40 s. It starts in the
instantaneous bicycle equilibrium at -0.15 s. Global yaw/position are integrated
alongside vy/r; body and global acceleration follow the plant derivative.

Longitudinal speed is prescribed as `8+1.2*sin(.35*t)` m/s. Steering-wheel
angle is `.006+.002*sin(.4*t)` rad and is explicitly divided by 16.2 before
entering the road-wheel bicycle equation. The input amplitudes are synthetic
choices for a gentle-turn test inside the existing LiDAR certificate, not
measured MnCAV commands or a representative full-drive distribution. The speed
lies within the recorded vehicle's range; the steering is deliberately much
smaller than the full recorded range.

Reference and tracking profiles each run on seven plants: nominal and separate
0.7/1.3 factors on inertia, front stiffness and rear stiffness, with other
values held fixed. This is 14 actual 40 s global-observer runs. It varies each
unidentified parameter separately, not only a common factor. It is not a
validated uncertainty set, a loaded-mass sensitivity study, or a lateral-stage
model-mismatch experiment: each plant's exact lateral interface is supplied.

Every run retains the preceding gain comparison's continuous virtual pose,
150 ms delay, 1e6*I information, sinusoidal pose noise (.01 m per axis and
.1 degree yaw), bounded high-rate noise (.02 m/s speed, .02 m/s^2 acceleration,
.0002 rad/s gyro), and initial state/history error
`[1,.3,.1,-1,-.2,-.1,10 degrees]`. Samples are 100 Hz and integration steps at
most 5 ms. No RNG is used. RMSE is evaluated over 20--40 s; peaks cover all
40 seconds. The source information/noise are synthetic assumptions and are
not claimed as the actual MnCAV sensor calibration.

## Results and actual operating-range limitation

| Nominal MnCAV plant metric | Reference | Tracking |
|---|---:|---:|
| Position RMSE (m) | 0.083786 | 0.067387 |
| Velocity RMSE (m/s) | 0.25003 | 0.19999 |
| Acceleration RMSE (m/s^2) | 0.25846 | 0.20808 |
| Heading RMSE (deg) | 0.057242 | 0.057241 |
| Velocity-error peak (m/s) | 1.1115 | 1.2016 |
| Acceleration-error peak (m/s^2) | 0.39815 | 0.53996 |

The approximately 20 percent settled-error reduction persists in all six
single-parameter sensitivity cases. Small metric changes across the vehicle
priors are unsurprising under such gentle steering and an exact lateral
interface; they do not establish full-cascade robustness to uncertain vehicle
parameters. All 14 matrix checks pass and no supplied course-rate excursion
is reported; the maximum true rate among the plants is about 0.001760 rad/s.

The actual recorded drive is materially broader. The 11,693-row steering
export for `raw_data_2024-06-07-12-09-31_0` has speed range 0--14.91945 m/s
and maximum absolute road-wheel angle 0.191555 rad after the 16.2 conversion.
The previous saved continuous GNSS replay audit reports maximum supplied
course rate **0.354975 rad/s**. That audit was reused, not recomputed as a
new LiDAR replay. These records are identified in `parameter_audit.json`.
The current LiDAR reference and tracking certificates cover only
**0.00215032 rad/s**. Changing the nominal vehicle parameter source does not
close this gap. Accordingly, **tracking remains a provisional gain candidate
for MnCAV gentle-turn studies, not a validated full-operation MnCAV setting**.

The next full-vehicle parameter selection must account for observed course
rates and stops, actual lateral-interface errors, physical loaded mass/inertia/
tires, and sensor/pose delay and information characterization. No unsupported
full-vehicle certificate is substituted for the current narrow example.

## Checks and artifacts

- 22/22 MATLAB tests pass: 15 lateral observer tests, 3 new vehicle-source
  tests and 4 current LiDAR tracking-profile tests.
- Nominal MnCAV lateral gain synthesis passes its existing verification and
  is saved as `output/mncav_parameter_validation_20260914/lateral_design.mat`.
- Four changed/new MATLAB files have zero factory Code Analyzer findings.
- Python calibration source parses and central values equal the previous
  MnCAV export exactly; final diff passes whitespace checks.
- Full physical scenarios, exact inputs, global states, source provenance,
  lateral design and audit are in `output/mncav_parameter_validation_20260914/`.
  CSV/JSON summaries and this report are versioned in this directory.
- No data calibration rerun, identified inertia/stiffness, real LiDAR tuning,
  complete cascade validation or parameter optimum is claimed.


## Subsequent operating-envelope update (2026-09-14)

The later `mncav` LiDAR profile replaces the zero-rate norm-ball enclosure
with a structured four-vertex condition, re-synthesizes its matrices, and
covers |q|<=0.4 rad/s at theta=2 and 150 ms delay. It addresses the rate gap
identified above; the historical tracking profile and this gentle-turn
experiment retain their original bounds. See the
[new proof and wider-turn results](../mncav_lidar_envelope_20260914/validation.md).
