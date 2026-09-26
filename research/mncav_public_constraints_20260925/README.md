# Public-source-constrained MnCAV identification

The public-first strategy is now explicit and reproducible: freeze verified
vehicle geometry and steering conversion, use stock mass/load distribution as
**declared nominal conditions**, then identify unknown tire stiffness, yaw
inertia and steering zero from the existing real recordings. This produces a
source-backed nominal model without treating all fitted coefficients as
independently measured physical properties.

## What public information establishes

| Parameter | Public value | Treatment |
|---|---:|---|
| Wheelbase | 3.089 m | Fixed |
| Steering-wheel/road-wheel ratio | 16.2 | Fixed |
| Front/rear track | 1.734 / 1.735 m | Fixed metadata; not used by the bicycle fit |
| Stock curb mass | 2273 kg | Fixed nominal condition, not actual loaded mass |
| Stock front/rear load distribution | 55.5% / 44.5% | Fixed nominal condition |
| Stock CG-to-front/rear distance | 1.374605 / 1.714395 m | Derived from static load balance |
| Stock GVWR | 2858 kg | Sensitivity endpoint, not a measured mass |
| Loaded yaw inertia and axle stiffnesses | Not found in the checked sources | Identify from recordings |

The [2021 manufacturer specifications](https://s3.amazonaws.com/chryslermedia.iconicweb.com/mediasite/specs/2021_CH_PacificaHybrid_Specificationsp2i5bgb1foc74s1scirptivlb5.pdf)
provide steering ratio on printed p. 2, geometry/GVWR on p. 3 and curb mass/load
split on p. 4. `lf=L*rearLoadFraction` and `lr=L*frontLoadFraction` follow static
axle force balance. The document was rechecked, including the page-4 table;
these are stock-platform values, not a survey of the instrumented vehicle.

[UMN's vehicle introduction](https://www.cts.umn.edu/news-pubs/news/2021/august/mncav)
establishes the 2021 Pacifica Hybrid identity. The current
[MnCAV vehicle page](https://mncav.umn.edu/vehicle) describes Dataspeed DBW,
a front 64-beam Ouster and two rear 16-beam units, plus NovAtel GNSS. This supports
hardware identification but gives no surveyed CG-relative installation offsets.
The [Dataspeed RU Hybrid FAQ](https://www.dataspeedinc.com/app/uploads/2022/05/FCA_RU_FAQ.pdf)
independently supports ratio 16.2. Its generic vehicle-mass table is inconsistent
with the model-year hybrid manufacturer specification and is not adopted.

Targeted searches covered `MnCAV vehicle parameters`, `MnCAV mass`, `MnCAV
cornering inertia`, `MnCAV github vehicle` and `MnCAV dataset download`, together
with official UMN/MnCAV pages, the manufacturer sheet,
[MnDOT's 2023 MnCAV report](https://rosap.ntl.bts.gov/view/dot/73017/dot_73017_DS1.pdf)
and [CTS 25-14](https://conservancy.umn.edu/server/api/core/bitstreams/dc64cb11-24de-46d6-ad17-ba34d96f6f01/content).
The 2023 report was screened for platform description and physical-parameter
terms; CTS 25-14's local text was searched for mass, inertia, cornering, wheelbase,
parameter and dataset references. This search did not locate a verified public
MnCAV dynamics-bag benchmark, measured loaded mass/axle loads, tire stiffness or
yaw inertia. That is a bounded search result, not proof that none exists.
Public reports/specifications and an openly downloadable raw dataset are
separate evidence classes. No additional public bag was downloaded or used.

The recorded DBW IMU is distinct from NovAtel. Public receiver noise values
cannot be assigned to the OEM DBW stream. Prior bag inspection identifies
reference IMU family G320N, while a later public report has an E1/E2 naming
conflict. This experiment retains the prior sensor audit rather than replacing
recorded identity with a website label. Sample rates, gyro bias, wheel radii and
effective output point remain data-derived calibrations, not public constants.
See the prior [parameter audit](../mncav_parameters_20260925/README.md).

## Executed constrained experiment

[parameter_registry.json](parameter_registry.json) separates fixed public,
fixed nominal, derived, estimated and sensitivity parameters. The runner reads
this registry, fixes ratio/wheelbase and nominal mass/CG, and fits only
`log(Cf/m), log(Cr/m), log(Iz/m), steeringWheelZeroRad`. It does not fit steering
ratio or pretend the rectangular-body inertia approximation is measured.

Data and split exactly follow the preceding
[multi-recording experiment](../mncav_multidrive_identification_20260925/README.md):
11 May training and five usable May validation recordings; June 11-24
1–39.5 s adds yaw/vy training, >=40.5 s is holdout; June 09-31 is historical
transfer evaluation. The prior extraction covered 19 bags, including one
very short unusable May recording. No new bag processing is claimed here.

Nine bounded robust optimizations (three starts for each CG scenario, seed
20260925) converged. The same residual weights, <=8 s windows, 1 s burn-in and
four implicit-midpoint substeps are retained. Bounds are Cf/m and Cr/m in
[5,400], Iz/m in [0.3,8], steering-wheel zero ±0.15 rad. All final solutions
are interior. This is offline validation with centered smoothing; it is not an
online-latency or closed-loop observer test.

The nominal CG scenario is chosen prospectively from public stock data, not
by its evaluation score. [constrained_model.json](constrained_model.json) gives:

| Quantity | Conditional nominal result |
|---|---:|
| Fixed mass | 2273 kg |
| Fixed lf / lr | 1.374605 / 1.714395 m |
| Fixed steering ratio | 16.2 |
| Identified front axle stiffness | 124178.83 N/rad |
| Identified rear axle stiffness | 139550.58 N/rad |
| Identified yaw inertia | 4485.72 kg m² |
| Identified steering-wheel zero | 2.366377 degrees |
| June holdout lateral RMSE | 0.035991 m/s |
| June transfer yaw RMSE | 0.002570 rad/s |
| June transfer lateral RMSE / bias | 0.255611 / -0.241085 m/s |

These reproduce the prior joint fit's normalized dynamics. Fixing mass to a
published stock value does **not** itself improve prediction: simultaneously
scaling `m,Cf,Cr,Iz` leaves the steering-driven bicycle unchanged. Compared with
the preceding 2530.70 kg torque-based scenario, the lower absolute stiffness and
inertia above primarily reflect that explicit mass choice, not new evidence
of softer tires. Both scenarios are preserved in `conditional_parameters.csv`.

## Sensitivity and remaining uncertainty

For the assumed CG shifts, the same reference point is maintained relative to
the vehicle geometry: `lf=lf0+d`, `lr=L-lf`, and `x_output=x0+d`.
Keeping the sensor's CG-relative offset unchanged while moving the CG would
silently move the physical reference point. Its baseline x0 remains empirical,
not surveyed. Vy initial state is zero in every scenario; the shared 1 s burn-in
reduces but does not eliminate initialization dependence.

| Assumed shift d | Cf N/rad at 2273 kg | Cr N/rad | Iz kg m² |
|---|---:|---:|---:|
| -0.15 m | 135376.87 | 124680.85 | 4350.15 |
| 0 m | 124178.83 | 139550.58 | 4485.72 |
| +0.15 m | 113047.53 | 154355.93 | 4536.28 |

Despite these changes, June transfer vy RMSE spans only 0.255587–0.255634 m/s.
The data/assumptions do not strongly select CG from this small scenario family.
This is sensitivity analysis, not a CG confidence interval or a global
identifiability proof. Occupants and roof/trunk equipment affect mass, CG and
inertia, while tire pressure/load affect cornering stiffness. Published geometry
is a firmer fixed input than curb mass or a stock load split.

The three mass scenarios (2273, 2530.70 and 2858 kg) give exactly identical
predictions when absolute stiffness and inertia are scaled consistently.
The middle value is the prior torque-derived conditional mass; the last is
published GVWR. Neither added payload nor operation within GVWR is asserted.
The old uniform-planform inertia estimate is retained only in historical
artifacts; it is not frozen as truth in this constrained identification.

The unresolved approximately -0.24 m/s transfer lateral offset remains a separate
reference-frame/measurement/model diagnostic. Public parameter freezing does
not establish its cause or certify that the observer algorithm is correct.

## Reproduce and verify

```bash
uv run --offline --with numpy --with scipy --with pandas --with matplotlib --with numba python research/mncav_public_constraints_20260925/fitConstrained.py
```

This uses existing ignored exports from the previous experiment. Verification
checks all nine fit convergence flags, fixed L/ratio constraints, 57 grouped
metric rows, equality to the previously validated integrator on the nominal
geometry, no final large-state guard activation and all three mass-invariance
cases. Details are in [verification.json](verification.json). Large prediction
exports stay under `output/mncav_public_constraints_20260925/`. Research files
are committed; production configurations and observer gains are not modified.
