# Continuous observer ISS: derivation and numerical checks

Date: 2026-09-13. Starting project version:
`a7fbb5e3452e603f553fb2add6413148d330e1cf`.

The current [derivation](../../improved_observer_derivation.md) replaces the
combined GNSS/LiDAR theoretical baseline with two continuous measurement
contracts. The seven-state dynamics, corrected jerk residual, and four
auxiliary outputs are retained. GNSS position uses a triangular gain and a
local yaw-sector proof. LiDAR uses its fixed delayed pose minus the estimated
pose at the same delayed time, with a constant-matrix Lyapunov--Krasovskii
certificate. No delivery or correction-pulse model is used in either proof.

The old GNSS base channel included LiDAR yaw and therefore could not prove a
GNSS-only seven-state result. At standstill, all headings are indistinguishable
under position-only GNSS and the auxiliary outputs. The new GNSS theorem
states sustained motion and gives an explicit initial-error/disturbance
condition that keeps the heading error inside a valid sector. An independent
GNSS heading sensor is a separately stated alternative, not presumed hardware.

The delayed-LiDAR theorem treats the past estimation error as part of the
continuous differential equation. Its proof includes the disturbance in both
the quadratic and derivative-integral terms, and uses a first-interval bound
to establish ISS for arbitrary continuous initial histories. It does not
assume an unprovided derivative bound on the initial history.

## Reproduction

With YALMIP and SeDuMi already available on the MATLAB path:

```matlab
setupVehicleLocalization;
report = validateContinuousObserverProof( ...
    'research/continuous_observer_iss_20260913');
```

The validation ran in MATLAB R2026a Update 3 with seed 20260913. The existing
external installations in the sibling `RobustVehicleLocalization/external/`
folder supplied YALMIP and SeDuMi; they were not modified or copied. Numerical
matrices and exact scalar outputs are in
[certificate_checks.json](certificate_checks.json). The check driver is
[scripts/validateContinuousObserverProof.m](../../scripts/validateContinuousObserverProof.m).

## GNSS constructive certificate

For each position chain, use $K=[3,3,1]^\top$, $\theta=20$, and the three
nonzero auxiliary channels recorded in the JSON. The fixed Lyapunov equation
has residual **1.2413e-15**. The analytic perturbation bound in equations
(11)--(12) covers the entire operating box, including all its output-Jacobian
vertices:

| Quantity | Value |
|---|---:|
| Course-rate magnitude | at most 0.6 rad/s |
| Velocity component magnitude | at most 16 m/s |
| Acceleration component magnitude | at most 5 m/s² |
| Uniform six-state matrix margin $a$ | 7.7504704575 |

This verifies the translational block in Theorem G without sampling the
uncertainty box. The seventh-state conclusion additionally requires the
motion, heading sector, positive yaw gain, and admission conditions in
(13)--(18). No numerical claim about a particular physical initialization or
sensor-noise budget is made. These gains are an existence example, not a
performance-tuned replacement for the stored runtime gains.

## Fixed-delay LiDAR example

The 28-by-28 Schur LMI was solved with $d_L=0.15$ s, $\theta=1$ and
$\rho=0.05$ s$^{-1}$. The nominal drift uses zero course rate, unit pose
weight, and zero auxiliary gain only to construct the center of a certificate.
The final certified family includes nonzero gains on **all four** auxiliary
channels, nonzero course rate, and arbitrary symmetric anisotropic pose
weights in the stated sector.

To establish the whole family, let $\Delta A_0,\Delta A_d$ denote perturbations
of the two drift matrices. The change in the affine Schur LMI is bounded by

$$
\|\Delta\mathcal L\|\le
2(\|P\|+d_L\|R\|)(\|\Delta A_0\|+\|\Delta A_d\|).
$$

The script assigns a strict fraction of the recovered numerical margin to
course-rate, auxiliary-Jacobian and information-weight uncertainty. The
analytic bounds $|q|\sqrt{q^2+4}$, $\|N\|\sqrt{16\bar v^2+4\bar a^2+2}$ and
$\|K\|(1-w_*)$ cover these three perturbations at $\theta=1$ and $S_p=I_3$.
This is a norm enclosure of the full matrix sector, not a check of two scalar
weight endpoints or a finite list of weight orientations.

| Quantity | Value |
|---|---:|
| Delay | 0.150000 s |
| Exponential rate | 0.050000 s$^{-1}$ |
| Recovered nominal block margin | 0.2134620565 |
| Final analytic uniform margin | 0.1185900177 |
| Course-rate magnitude | at most 0.00215031982 rad/s |
| Pose-weight sector | $0.9985200484 I\preceq W\preceq I$ |
| Velocity/acceleration component bounds | 16 m/s / 5 m/s² |
| Disturbance coefficient $g$ | 999.9996832 |

The positive margin demonstrates that the continuous delayed theorem is
nonvacuous with a full seven-state, four-auxiliary-output design. The
course-rate range is narrow and the information lower bound is high. This
example does **not** certify the runtime's 0.6 rad/s course-rate envelope,
its earlier information sector, its gains, or its correction logic. The large
$g$ and modest rate also show that this is a conservative feasibility example.
Floating-point margins were recomputed from the recovered matrices; no
interval-arithmetic enclosure of solver rounding was performed.

## Algebra and document checks

One hundred deterministic random cases independently compared the true-minus-
estimated nonlinear model derivatives with the delayed error equation. Another
calculation expanded the functional derivative and compared it with the Schur
quadratic form. An affine-history integral checked the weighted Jensen sign.

| Check | Result |
|---|---:|
| Maximum delayed error identity residual | 5.1023e-15 |
| Maximum functional/Schur identity residual | 3.6380e-12 |
| Minimum weighted Jensen slack | 2.7182e-4 |
| MATLAB Code Analyzer findings in the new driver | 0 |

An independent NumPy 2.5.3 check reloaded the JSON and enumerated all **4,096**
GNSS output/course-rate vertices. The worst Lyapunov eigenvalue was
**-18.2675680686**, consistent with the conservative analytic bound. It also
reconstructed the LiDAR Schur matrix and uniform perturbation margin, and
differentiated the actual integral functional using 64-point Gauss--Legendre
quadrature and a central step of $10^{-5}$ s. The functional derivative's
relative residual was **1.4477e-9**. Reproduce with
`python3 research/continuous_observer_iss_20260913/verify_export.py`;
[the independent results](independent_checks.json) record the settings.

The derivation was rendered using Pandoc and pdfLaTeX. An initial compilation
identified two equation tags inside `aligned`; moving the tags outside their
inner environments corrected the error. The final PDF is readable and has no
overfull boxes. The generated LaTeX and PDF are local exports under
`output/continuous_observer_iss_20260913/`, excluded from the project commit.
The source document is English throughout.

The local Bessafa et al. PDF was read at Section 4, equations (41)--(58), and
its author-institution record was verified. Fridman's 2014 tutorial provides
the cited delay-analysis framework; the mode-specific equations and ISS bounds
are derived in this project document. No paper is redistributed.

No vehicle simulation, dataset replay, production gain replacement, runtime
migration, or full repository test suite was run. The new driver evaluates
mathematical certificates and algebra only. The previous event-based runtime
and its historical results remain explicitly outside this theorem's scope.
