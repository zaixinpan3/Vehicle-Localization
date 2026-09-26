# Remove obsolete vehicle profiles and lateral gains

Prepared September 26, 2026, following the request to delete old parameters.

The active configuration now has one vehicle: the adopted MnCAV model in
`config/mncavVehicleParameters.json`. `lateralObserverConfig()` and the existing
explicit `"mncav"` selector load the same vehicle, output point and sensor priors.
The old sedan constants, zero-output-offset branch and unused noise defaults
were removed. `"reference"` is rejected with
`VehicleLocalization:RemovedVehicleProfile`; it is not silently mapped to MnCAV.

Removed `tests/reference/lateralObserverDesign.mat` (old sedan model) and
`tests/reference/mncavLateralObserverDesign.mat` (pre-identification MnCAV model).
The modified working copies were confirmed to contain those obsolete models
before deletion and preserved outside the repository in a temporary task backup.
No new binary gain fixture replaces them. Lateral regression tests now synthesize
from current configuration once per class, and the cascade-interface test uses
fresh MnCAV gains. These tests consequently require YALMIP and an SDP solver.

The longitudinal-speed-rate regression now compares `ax + vy*r` at the observer
point, using `observerPointLateralVelocity`, rather than the transported output
velocity. This distinction was invisible for the removed zero-offset profile.
Mismatch tests use explicit perturbations of the current model. Documentation
and examples no longer provide a route to the old vehicle or gains. Recorded
research results and their original assumptions remain unchanged; reproduce
historical experiments at their recorded Git revision.

## Verification

- Exported the staged tree `f93249e4a5970280ae5378fd2cad6ebf8c1ecc2b`
  with `git archive` into `/tmp/mncav-legacy-removal-index` to exclude independent
  uncommitted observer changes. All **90/90 tests passed** in MATLAB via MCP:
  `mncavDefaultsTest`, `mncavVehicleConfigTest`, `wheelMotionInputTest`,
  `receiverClockTest`, `lateralObserverTest`, and `improvedObserverTest`.
- The full working tree passed **90/91 tests**. Its independent uncommitted
  synthesis/test implementation adds one scheduling-rate test and a gain-bound
  assertion absent from the committed implementation. The failing assertion
  compared maximum vertex-gain norm `5.478378781907876e-10` against upper limit
  `9.458917798842216e-11`. All other checks in that test, including certificate
  checks, completed without reported failures. The near-zero inequality failure
  remains unresolved in that draft; no tolerance was relaxed or solver changed.
- `source_hashes.json` distinguishes the staged and working sources, including
  the independent synthesis/runtime changes. Those changes are excluded from
  this commit. The removal does not claim to validate that entire draft.
- Source search found no remaining old sedan constants or deleted gain-file
  references in active configuration, localization, scripts, tests and README.
  The old profile name remains only in its rejection test and explanatory docs.
- Scoped staged whitespace checks passed. Existing current physical constants
  were not changed. No real-data replay or new parameter identification ran.

The test CSVs and `validation.json` retain both outcomes. Reproduce with the
six-suite `runtests` call above after adding the project, YALMIP and SeDuMi paths.
Synthetic scenarios retain configured seed 2026; assertions check convergence,
kinematic identities, certificates and interfaces using the current model.
