# Human-confirmed pole missing from the fine reference

During inspection of the Mississippi frame-900 comparison, the user confirmed
that the displayed coarse pole is a real pole, while the original fine
perception reference does not label it. This is a human review of one selected
pillar, not a new point-level annotation of the entire scene.

The displayed coarse output has one pole pillar, ID 5744. The audit reproduces
that selection and verifies that no frozen original fine pole point projects
into it. The raw frame still exactly matches the stored reference XYZ.
This establishes a reference-empty, human-confirmed positive cell. The
physical class judgment comes from the user's inspection; the script verifies
the selection, geometry and reference mismatch, not physical class truth.

## Evaluation decision

- Preserve the original 0.3 m fine reference and all historical comparison
  counts. They remain reproducible detector-agreement measurements.
- Record the user review separately as `pillarContainsPole=true`. Do not mark
  every raw point in the mixed 0.6 m pillar as a pole point: exact point labels
  were not supplied.
- Under the old reference-empty rule, this frame has one extra selected pole
  cell out of one selected cell. Under the user's physical review, this one
  reviewed selection is correct. Neither statement supplies true recall over
  all poles in the scene or sequence.
- Reference-empty selection is an automatic **review candidate**, not a
  confirmed physical false positive. Future physical-precision calibration
  should use independently reviewed positive/negative/unknown cells and keep
  review coverage explicit. Unreviewed disagreements must remain unknown.
- Do not tighten pole thresholds merely to remove this reviewed true pole or
  train on an assumed negative label derived from its missing fine reference.
  This audit changes neither the deployed classifier nor its training labels;
  any retraining needs an explicit review-aware dataset and separate validation.

The previous 10% operating limits refer to reference-empty selected-pillar
fractions. This example shows why those fractions cannot be reported as
measured physical false-detection rates. It does not determine the size or
direction of the sequence-wide bias, since the old detector can also contain
incorrect positive labels.

## Reproduction

```bash
matlab -batch "setupVehicleLocalization; addpath('research/pole_reference_audit_20260928'); auditFrame900"
```

`frame900_audit.json` stores the exact geometry, original-point IDs in the
selected cell, frozen-reference metrics and the scope/provenance of the user
review. These point IDs identify the cell's support; they are not point-level
positive labels. Original reference caches and raw recordings remain in their
existing local locations. `validation.json` records the executed checks and
hashes of independent supporting artifacts. The existing niri frame-900
window remains the comparison shown to the user.
