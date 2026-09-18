# Preserve the reviewed pole detector across lattice refactoring

Prepared September 18, 2026. The comparison baseline is commit
`e357ade75f1b7453095d17586a2f547bc5faa0b8`, immediately before the lattice
refactor `be01ba25590dc6cfd44c88e97df378045125acb3`. The user explicitly confirmed
that the previously reviewed frame results are the desired baseline.

## Diagnosis

The refactor changed three inputs to the calibrated detector: the XY cell phase
(lower bound -50 m became -30 m), the retained extent (about 100 m became 60 m
across), and the fine height-histogram phase (the minimum of all retained returns
became a multiple of the height-bin spacing). These are algorithm inputs, not
just alternative storage layouts. They change connected candidate footprints,
whole-pillar recovery statistics, height occupancy, and neighborhood support.

A pre-existing, uncommitted trial repeated fine seeding on three half-cell shifts
and required two phase votes before validation on the nominal lattice. Its
starting files were preserved in the ignored experiment folder with SHA-256
identifiers in `initial_manifest.json`. It was not retained: candidate votes on
one geometry do not preserve a shaft's footprint or support on another geometry.
The trial also included facade voting; that unvalidated voting path was removed
with the trial rather than introduced into the active pipeline.

The diagnostic frames were 28, 91, 214, 260, 384, 615, 746, 901, 1047 and 1137.
To separate cropping from classification, the first comparison used original
points satisfying abs(x), abs(y) < 29.8 m. The baseline contained 1,872 pole points:

| Variant | Retained baseline points | Removed | Added | Aggregate point IoU |
| --- | ---: | ---: | ---: | ---: |
| Half-cell seed voting trial | 1,557 | 315 | 675 | 0.6113 |
| Refactored single phase | 1,249 | 623 | 366 | 0.5581 |
| Restore XY phase only, still cropped and fixed-height phase | 1,650 | 222 | 173 | 0.8068 |
| Restore XY phase, support extent and fine height reference | 1,872 | 0 | 0 | 1.0000 |

For example, seed voting on frame 91 retained 98 of 143 baseline pole points,
removed 45, and added 275. Restoring only the XY phase is insufficient. Detailed
per-frame ablations, including cropped support with restored height phase, are
in `diagnosis_summary.json`.

## Current implementation

- Keep a single fixed XY lattice: 334 by 334 whole pillars at 0.3 m spacing,
  centered at [0.1, 0.1] m. Bounds are [-50, 50.2) m and cell boundaries match
  the tuned -50 + k*0.3 m partition. All XY consumers derive their geometry
  from the same configuration; the deleted ROI/alias and general voxelizer
  implementations remain deleted.
- `voxelizePillars(pillars, dz, zReference)` has one explicit reference-height
  contract. Offline refinement supplies the minimum Z of all retained returns,
  including ground, before constructing nonground height statistics. The
  compact index retains a return exactly on the top bin boundary.
- Coarse perception still processes whole pillars and their XYZ statistics;
  it never constructs height bins or point-level semantic refinements.
- Pole thresholds, recorded point expectations, and ring-independent validation
  remain unchanged. Two synthetic tests were relocated into one cell/across
  one cell boundary as their geometric fixtures require. Their statistical and
  classification assertions were preserved.

This correction preserves a calibrated detector; it does not establish grid-phase
invariance. The fine reference still depends on the retained frame minimum.
Replacing these support statistics with invariant geometric measures would be
an independent algorithm change requiring new recall and false-positive audits.
The new complete-cell upper edge extends 0.2 m beyond the previous +50 m ROI;
there is deliberately no second clipping geometry.

## Reproduction and limitations

Local experiment scripts and full mask outputs are under
`output/pole_consistency_20260918/`: `diagnose.m`, `diagnose_height.m`,
`validate.m`, and `checks.m`. `baseline/` is an immutable `git archive` export
used only for validation, not an installed fallback. Historical source snapshots,
MAT files, raw recordings and binaries are excluded from the source commit.
Both sides of paired mask validation explicitly use the MATLAB backend; a
separate check compares the corrected MATLAB and native outputs.

Point agreement measures consistency with the reviewed implementation, not
independent semantic accuracy. No full-sequence video, map rebuild or localization
experiment is claimed by this task.

## Final validation

- Paired MATLAB-backend validation on 78 Mississippi frames, selected as
  `unique([1:20:1170, 28,91,214,260,384,425,458,538,600,615,687,746,832,855,
  901,943,963,1047,1137,1170])`, used full original-point masks without the
  diagnostic common-range crop. All 12,859 baseline pole points were retained,
  with zero added points. All 78 pole masks, curb masks and traffic-sign masks
  were exactly equal. Frame results and the complete frame list are committed.
- 107 distinct MATLAB tests pass across pillar contracts, fine height-reference
  tests, pole continuity/isolation/recovery/rejection, semantic probability
  clouds, structural channels and native-kernel parity. Existing recorded
  classification expectations were not changed. The first test invocation
  lacked a root path/data-root environment; rerunning with the project root on
  the MATLAB path and `VEHICLE_LOCALIZATION_DATA_ROOT` set resolved setup skips.
  Two synthetic geometry fixtures then required the relocation described above;
  their rerun passed. `combined_tests.csv` records each test's final result.
- All feature masks from MATLAB and native execution agree on the 10 diagnostic
  frames. Native/compact-raster tests also cover frames 75,260,775,1100,1125,
  and structural tests cover both Mississippi and Downtown.
- Factory-settings Code Analyzer reports zero issues in all nine changed MATLAB
  files. A first analysis invocation reported only a missing personal settings
  file; the explicit factory-settings pass removes that environment dependency.
- After the paired validation and test jobs completed, native coarse perception
  was timed on the 10 cached diagnostic frames, excluding loading, using 11 calls
  per configuration and discarding the first. Per-frame corrected medians range
  from 51.81 to 66.45 ms; the median of frame medians is 62.79 ms versus 46.63 ms for
  the 200-cell centered grid. These samples fit a 100 ms compute budget, but they
  are not a full-sequence worst-case latency guarantee. Earlier overlapping-job
  timings in the raw diagnostic output are not used as the performance result.

The reviewed spatial partition and support extent are deliberately restored
rather than retuning acceptance thresholds around altered statistics. This
recovers the known results while retaining the single-path refactoring. The
larger default extent costs computation and must be included in any future
real-time performance evaluation.
