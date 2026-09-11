# Qualified-output continuity for broad pole fits

## Report and diagnosis

The user identified Mississippi frame 615, OriginalIndex 38985, at
`[12.873674, -9.545250, 1.940620] m` as a false pole return.
The baseline is commit `17821231a206358abb9520fd4141223b0bb32af0`.

The candidate contained 56 points and produced 25 pole labels. Its six qualified
0.5 m fine support cells summed to 3.0 m, with fitted tilt 1.95 degrees and
supported radial RMS 0.126 m. The final labels comprised a 22-point lower run
ending at Z = 0.036317 m and three upper points at Z = 1.940620 to 2.2463 m.
The output gap was 1.9043 m. Axis-consistent candidate returns in unqualified
cells bridged that gap in the old continuity check.
Only 38985 is user-annotated; 39751 and 40136 are inferred members of the same
short detached output fragment.

## Change

Fine pole validation still establishes continuity from actual axis-consistent
returns. When the supported fit exceeds the existing 0.10 m compactness limit,
it additionally validates the qualified final labels using the existing metric
run rule: gaps above 0.75 m split runs, and each run must contain at least six
points and span at least 1.0 m. Independent recovery retains its mandatory
output-run validation. Compact shafts can continue using sparse unqualified
returns to establish continuity.

An unconditional output-run check was evaluated first. It removed 124 labels
across 58 frames, including previously accepted sparse shafts, and failed four
pole regressions. That prototype was replaced. For example, the confirmed
frame-214 shaft has radial RMS 0.075 m and must retain its sparse support.
The production change uses no frame IDs, original indices, ring numbers, or
scene-specific coordinates. Coarse whole-pillar processing is unchanged.

## Validation

The final 58-frame comparison changes 8968 pole labels to 8933 (-35, no
additions), leaving 53 pole masks identical. Changes are Mississippi 100 (-14),
900 (-4), 950 (-6), 615 (-3), and downTown 350 (-8). The additional 32 removals
are unannotated and remain an accuracy-review limitation. Previously reviewed
pole frames 91, 214, 260, 384, 425, 687, 746, 832, 901, 1047 and 1137 retain
identical pole masks. Every compared non-pole mask and coarse probability
product remains identical. Frame 615 changes 212 to 209 pole labels, with all
118 curb labels and zero signs unchanged.

All 92 tests passed: 24 pole-specific tests plus 68 current-reference,
feature-selection, pipeline, and whole-pillar tests. Code Analyzer reported
zero findings in the three modified MATLAB files. The native preview check
confirmed identical camera properties and updated frame-615 labels.
Measurements are in `output/pole615_review_20260911/`. The reported fragment regression compares
all frame-615 pole labels with the stored baseline minus the three fragment
indices, then repeats inference on unorganized shuffled XYZ points with seed
61538985. Historical fixtures remain immutable; explicit output revisions
record any additional changes, which are regression expectations rather than
independent ground truth.

The fixed-view 1170-frame video exported earlier on this date remains an
original artifact of the baseline perception version. This correction does
not retroactively change that video.
