# Fine pole shaft completion at pillar boundaries

## Finding and change

Mississippi frame 1047 contains a user-identified shaft around (16.6, -4.3) m
in XY. None of the 14 supplied points entered the fine pole candidate set.
The main whole-pillar distribution has 26 points, 3.43 m vertical span,
7.1 cm radial RMS and 2.51 degrees fitted inclination. It narrowly failed the
2-degree recovery seed limit. Ten neighboring shaft returns occupy another
pillar whose short detailed support run cannot independently seed recovery.

Raise the offline recovery seed inclination limit to 3 degrees. Complete
only the immediate XY neighbors of the original recovery seeds. A neighbor
must have at least six points, at least 1.5 m vertical span and overlap with
the seed, inclination at most 3 degrees, intrinsic radial RMS at most 0.10 m,
and RMS distance to the seed's fitted XY-versus-Z axis at most 0.15 m.
The latter uses the entire neighbor's mean and covariance, before trimming.
Facade candidates remain excluded. Completion does not recursively propagate.
The existing final shaft geometry and neighborhood isolation tests still
control per-point labels. Coarse processing remains whole-pillar statistics.
No ring, original index, frame number or coordinate region controls inference.

For seed slope s, neighbor covariance C and mean displacement d, the mean
squared XY residual is Cxx + Cyy - 2*s*[Cxz,Cyz]' + Czz*sum(s.^2)
+ sum((dxy-dz*s).^2). This distinguishes a shared shaft from a neighboring
parallel object even when both are independently slender.

## Validation

Baseline commit: `26caf07f6e920b7529c10d68f4d1767eff90274e`.
The prior 2-degree seed limit and zero neighbor allowance exactly reproduce
the saved complete baseline output on frame 1047. Paired comparisons use
these settings against the new defaults, with the short-support boundary
excluded using `2-eps(2)` to reproduce the prior strict comparison.

All 14 supplied points are now pole labels. Frame 1047 pole count changes
61 to 88; curb count stays 128 and traffic-sign count stays zero.
Across 39 Mississippi and 10 downTown frames, 43 outputs are unchanged.
All non-pole masks and coarse probability clouds match on all 49 frames.
Nine old pole labels are removed from the user-identified false candidate;
155 labels are added, and total pole labels change 8978 to 9124.

| Sequence | Frame | Pole count before | After |
|---|---:|---:|---:|
| Mississippi | 326 | 136 | 156 |
| Mississippi | 825 | 95 | 119 |
| Mississippi | 1047 | 61 | 88 |
| Mississippi | 1125 | 62 | 73 |
| downTown | 100 | 205 | 216 |
| downTown | 150 | 571 | 624 |

The other five changed frames have no new manual labels. Their changes are
regression effects, not evidence of increased accuracy. Frame 91 false pole
indices 47906, 16728, 43363 and 42593, and frame 687 index 47135, remain
rejected. Frame 260 true pole index 63010 remains detected; the reviewed
91, 260, 425, 687 and 832 masks remain identical.

`poleShaftCompletionTest` checks shared-axis recovery, separate parallel
shafts, diffuse returns, tilted neighbors, disjoint height and nonrecursive
completion, plus recorded-frame detection and unorganized permutation with
seed 104734897. Explicit regression deltas reside in
`tests/reference/finePoleShaftCompletion.json`; the immutable reference is
preserved. Local outputs and comparison/test scripts are exported to the
technical archive. This task does not rerun the full-route video or mapping,
and does not establish a new real-time performance or accuracy claim.

## Follow-up false pole at index 60330

The user identified original index 60330 as a false pole in the same frame.
It was already labeled pole by the saved baseline. Its connected candidate
has 36 points, 4.71-degree fitted inclination, 13.79 cm supported radial RMS,
and four qualifying 0.5 m height cells (2 m support in total). The local
object support ratio is 0.7893; the wider isolation core fraction is 25/30,
which passes 0.80. Trimming to a 0.10 m core concealed the diffuse candidate.
The short-support test previously excluded exactly 2 m from its RMS check.

Apply the existing 0.10 m whole-supported-object RMS requirement at that
boundary when local support contrast is weak (below 0.80). Objects with less
than 2 m support still receive the existing unconditional RMS check.
Strongly separated objects at 2 m retain their existing validation. This
rejects the candidate's nine final labels, including 60330, while retaining
all 14 user-specified true shaft points. No point-coordinate exception is used.
An initially tried unconditional inclusive 2 m check removed 635 labels in
49 frames; it was narrowed to weakly separated candidates to preserve the
accepted baseline. Those intermediate removals are not claimed as improvements.

Final validation: 107 MATLAB tests passed, zero failed or incomplete; factory
Code Analyzer reported zero findings in the five affected MATLAB files.
The native niri frame-1047 preview retained the user's camera and showed
88 pole points, with all supplied true shaft indices accepted and index60330
excluded. The next requested preview is frame538; changing previews alone
is not recorded as a separate engineering activity.
