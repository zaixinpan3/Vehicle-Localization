# Reject cluttered fine pole candidates

Date: 2026-09-10. Baseline commit:
`e07f0613453eecd1a3b4cfebad3c4c1da028116b`.

The user reported Mississippi frame-91 point 42593, XYZ
`[8.622111 -13.152769 -1.264802]` m, and its surrounding pole-labeled
returns as false positives. This is the same candidate as earlier point 43363.
The required landmark is a vertically supported, isolated shaft; trimming a
cluttered object's points to a narrow fitted axis is insufficient.

## Diagnosis and rule

The reported candidate has about 2.532 degrees fitted tilt, 2.5 m qualified
vertical support, radial RMS 0.1195 m, and mean local voxel support fraction
0.7648. The previous low-contrast radius cap removed peripheral returns but
retained 28 points. Thus tightening tilt or testing only the final narrow
core does not address this failure mode.

The fine validator now combines local and wider spatial evidence:

1. Keep the existing vertical line fit, qualified shaft support, short-support
   spread rejection and robust point-level distance tests.
2. When mean qualified object/neighborhood voxel support is below 0.80,
   inspect **all retained nonground raw points**, including returns outside
   the candidate's XY pillars, at the qualified shaft heights.
3. Measure distance to the fitted axis at each return's Z. At least 75% of the
   returns within a 0.75 m radius must fall inside the 0.25 m shaft core.
   Otherwise reject the entire candidate before point trimming. The radius
   values bound the isolation measurement; accepted weak-contrast points still
   obey the existing tighter 0.10 m radial cap.

For the reported group, 54 of 74 neighborhood returns lie in the shaft core:
72.97%, below 75%. All 28 previously accepted points in this group are now
rejected. Two other low-local-contrast frame-91 candidates also fail the
isolation test, removing another 50 points. Their correctness has not been
independently labeled.

This is a combined geometric test, not a guarantee of an entirely empty
neighborhood. Strong local separation remains sufficient; the wider raw-point
test guards ambiguous low-contrast candidates. It tolerates a limited number
of surrounding returns and excludes unsupported height intervals, so ground
adjacency and end attachments do not automatically invalidate a shaft.

Exploratory checks that counted the candidate's entire observed height range
also counted clutter outside the validated shaft and could reject the known
true frame-260 pole. A universal fixed narrow-core test removed substantially
more reference points, including wider shafts. Neither exploratory detector
was added to production. The current implementation is fine-only, uses XYZ
rather than scan order/rings/indices, and leaves the whole-pillar coarse stage
unchanged. No new coarse subdivision or neighborhood raw-point pass is added.

## Validation and limits

Thirty frozen-reference frames plus Mississippi 91/425 were compared with
this same pipeline with the new isolation fraction threshold disabled:

- Full non-pole masks and coarse probability clouds are identical on all 32.
  No pole points are added; 22 complete frame masks are unchanged.
- The 30 reference frames retain 5787 of 6033 preceding pole points (95.92%),
  with 246 removed. Across all 32 frames, counts change 6279 to 5955.
- Frame 91 changes 225 to 147. The entire reported 28-point neighborhood is
  empty of pole labels; prior false-positive indices 47906, 16728 and 43363
  remain excluded. Curbs stay 132 and signs stay 126.
- Frame 260 retains all 136 pole points, including original index 63010 and
  its recovered neighborhood. Frame 425 retains all 21 pole points and its
  full previously approved mask.
- Other removals: Mississippi 100:29, 225:20, 900:34, 1000:38, 1100:31;
  Downtown 100:29, 200:14, 400:11, 500:40. These are explicit algorithm effects,
  not independently confirmed false positives. Retention is not accuracy.

The current cumulative rejection fixture is updated; original perception and
pole-recovery evidence stays intact. Tests exercise a clear shaft, a narrow
core surrounded by dense clutter, distant/unsupported-height returns, fitted
axis compensation, spatial transforms, point-order invariance, the reported
region, exact reference masks, feature selection and whole-pillar behavior.
All 98 affected tests pass, with zero failures and no filters. Factory Code
Analyzer reports zero findings across five changed/new MATLAB files.
Check exports are under `research/results/pole_isolation_20260910/`.

The native niri frame-91 display was refreshed with source label at 42593,
147 pole points, current camera unchanged, and original-point RGB coloring.
No full-route video/mapping rerun or new timing/accuracy qualification occurred.
Raw diagnostics and native figures remain under `output/pole_isolation_20260910/`.

## Subsequent threshold review

The [frame-687 review](fine_pole_isolation687.md) raises the production core
fraction from 75% to 80%. The measurements above describe the original
75% implementation; the follow-up records the incremental effects.
