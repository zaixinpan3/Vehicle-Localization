# Frame 851: why the LiDAR-only match is 68 cm off

Date: 2026-09-24. Data: the production chain of
[the output-point transport record](../output_point_transport_20260924/README.md)
(`output/mncav_coarse_localization_20260924`), recursive LiDAR-only matching
stage, code revision `26f6499`. Reference poses are used for evaluation and
for the diagnostic basin scans only.

## What the error looks like

The error is **longitudinal**: at frame 851 the matched pose is 68.4 cm ahead
of the reference along the driving direction and 3.7 cm to the side. It does
not appear suddenly. From frame 834 (9 cm) to 848 (66 cm) it grows by 2–7 cm
per frame, holds at 66–68 cm through frame 853 and collapses to 7 cm at
frame 854. At every frame the matched pose stays within a few centimetres of
its seed (frame 838: seed 22.5 cm, match 22.0; frame 846: 60.6 → 60.9), so the
matcher was following the seed instead of correcting it. Similarity fell from
0.58 to 0.34 but never approached the 0.15 acceptance threshold.

Seeding frame 851 at the reference pose converges to **3.6 cm** error with
similarity 0.53. The matcher can solve this scene; it was given a seed in the
wrong basin.

## Cause 1: the odometry seed ran ahead

The seed is the previous accepted match plus one frame of wheel/gyro/lateral
odometry. Over frames 826–856 the wheel-derived speed reads **0.19 m/s above
the reference speed** (8.3 vs 8.0 m/s at frame 841), against a whole-drive mean
of 0.047 m/s. Each frame the odometry step is 1–3 cm too long; the cumulative
excess over those 3 s is 59 cm, which is the error that appeared. The vehicle
is accelerating at about 0.46 m/s² there, but the drive-wide acceleration
dependence of the wheel error (0.033 s per m/s²) explains only 1.5 cm/s of it.
The rest is a local over-read of the wheels for about three seconds, cause not
established (surface, slip or the reference itself).

An odometry error of 2 cm per frame is normally harmless: elsewhere the map
pulls the match back every frame.

## Cause 2: the scene cannot fix the longitudinal position

Body-frame basin scans (seed offsets −1.5…+1.5 m along the road at every
frame, converged longitudinal error in cm):

| Frame | Converged error for seed offsets −150 … +150 cm (25 cm steps) |
|---|---|
| 834 | −55 −55 −55 −55 −55 **9 9 9 9 9 9 9 9** |
| 842 | −1 −1 −1 −1 −1 −1 −1 **37 37 37 37 37 37** |
| 846 | −8 −8 −8 −8 −8 −8 −8 −8 **61 61 61 61 61** |
| 851 | 3 3 3 3 3 3 3 5 **68 68 68 68 68** |
| 853 | 6 6 6 6 6 6 6 6 6 **62 62 62 62** |
| 854 | 6 6 6 6 6 6 6 6 6 6 6 6 6 |

Along the road the cost has **two attractors about 60–70 cm apart**. The
forward one is where the seed sat from frame 842 on, and it slides forward
with the seed (37 → 61 → 68 cm) as the robust weights move from the correct
map components to their neighbours. At frame 854 the basin becomes
single-valued and the error disappears.

Why two attractors: the only landmarks that constrain the longitudinal
position at these frames are behind the vehicle, at about −8 m (a pole and a
sign, 4 m to the right) and −2 m (a sign and a pole, 9.5 m to the left). The
curbs run parallel to the road and constrain only the lateral position and
yaw. In the map, each of those objects is not one Gaussian but several
spread along the road:

| Object | Map components (body-frame longitudinal offset from the first) |
|---|---|
| Sign at −8 m | 1264 (0), 1278 (+0.001, tile duplicate), 1263 (+0.94), 1277 (+0.94, tile duplicate) |
| Pole at −7 m | 1033 (0, 1.13 m longitudinal sd), 1051 (+0.06), 1052 (−1.21), 1053 (+0.54) |
| Sign at −2 m | 1279 (0), 1286 (0, tile duplicate), 1280 (+0.38), 1287 (+0.38, tile duplicate) |

Component IDs such as `trafficSign:60542:622078:2` and
`trafficSign:60543:622078:2` differ only in the tile field: the same object
was published by two adjacent map tiles at the same position with the weight
split between them. The other members are sub-components 0.4–1.2 m apart
along the road, from a pole footprint elongated by the coarse pillar
representation (1.13 m sd). A source pole shifted 0.5–0.9 m forward still
lands on a map component of the same class, which is the second attractor.
The next longitudinal landmarks ahead (signs at +21 m, poles at +24.6 m) take
over at frame 854.

`frame851_scene.png` shows the map components and the source clouds at the
two solutions in the body frame of the reference pose.

## Summary

1. A three-second wheel-speed over-read pushed the odometry seed forward by
   about 2 cm per frame.
2. In this stretch the map constrains the longitudinal position only through
   two objects behind the vehicle, each represented by several Gaussians
   0.4–1.2 m apart along the road plus exact duplicates from adjacent tiles.
   That makes the longitudinal cost bimodal with attractors 60–70 cm apart.
3. Once the seed crossed into the forward attractor the matcher followed it,
   the attractor drifted with the seed, and the acceptance test (similarity
   0.34 > 0.15) did not notice. GNSS aiding removes the error in the fused
   run by ranking the two attractors (the GNSS-aided match is 29 cm at frame 840 and
   under 16 cm from frame 842 on; the fused output stays under 14 cm).

Remedies, in order of expected value: merge tile-duplicate and split pole/sign
components at map publication; in LiDAR-only operation, solve from a second
seed displaced along the road when a class has several map components within
1.5 m of each other and keep the higher-similarity solution (0.53 vs 0.36
here); investigate the wheel over-read in frames 826–856.

## Reproduction

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/frame851_longitudinal_diagnosis_20260924'); diagnoseFrame851;
```

Requires `output/mncav_coarse_localization_20260924` (matching report, observer
experiment and `sources.mat`). MATLAB R2026a; deterministic.
