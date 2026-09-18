# Coarse and fine perception timing

Measured September 18, 2026 on an AMD Ryzen 7 7800X3D (8 cores / 16 logical
CPUs), MATLAB 26.1.0.3276743 (R2026a) Update 3, native perception kernels, source commit
`4fa219e952c8e918ac020dacc2d4e3d79ad01d54`. No perception algorithm or parameter was modified for this task.

## Method

Frames: 28, 91, 214, 260, 384, 615, 746, 901, 1047, 1137, 291 from Mississippi. All frames have
65,536 input points. The current configuration selects curb, pole and trafficSign;
334 by 334 whole XY pillars at 0.3 m cover [-50,50.2) m. The fine mode adds
point-level refinement to common preprocessing and coarse processing.

Each frame and mode has one untimed warm-up followed by seven timed calls.
Execution order alternates coarse/fine and fine/coarse. Each repetition must
reproduce its warm-up feature mask or coarse probability cloud. Frame loading,
cache loading, pcshow/GUI rendering, mapping and localization are outside the
timed regions. All repeat measurements are in `summary.json`; `timing.csv`
contains the per-frame medians. The headline is the median of the 11 frame
medians, and the range is the minimum/maximum of those medians, not an estimate
of full-sequence worst-case latency.

## Results

| Pipeline mode | Median across frame medians | Range of frame medians | Reciprocal median rate |
| --- | ---: | ---: | ---: |
| Coarse | 64.59 ms | 51.68 to 68.18 ms | 15.48 frames/s |
| Complete offline fine | 2012.53 ms | 1037.14 to 4015.64 ms | 0.50 frames/s |

The displayed frame 291 measures 55.76 ms
coarse and 2012.53 ms complete fine. Its 157
pole points match the preceding native desktop preview. The fine number already
includes the common/coarse work; do not add the coarse number again. These are
all-channel pipeline timings, not pole-only timings.

The coarse sample medians fit a 100 ms compute budget. This is not an end-to-end
10 Hz deadline guarantee: loading, other localization modules, operating-system
scheduling and unmeasured frames remain outside the claim. Fine is an offline
path on this hardware. No precision/recall claim follows from timing.

## Artifacts and reproduction

The executed script is `output/perception_timing_20260918/benchmark.m` and its
complete result is `output/perception_timing_20260918/timing.mat`. Ten raw input
frames were read from the prior diagnostic cache at
`output/pole_consistency_20260918/height_diagnosis.mat`; frame 291 was loaded with
`loadPointCloudFrame` from `data/raw/MissisipiPointClouds.mat`. Input preparation
is untimed. Run using `matlab -batch` with the benchmark script. Raw data and
cache MAT files are not added to the public repository. The archive preserves
the exact executed script and the resulting commit record.
