# Mississippi fine-perception video regeneration, 2026-09-12

All 1,170 frames of `data/raw/MissisipiPointClouds.mat` were processed with
fine perception from commit `63eecd077ab8c954c0aab5aa546e6e3a0d1fa23f`.
This includes the frame-458 curb continuation correction and all earlier
accepted changes. The saved frame-458 camera was copied into the isolated
recording display. All ten saved camera and axis properties match the run
configuration exactly and were checked on every rendered frame.

The displayed classes are curb, pole and traffic sign, colored directly on
original points at the same marker size. Road markings are disabled. Each
image shows its frame index out of 1,170. The recording uses complete RGB
image exports because software-interactive rendering sampled markers even
though the scatter retained all source coordinates. The final two-frame smoke
export and samples at frames 1, 100, 500 and 1,170 were visually inspected.
Earlier smoke attempts and diagnostic logs remain in the output directory.

## Recorded result

- Source and recorded indices: 1 through 1,170, inclusive.
- Dimensions: 1,910 by 2,086 pixels.
- Cadence: 10.909039310639 frames/s, from matched LiDAR timestamps.
- Playback duration: 107.250507 seconds.
- Processing and recording elapsed time: 4066.226072 seconds.
  This is machine time, not user work hours or a real-time performance claim.
- Aggregate observations: 133,836 curb,
  194,300 pole and 70,950 traffic-sign points.

The count CSV has exactly the continuous sequence 1:1170. Its sums match the
MATLAB result. Recorded curb counts at frames 458, 943 and 600 are 183, 48 and
27 respectively, matching the accepted current results. The code was checked
by 122 passing regression tests and zero Code Analyzer findings across four
changed MATLAB files before recording completion.

## Original artifacts

`output/mississippi_perception_video_20260912/` contains the AVI master and
MP4 export; captured and calibrated figure/configuration files; exact frame
counts and timestamps; saved feature observations registered to matched poses;
run result, progress, validation and integrity JSON; and periodic PNG samples.
The previous video remains unchanged in its dated directory.

No map was fitted during this video regeneration (`cfg.buildMap=false`).
The saved observations are reusable mapping inputs. Frame-count, camera and
decoding checks establish recording integrity, not semantic ground truth for
all frames. Large generated videos, raw data and observation binaries remain
outside the source-code commit.

## Export integrity

Both the AVI master and MP4 export contain exactly 1,170 decoded frames at
1,910 by 2,086 pixels and fully decode without errors. The MP4 uses libx264,
CRF 14, medium preset, yuv420p, faststart and passthrough cadence.

- `perception.avi`: 1,168,920,704 bytes; SHA-256
  `f35335c7cdce72bacc72b287db13b3b49c9032ba5c5bf682667482a13e7bf916`.
- `perception.mp4`: 464,794,000 bytes; SHA-256
  `66540df050a43eb84eeed9aec445476d009abb11cbd9ea5daae5eb5771cfaafa`.
