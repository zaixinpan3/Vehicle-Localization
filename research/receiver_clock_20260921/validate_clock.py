"""Validate the shared clock against recorded timestamps and frozen poses.

uv run --offline --with numpy python research/receiver_clock_20260921/validate_clock.py
The frozen-match rescore is a diagnostic, NOT a rematched localization result.
"""
from pathlib import Path
import csv
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from receiverClock import ensure_clock, convert_time, native_seconds, fit_clock


def main():
    folder = ROOT / 'data/raw/Missisipi/gnss'
    stem = 'raw_data_2024-06-07-12-09-31_0'
    ins = np.genfromtxt(folder / (stem + '_inspva.csv'), delimiter=',', names=True)
    lidar = np.genfromtxt(folder / (stem + '_front_lidar_points.csv'), delimiter=',', names=True)
    clock = ensure_clock(folder / (stem + '_inspva.csv'))
    native = native_seconds(ins['gps_week'], ins['gps_seconds'])
    source = ins['stamp_sec'] - ins['stamp_sec'][0]
    query = lidar['stamp_sec'] - ins['stamp_sec'][0]
    old = np.interp(query, source, native)
    new = convert_time(clock, lidar['stamp_sec'])
    # Deliberately hold out contiguous temporal blocks, not random samples.
    # These controls measure fit stability, not absolute sensor-clock accuracy.
    blocks = np.array_split(np.arange(len(native)), 10)
    deviations = []
    for indices in blocks:
        keep = np.ones(len(native), dtype=bool); keep[indices] = False
        held = fit_clock(ins['stamp_sec'][keep], native[keep])
        prediction = held['scale'] * (lidar['stamp_sec'] - held['sourceOriginSeconds']) + held['offsetSeconds']
        deviations.append(float(np.max(np.abs(prediction - new))))
    with (ROOT / 'output/saved_perception_inspva_20260915/calls.csv').open() as stream:
        calls = [r for r in csv.DictReader(stream) if r['mode'] == 'per_frame_zero' and float(r['fullPose']) == 1]
    poses = np.genfromtxt(folder / (stem + '_front_lidar_synchronized_pose_1_1170.csv'), delimiter=',', names=True,
                         dtype=None, encoding='utf-8')
    index = np.array([int(r['frame'])-1 for r in calls])
    estimate = np.array([[float(r['outputX']), float(r['outputY'])] for r in calls])
    reference = np.c_[poses['pose_x_m'], poses['pose_y_m']]
    corrected_error = np.linalg.norm(estimate-reference[index], axis=1)
    original_error = np.array([float(r['mappingErrorM']) for r in calls])
    frame307 = int(np.flatnonzero(index == 306)[0])
    summary = dict(clock=clock, frames=len(new),
        timestampChangeSeconds=dict(median=float(np.median(new-old)), maximum=float(np.max(new-old)),
                                    minimum=float(np.min(new-old)), frame307=float(new[306]-old[306])),
        contiguousTenFoldMaximumClockPredictionDifferenceSeconds=max(deviations),
        frozenMatchingDiagnostic=dict(samples=len(calls), rematched=False, mapRebuilt=False,
            beforeRmseM=float(np.sqrt(np.mean(original_error**2))), afterRescoreRmseM=float(np.sqrt(np.mean(corrected_error**2))),
            frame307BeforeM=float(original_error[frame307]), frame307AfterRescoreM=float(corrected_error[frame307]),
            limitation='Unchanged old-map matching poses scored at corrected epochs; not a coherent new-pipeline accuracy result'),
        timeAccuracyLimitation='Inlier residual and holdout fit agreement do not calibrate constant publication latency, LiDAR scan epoch or per-point deskew')
    out = ROOT / 'research/receiver_clock_20260921'
    (out / 'clock_validation.json').write_text(json.dumps(summary, indent=2)+'\n')
    np.savetxt(out / 'frame_times.csv', np.c_[np.arange(1,len(new)+1),old,new,new-old],delimiter=',',
               header='frame,oldReceiverSeconds,newReceiverSeconds,changeSeconds',comments='',fmt=['%d','%.12f','%.12f','%.12f'])
    np.savetxt(out / 'frozen_match_rescore.csv',np.c_[index+1,original_error,corrected_error],delimiter=',',
               header='frame,originalErrorM,correctedEpochErrorM',comments='',fmt=['%d','%.12f','%.12f'])
    rematched = out / 'rematched_fine_calls.csv'
    if rematched.exists():
        fresh = np.genfromtxt(rematched, delimiter=',', names=True)
        baseline = dict(zip(index + 1, original_error))
        common = [r for r in fresh if r['accepted'] == 1 and int(r['frame']) in baseline]
        paired = dict(commonAccepted=len(common),
            beforeRmseM=float(np.sqrt(np.mean([baseline[int(r['frame'])]**2 for r in common]))),
            afterRmseM=float(np.sqrt(np.mean([r['positionErrorM']**2 for r in common]))))
        (out / 'paired_fine_comparison.json').write_text(json.dumps(paired, indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='clock'},indent=2))


if __name__ == '__main__':
    main()
