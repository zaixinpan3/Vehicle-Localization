#!/usr/bin/env python3
"""Independently recompute metrics, availability and certificate eigenvalues.

uv run --offline --with numpy --with h5py python research/full_observer_20260916/verify_results.py
This checks saved evidence; it does not claim an independent estimator or truth.
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
OUT = ROOT / 'output/full_observer_20260916'


def array(group, name):
    return np.asarray(group[name]).T


def active(source, query, valid):
    time = array(source, 'time').ravel()
    ix = np.searchsorted(time, query, side='right') - 1
    return (ix >= 0) & valid[np.maximum(ix, 0)] & (query < time[np.maximum(ix, 0)] + .2)


def main():
    with (OUT / 'metrics.csv').open() as stream:
        metrics = list(csv.DictReader(stream))
    modes = ['both', 'lidar_only', 'gnss_only', 'gnss_outage', 'lidar_outage', 'both_outage', 'alternating']
    maximum = 0.
    with h5py.File(OUT / 'experiment.mat') as saved:
        truth = array(saved, 'reference')
        t = array(saved['data/highRate'], 'time').ravel()
        gnss = saved['data/gnss']; lidar = saved['data/lidar']
        gt = array(gnss, 'time').ravel(); lt = array(lidar, 'time').ravel()
        gv = array(gnss, 'valid').ravel().astype(bool); lv = array(lidar, 'valid').ravel().astype(bool)
        source_counts = {'gnss': len(gt), 'lidar': len(lt), 'lidar_full': int(lv.sum())}
        for j, mode in enumerate(modes):
            run = saved[saved['runs'][0, j]]['estimate']
            pose = array(run, 'pose')
            assert pose.shape == truth.shape and np.isfinite(pose).all()
            assert np.array_equal(array(run, 'z'), array(run, 'onlineZ'))
            assert np.array_equal(array(run, 'time').ravel(), t)
            error = np.linalg.norm(pose[:, :2] - truth[:, :2], axis=1)
            yaw = np.angle(np.exp(1j * (pose[:, 2] - truth[:, 2])))
            for row in [r for r in metrics if r['scenario'] == mode]:
                mask = {'full': np.ones(len(t), bool), 'outage_40_60': (t >= 40) & (t < 60),
                        'recovery_60_70': (t >= 60) & (t < 70)}[row['population']]
                e = error[mask]
                computed = {'positionRmseM': np.sqrt(np.mean(e**2)), 'positionMedianM': np.median(e),
                            'positionMaximumM': e.max(), 'positionP95M': np.percentile(e, 95, method='hazen'),
                            'fractionAtMost10cm': np.mean(e <= .1),
                            'headingRmseDeg': np.rad2deg(np.sqrt(np.mean(yaw[mask]**2)))}
                assert int(row['samples']) == mask.sum()
                maximum = max(maximum, *(abs(v-float(row[k])) for k, v in computed.items()))
            current_g = gv.copy(); current_l = lv.copy()
            if mode in ['gnss_outage', 'both_outage']: current_g[(gt >= 40) & (gt < 60)] = False
            if mode in ['lidar_outage', 'both_outage']: current_l[(lt >= 40) & (lt < 60)] = False
            if mode == 'alternating':
                current_g &= np.floor(gt) % 2 == 0
                current_l &= np.floor(lt) % 2 == 1
            ga = active(gnss, t, current_g) if mode != 'lidar_only' else np.zeros(len(t), bool)
            la = active(lidar, t, current_l) if mode != 'gnss_only' else np.zeros(len(t), bool)
            expected = ga.astype(int) + 2*la.astype(int)
            actual = array(run['diagnostics'], 'mode').ravel()
            assert np.array_equal(actual, expected), mode
            correction = array(run['diagnostics'], 'positionCorrection')
            assert np.all(correction[~ga, :2] == 0) and np.all(correction[~la, 2:] == 0)
            age = array(run['diagnostics'], 'sourceAge')
            assert np.all(age[ga, 0] >= 0) and np.all(age[ga, 0] < .2)
            assert np.all(age[la, 1] >= 0) and np.all(age[la, 1] < .2)
        baseline = array(saved, 'baselinePose')
        baseline_rmse = np.sqrt(np.mean(np.sum((baseline[:, :2]-truth[:, :2])**2, axis=1)))
    assert maximum < 1e-10
    # Native LiDAR poses are untouched; no resampling is hidden in source arrays.
    with (ROOT / 'output/saved_perception_inspva_20260915/calls.csv').open() as stream:
        original = [r for r in csv.DictReader(stream) if r['mode'] == 'per_frame_zero' and float(r['time']) <= t[-1]]
    original_pose = np.array([[float(r[k]) for k in ['measurementX', 'measurementY', 'measurementPsi']] for r in original])
    with h5py.File(OUT / 'experiment.mat') as saved:
        stored_pose = array(saved['data/lidar'], 'pose')
    assert np.array_equal(stored_pose, original_pose, equal_nan=True)
    summary = json.loads((OUT / 'summary.json').read_text())
    margins = []
    for alpha in [.25, 1., 1.25]:
        margins.append(min(np.linalg.eigvalsh([[2*alpha, -1, 0], [-1, 8, -(1+q2)],
                                              [0, -(1+q2), 24]])[0] for q2 in [0, .16]))
    assert np.max(np.abs(np.array(margins)-summary['design']['translationMargins'])) < 1e-12
    check = {'metricRowsChecked': len(metrics), 'maximumMetricDifference': maximum,
             'availabilitySamplesChecked': len(t)*len(modes), 'invalidChannelCorrectionExactlyZero': True,
             'nativeLidarPacketsUnchanged': True, 'sourceCounts': source_counts,
             'independentTranslationMargins': margins, 'legacyOfflinePositionRmseM': baseline_rmse,
             'scope': 'Independent aggregation and matrix verification, not independent ground truth or a new field experiment.'}
    (DEST / 'independent_checks.json').write_text(json.dumps(check, indent=2)+'\n')
    for name in ['metrics.csv', 'summary.json', 'validation.json', 'tests.csv', 'code_analyzer.csv']:
        shutil.copy2(OUT / name, DEST / name)
    paths = [p for p in OUT.iterdir() if p.is_file()]
    paths += [ROOT / p for p in ['output/saved_perception_inspva_20260915/calls.csv',
              'output/mncav_interface_audit_20260916/vehicle_parameters.json',
              'output/mncav_inspva_observer_20260915/native_reference.csv',
              'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_odom.csv',
              'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv']]
    manifest = []
    for p in sorted(paths):
        with p.open('rb') as stream: digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        manifest.append({'path': str(p.relative_to(ROOT)), 'bytes': p.stat().st_size, 'sha256': digest})
    (DEST / 'artifact_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(check, indent=2))


if __name__ == '__main__':
    main()
