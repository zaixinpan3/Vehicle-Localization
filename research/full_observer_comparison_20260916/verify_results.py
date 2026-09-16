#!/usr/bin/env python3
"""Verify the same-frame raw-matching/observer comparison independently.

uv run --offline --with numpy --with h5py python research/full_observer_comparison_20260916/verify_results.py
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/full_observer_comparison_20260916'
DEST = Path(__file__).resolve().parent


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    frames = read(OUT / 'frame_errors.csv')
    metrics = read(OUT / 'paired_metrics.csv')
    difference = 0.
    for row in metrics:
        subset = [x for x in frames if x['method'] == row['method'] and x['accepted'] == '1']
        population = row['population']
        if population == 'accepted_before60': subset = [x for x in subset if float(x['time']) <= 60]
        if population == 'accepted_after60': subset = [x for x in subset if float(x['time']) > 60]
        if population == 'accepted_80_86': subset = [x for x in subset if 80 <= float(x['time']) < 86]
        e = np.array([float(x['positionErrorM']) for x in subset])
        yaw = np.array([float(x['headingErrorDeg']) for x in subset])
        assert len(e) == int(row['samples'])
        computed = dict(positionRmseM=np.sqrt(np.mean(e**2)), positionMedianM=np.median(e),
                        positionP95M=np.percentile(e, 95, method='hazen'), positionMaximumM=max(e),
                        fractionAtMost5cm=np.mean(e <= .05), fractionAtMost10cm=np.mean(e <= .1),
                        headingRmseDeg=np.sqrt(np.mean(yaw**2)), headingMaximumDeg=max(abs(yaw)))
        difference = max(difference, *(abs(value-float(row[key])) for key, value in computed.items()))
    assert difference < 1e-12
    with h5py.File(OUT / 'audit.mat') as a, h5py.File(ROOT / 'output/full_observer_20260916/experiment.mat') as s:
        uniform = a['uniform'][()].ravel().astype(int)-1
        native = a['native'][()].ravel().astype(int)-1
        ref = a['reference'][()].T
        current = s[s['runs'][0, 0]]['estimate']
        assert np.array_equal(a['full/z'][()].T[uniform], current['z'][()].T)
        assert np.array_equal(a['lidarMotion/z'][()].T[uniform], s[s['runs'][0, 1]]['estimate/z'][()].T)
        e = np.linalg.norm(a['full/pose'][()].T[native, :2]-ref[:, :2], axis=1)
        exported = [x for x in frames if x['method'] == 'full_observer']
        assert np.max(abs(e-np.array([float(x['positionErrorM']) for x in exported]))) < 1e-12
        full_error = np.linalg.norm(current['position'][()].T-s['reference'][()].T[:, :2], axis=1)
        mode = current['diagnostics/mode'][()].ravel()
        t = current['time'][()].ravel()
        segments = read(OUT / 'error_segments.csv')
        for row in segments:
            mask = {'all_uniform': np.ones(len(t), bool), 'both_channels': mode == 3,
                    'gnss_without_lidar': mode == 1, 'time_80_86': (t >= 80) & (t < 86),
                    'outside_80_86': (t < 80) | (t >= 86)}[row['population']]
            assert mask.sum() == int(row['samples'])
            assert abs(np.sqrt(np.mean(full_error[mask]**2))-float(row['positionRmseM'])) < 1e-12
            assert abs(np.sum(full_error[mask]**2)/np.sum(full_error**2)-float(row['fractionOfTotalSquaredPositionError'])) < 1e-12
        control = np.linalg.norm(a['headingControl/position'][()].T-s['reference'][()].T[:, :2], axis=1)
        control_rmse = np.sqrt(np.mean(control**2))
    report = json.loads((OUT / 'summary.json').read_text())
    assert abs(control_rmse-report['headingControl']['uniformMetrics'][0]) < 1e-12
    primary = {r['method']: r for r in metrics if r['population'] == 'same_accepted_frames'}
    gain = 1-float(primary['full_observer']['positionRmseM'])/float(primary['raw_lidar_matching']['positionRmseM'])
    check = {'pairedMetricRows': len(metrics), 'sameFrameSamples': 1083, 'maximumMetricDifference': difference,
             'uniformStatesBitwiseUnchanged': True, 'exactNativeObserverOutputsVerified': True,
             'fullObserverRmseReductionVersusRawLidar': gain, 'diagnosticHeadingControlRmseM': control_rmse,
             'scope': 'Paired error audit; no estimator/gain change or independent ground-truth claim.'}
    (DEST / 'independent_checks.json').write_text(json.dumps(check, indent=2)+'\n')
    for name in ['paired_metrics.csv', 'error_segments.csv', 'summary.json']:
        shutil.copy2(OUT / name, DEST / name)
    paths = [p for p in OUT.iterdir() if p.is_file()]
    paths += [ROOT / p for p in ['output/full_observer_20260916/experiment.mat',
              'output/saved_perception_inspva_20260915/calls.csv',
              'output/saved_perception_inspva_20260915/experiment.mat',
              'output/mncav_interface_audit_20260916/correction_experiment.mat',
              'scripts/auditFullObserverLidarComparison.m']]
    manifest = []
    for path in sorted(paths):
        with path.open('rb') as stream: digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        manifest.append({'path': str(path.relative_to(ROOT)), 'bytes': path.stat().st_size, 'sha256': digest})
    (DEST / 'artifact_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(check, indent=2))


if __name__ == '__main__':
    main()
