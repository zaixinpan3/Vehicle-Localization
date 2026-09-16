#!/usr/bin/env python3
"""Independently aggregate exported errors and identify operational artifacts.

uv run --offline --with numpy python research/mncav_interface_audit_20260916/verify_results.py
This script never reads or hashes weekly/monthly archive reports.
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/mncav_interface_audit_20260916'
DEST = Path(__file__).resolve().parent


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    metrics = read(OUT / 'correction_metrics.csv')
    frames = read(OUT / 'correction_frame_errors.csv')
    old_frames = read(ROOT / 'output/mncav_inspva_observer_20260915/frame_errors.csv')
    difference = 0.; checked = 0
    for row in metrics:
        if row['population'] not in ['native_full', 'native_all']:
            continue
        full = row['population'] == 'native_full'
        if row['method'] == 'lidar_measurement':
            selected = [f for f in old_frames if f['mode'] == row['mode'] and f['fullMeasurement'] == '1']
            values = np.array([float(f['lidarMeasurementErrorM']) for f in selected])
        else:
            selected = [f for f in frames if f['mode'] == row['mode'] and f['variant'] == row['method']
                        and (not full or f['fullMeasurement'] == '1')]
            values = np.array([float(f['positionErrorM']) for f in selected])
        assert len(values) == int(row['samples'])
        computed = {'positionRmseM': np.sqrt(np.mean(values**2)), 'positionMedianM': np.median(values),
                    'positionMaximumM': np.max(values), 'fractionAtMost5cm': np.mean(values <= .05),
                    'fractionAtMost10cm': np.mean(values <= .1)}
        for key, value in computed.items():
            difference = max(difference, abs(value-float(row[key])))
        checked += 1
    assert difference < 1e-12
    lateral = np.genfromtxt(OUT / 'lateral_comparison.csv', delimiter=',', names=True)
    audit = np.genfromtxt(ROOT / 'output/mncav_inspva_median_20260915/motion_audit.csv', delimiter=',', names=True)
    moving = np.interp(lateral['time'], audit['time'], audit['measuredVx']) >= 5
    straight = moving & (np.abs(np.interp(lateral['time'], audit['time'], audit['measuredYawRate'])) < .03)
    masks = {'moving': moving, 'straight': straight, 'moving_first60': moving & (lateral['time'] <= 60),
             'moving_after60': moving & (lateral['time'] > 60)}
    fields = {'previous_observer': 'previousVy', 'steering_only': 'steeringVy',
              'lidar_bias_legacy_steering': 'biasLegacyVy', 'steering_and_lidar_bias': 'biasSteeringVy'}
    lateral_difference = 0.
    for row in read(OUT / 'lateral_metrics.csv'):
        if row['mode'] != 'per_frame_zero':
            continue
        mask = masks[row['population']]
        e = (lateral[fields[row['variant']]]-lateral['referenceVy'])[mask]
        assert len(e) == int(row['samples'])
        for key, value in {'rmseMps': np.sqrt(np.mean(e*e)), 'biasMps': np.mean(e), 'medianAbsMps': np.median(np.abs(e))}.items():
            lateral_difference = max(lateral_difference, abs(value-float(row[key])))
    assert lateral_difference < 1e-12
    for mode in ['per_frame_zero', 'per_frame_positive', 'per_frame_negative']:
        subset = {r['method']: r for r in metrics if r['mode'] == mode and r['population'] == 'native_full'}
        for key in ['positionRmseM', 'positionMedianM']:
            assert float(subset['steering_and_lidar_bias'][key]) < float(subset['lidar_measurement'][key])
            assert float(subset['steering_and_lidar_bias'][key]) < float(subset['previous_observer'][key])
    summary = json.loads((OUT / 'correction_summary.json').read_text())
    (DEST / 'experiment_audit.json').write_text(json.dumps({k: summary[k] for k in ['metadata','checks']}, indent=2)+'\n')
    exports = ['interface_audit.json', 'correction_metrics.csv', 'lateral_metrics.csv', 'tests.csv',
               'code_analyzer.csv', 'correction_diagnostics.json']
    for name in exports:
        shutil.copy2(OUT / name, DEST / name)
    checks = {'independentNativeMetricRows': checked, 'maximumPositionMetricDifference': difference,
              'maximumLateralMetricDifference': lateral_difference,
              'bothPrimaryMetricsImproveInAllThreeMatchingModes': True,
              'scope': 'Independent aggregation of exported errors, not independent ground truth or a statistical significance test.'}
    (DEST / 'independent_checks.json').write_text(json.dumps(checks, indent=2)+'\n')
    paths = [ROOT / 'output/mncav_inspva_observer_20260915/experiment.mat',
             ROOT / 'output/mncav_inspva_observer_20260915/motion_gap_experiment.mat',
             ROOT / 'output/mncav_inspva_median_20260915/motion_audit.csv',
             ROOT / 'output/saved_perception_inspva_20260915/calls.csv']
    for seq in ['12-09-31', '12-11-24']:
        paths.append(ROOT / f'data/raw/Missisipi/raw_data_2024-06-07-{seq}_0.bag')
    base = ROOT / 'output/mississippi_20240607_120931_20260907'
    for folder in ['sensors', 'calibration_sensors']:
        paths += sorted((base / folder).glob('*.csv'))
    paths += [ROOT / 'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv',
              base / 'vehicle_parameters.json', ROOT / 'config/mncavVehicleParameters.json',
              ROOT / 'config/mncavReplayInterface.json']
    paths += [p for p in OUT.rglob('*') if p.is_file()]
    manifest = []
    for path in sorted(paths):
        with path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        manifest.append({'path': str(path.relative_to(ROOT)), 'bytes': path.stat().st_size, 'sha256': digest})
    (DEST / 'artifact_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(checks, indent=2))


if __name__ == '__main__':
    main()
