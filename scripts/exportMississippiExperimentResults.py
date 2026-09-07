#!/usr/bin/env python3
"""Export compact experiment metrics; retain raw poses and large MAT files locally."""
import argparse
import csv
import json
import math
import statistics
import shutil
from pathlib import Path


def read_csv(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows, fields):
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore', lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('output/mississippi_20240607_120931_20260907'))
    parser.add_argument('--output', type=Path, default=Path('research/results/mississippi_full_sequence_20260907'))
    args = parser.parse_args()
    source, output = args.source, args.output
    output.mkdir(parents=True, exist_ok=True)
    load = lambda path: json.loads(path.read_text())
    summary = {'map': load(source/'map_summary.json'),
               'd2d_8threads': load(source/'recursive_8threads/summary.json'),
               'd2d_1thread_diagnostic': load(source/'recursive/summary.json'),
               'scope': 'Same-drive map consistency; fixed-delay feed-forward replay; causal onlineZ; unidentified nominal dynamics.'}
    calls = read_csv(source/'recursive_8threads/calls.csv')
    one_thread = read_csv(source/'recursive/calls.csv')
    assert len(calls) == len(one_thread) == 1170
    summary['thread_replay_comparison'] = {
        'different_acceptance_count': sum(a['accepted'] != b['accepted'] for a, b in zip(calls, one_thread)),
        'maximum_position_error_difference_m': max(abs(float(a['positionErrorM'])-float(b['positionErrorM'])) for a, b in zip(calls, one_thread)),
        'maximum_yaw_error_difference_deg': max(abs(float(a['yawErrorDeg'])-float(b['yawErrorDeg'])) for a, b in zip(calls, one_thread))}
    fields = ['frame', 'timeSeconds', 'accepted', 'reason', 'positionErrorM', 'yawErrorDeg',
              'similarity', 'rank', 'iterations', 'matches', 'totalMs', 'perceptionMs',
              'registrationMs', 'mapSelectionMs', 'diskLoadMs', 'informationXX',
              'informationXY', 'informationXPsi', 'informationYY', 'informationYPsi', 'informationPsiPsi']
    write_csv(output/'d2d_calls.csv', calls, fields)
    write_csv(output/'one_thread_diagnostic.csv', one_thread, fields)
    summary['observers'] = {}
    observer_rows = []
    for name in ['gpsOnly', 'fusion', 'positionOutage', 'outageNoLidar', 'fusion_0.7', 'fusion_1.3', 'fusion_delay0.30']:
        folder = source/f'observer_{name}'
        assert (folder/'summary.json').is_file(), f'Missing observer experiment: {name}'
        summary['observers'][name] = load(folder/'summary.json')
        # All original 100 Hz outputs remain in the local artifact. Public
        # error traces use 10 Hz; summary extrema/RMSE still use every sample.
        for index, row in enumerate(read_csv(folder/'online.csv')):
            if index % 10 == 0:
                observer_rows.append(dict(scenario=name, **row))
        if (folder/'failure.json').exists():
            failure = load(folder/'failure.json')
            summary['observers'][name]['failure_diagnostic'] = failure
    write_csv(output/'observer_error_traces_10hz.csv', observer_rows,
              ['scenario', 'time', 'positionErrorM', 'yawErrorDeg', 'lateralVelocityMps', 'sideSlipAngleRad'])
    tests = {row['Name']: row for row in read_csv(source/'tests.csv')}
    tests.update({row['Name']: row for row in read_csv(source/'required_tests.csv')})
    assert all(row['Passed'] == '1' and row['Incomplete'] == '0' for row in tests.values())
    summary['validation'] = {'unique_tests': len(tests), 'passed': len(tests), 'failed': 0, 'incomplete': 0}
    reference_rows = read_csv(source/'observer_gpsOnly/online.csv')
    diagnostic = []
    for shift in range(5):
        errors = [math.hypot(float(reference_rows[i]['x'])-float(reference_rows[i-shift]['referenceX']),
                             float(reference_rows[i]['y'])-float(reference_rows[i-shift]['referenceY']))
                  for i in range(4, len(reference_rows))]
        diagnostic.append({'reference_shift_seconds': shift*.01, 'median_m': statistics.median(errors),
                           'rmse_m': math.sqrt(sum(e*e for e in errors)/len(errors))})
    summary['reference_shift_diagnostic'] = {
        'scope': 'Post hoc timing diagnostic only. No shift enters any primary result or online algorithm.',
        'values': diagnostic}
    write_csv(output/'tests.csv', tests.values(), ['Name', 'Passed', 'Failed', 'Incomplete', 'Duration'])
    for name in ['map_summary.json', 'map_layers.csv', 'vehicle_parameters.json', 'design_certificates.json', 'outage_certificate_audit.csv']:
        shutil.copyfile(source/name, output/name)
    shutil.copyfile(source/'recursive_8threads/metadata.json', output/'d2d_metadata.json')
    shutil.copyfile(source/'recursive_8threads/disk_blocks.csv', output/'disk_blocks.csv')
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
