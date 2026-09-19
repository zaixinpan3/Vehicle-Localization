"""Audit baseline and weighted-NDT runs using identical frame populations."""
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
RUNS = {
    'geometric_baseline': ROOT / 'output/mississippi_matching_only_20260919',
    'log_mixture_candidate': ROOT / 'output/stability_ndt_20260919/full',
    'weighted_overlap_candidate': ROOT / 'output/stability_ndt_20260919/overlap_full',
}

def metrics(rows):
    errors = [float(r['positionErrorM']) for r in rows]
    return {
        'samples': len(rows),
        'position_rmse_m': math.sqrt(sum(e * e for e in errors) / len(errors)),
        'heading_rmse_deg': math.sqrt(sum(float(r['yawErrorDeg'])**2 for r in rows) / len(rows)),
        'position_max_m': max(errors),
    }


def main():
    runs = {}
    out = {'same_drive_map': True, 'reference': 'interpolated INSPVA, not independent ground truth', 'runs': {}}
    for method, folder in RUNS.items():
        out['runs'][method] = {}
        for mode in ['recursive', 'referenceSeed']:
            rows = list(csv.DictReader((folder / mode / 'calls.csv').open()))
            assert len(rows) == 1170
            assert [int(r['frame']) for r in rows] == list(range(1, 1171))
            assert not any(int(r['directionalAccepted']) for r in rows)
            for row in rows:
                position = math.hypot(float(row['x']) - float(row['referenceX']),
                                      float(row['y']) - float(row['referenceY']))
                yaw = float(row['psi']) - float(row['referencePsi'])
                yaw = math.degrees(math.atan2(math.sin(yaw), math.cos(yaw)))
                assert abs(position - float(row['positionErrorM'])) < 1e-8
                assert abs(yaw - float(row['yawErrorDeg'])) < 1e-9
                if int(row['accepted']):
                    a, b, c, d, e, f = [float(row[key]) for key in [
                        'informationXX', 'informationXY', 'informationXPsi',
                        'informationYY', 'informationYPsi', 'informationPsiPsi']]
                    assert a > 0 and a*d - b*b > 0
                    assert a*d*f + 2*b*c*e - a*e*e - d*c*c - f*b*b > 0
            runs[method, mode] = rows
            accepted = [r for r in rows if int(r['accepted'])]
            reasons = {}
            for row in rows:
                reasons[row['reason']] = reasons.get(row['reason'], 0) + 1
            out['runs'][method][mode] = {
                'accepted_measurements': metrics(accepted),
                'all_outputs_including_predictions': metrics(rows),
                'acceptance_fraction': len(accepted) / len(rows),
                'reasons': reasons,
            }
    out['common_accepted_frames'] = {}
    for mode in ['recursive', 'referenceSeed']:
        common = set.intersection(*[{int(r['frame']) for r in runs[method, mode] if int(r['accepted'])} for method in RUNS])
        out['common_accepted_frames'][mode] = {method: metrics([r for r in runs[method, mode] if int(r['frame']) in common]) for method in RUNS}
    (DEST / 'comparison.json').write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
