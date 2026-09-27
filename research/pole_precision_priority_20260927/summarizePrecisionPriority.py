"""Check completed raw replays and retain exact precision/coverage counts."""
from pathlib import Path
import csv
import json
import subprocess

from selectPrecisionThreshold import LIMITS

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read_csv(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def aggregate(rows):
    selected = sum(int(r['candidatePillarCount']) for r in rows)
    false = sum(int(r['extraPillarCount']) for r in rows)
    covered = sum(int(r['coveredFinePointCount']) for r in rows)
    reference = sum(int(r['finePointCountInRoi']) for r in rows)
    return dict(frames=len(rows), selected=selected, false=false,
                falseFraction=false / selected if selected else None,
                coveredPoints=covered, referencePoints=reference,
                pointCoverage=covered / reference, missedPointFraction=1 - covered / reference,
                framesWithAnySelection=sum(int(r['candidatePillarCount']) > 0 for r in rows),
                referenceFramesWithNoSelection=sum(int(r['finePointCountInRoi']) > 0
                    and int(r['candidatePillarCount']) == 0 for r in rows))


def main():
    freeze = json.loads((HERE / 'threshold_freeze.json').read_text())
    threshold = freeze['choices']['cap05']['threshold']
    predicted = json.loads((HERE / 'threshold_validation.json').read_text())['cap05']
    previous = json.loads((ROOT / 'research/pole_geometry_20260927/summary.json').read_text())
    replay = {}
    for dataset, limit in LIMITS.items():
        rows = read_csv(HERE / f'{dataset.lower()}_replay.csv')
        old = read_csv(HERE / f'{dataset.lower()}_previous_replay.csv')
        assert len(rows) == (1170 if dataset == 'Mississippi' else 128)
        assert all(r['nonPoleComponentsUnchanged'] == '1' and r['sameFrozenSelections'] == '1' for r in rows)
        assert [r['frame'] for r in rows] == [r['frame'] for r in old]
        replay[dataset] = {}
        for part in ['prefix', 'suffix', 'all']:
            def eligible(row):
                return part == 'all' or (int(row['frame']) <= limit) == (part == 'prefix')
            current = aggregate([r for r in rows if eligible(r)])
            baseline = aggregate([r for r in old if eligible(r)])
            for key in ['selected', 'false', 'coveredPoints', 'referencePoints']:
                assert current[key] == predicted[part][dataset][key], (dataset, part, key)
            assert current['referencePoints'] == baseline['referencePoints']
            replay[dataset][part] = dict(previous=baseline, precisionFirst=current)
        assert replay[dataset]['all']['previous']['false'] == previous['actual']['all'][dataset]['extra']
        assert replay[dataset]['all']['previous']['coveredPoints'] == previous['actual']['all'][dataset]['coveredPoints']
    scores = read_csv(HERE / 'prefix_owner_scores.csv')
    fold_rows = []
    for dataset, limit in LIMITS.items():
        denominators = read_csv(ROOT / 'research/pole_precision_20260927' / f'{dataset.lower()}_frames.csv')
        for fold in range(5):
            selected = [r for r in scores if r['dataset'] == dataset
                        and (int(r['frame']) - 1) * 5 // limit == fold and float(r['score']) >= threshold]
            total = sum(int(r['finePoints']) for r in denominators if int(r['frame']) <= limit
                        and (int(r['frame']) - 1) * 5 // limit == fold)
            false = sum(int(r['finePointCount']) == 0 for r in selected)
            covered = sum(int(r['finePointCount']) for r in selected)
            fold_rows.append(dict(dataset=dataset, fold=fold, selected=len(selected), false=false,
                                  falseFraction=false / len(selected) if selected else None,
                                  coveredPoints=covered, referencePoints=total, pointCoverage=covered / total))
    with (HERE / 'prefix_fold_metrics.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fold_rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(fold_rows)
    tests = read_csv(HERE / 'tests.csv')
    assert len(tests) == 210 and all(r['Passed'] == '1' and r['Failed'] == '0' and r['Incomplete'] == '0' for r in tests)
    model_path = ROOT / 'config/polePillarDistributionModel.json'
    model = json.loads(model_path.read_text())
    old_model = json.loads(subprocess.check_output(['git', 'show',
        '6df904cfed15c72e09cabb3a39c2c620522faae1:config/polePillarDistributionModel.json'], cwd=ROOT, text=True))
    assert model['decisionThreshold'] == threshold
    for key in model:
        if key != 'decisionThreshold':
            assert model[key] == old_model[key]
    result = dict(previousCommit='6df904cfed15c72e09cabb3a39c2c620522faae1',
                  oldThreshold=old_model['decisionThreshold'], threshold=threshold,
                  metric='False selected pillar means zero original fine points; FP/selected. '
                         'Coverage uses all original fine points in the unchanged coarse ROI.',
                  prefixOOF=freeze['choices']['cap05']['metrics'], replay=replay,
                  validation=dict(passed=len(tests), failed=0, incomplete=0, rawFrames=1298,
                                  frozenSelectionsMatch=True, scoringModelUnchanged=True,
                                  originalReferenceDenominatorsPreserved=True),
                  allDatasetAndSuffixFalseFractionsBelow10Percent=all(
                      replay[d][p]['precisionFirst']['falseFraction'] <= .10 for d in LIMITS for p in ['all', 'suffix']),
                  limitations='Coverage below 80% is explicitly authorized. Full replay includes fitted prefixes. '
                              'Suffixes are reused and informed promotion. The <=10% result is aggregate on these '
                              'recordings, not per-frame, per-block or future-scene assurance; some prefix folds exceed 10%. '
                              'No new runtime benchmark or localization-accuracy claim.')
    (HERE / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
