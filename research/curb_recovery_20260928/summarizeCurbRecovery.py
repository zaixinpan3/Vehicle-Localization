"""Summarize raw replay, paired timings, tests, and independent artifact hashes."""
from pathlib import Path
import hashlib
import json
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
P = Path(__file__).resolve().parent
before = pd.read_csv(ROOT / 'research/semantic_precision_20260927/mississippi_replay.csv')
after = pd.read_csv(P / 'replay.csv')
assert after.frame.nunique() == 1170 and len(after) == 3510
results = {}
for split, frames in [('all', range(1, 1171)), ('prefix', range(1, 781)),
                      ('suffix', range(781, 1171)), ('frame500', [500])]:
    results[split] = {}
    for feature in ['curb', 'pole', 'trafficSign']:
        results[split][feature] = {}
        for label, table in [('before', before), ('after', after)]:
            t = table[table.frame.isin(frames) & (table.feature == feature)]
            n = int(t.candidatePillarCount.sum())
            false = int(t.extraPillarCount.sum())
            covered = int(t.coveredFinePointCount.sum())
            total = int(t.finePointCountInRoi.sum())
            results[split][feature][label] = dict(
                selected=n, false=false, falseFraction=false/n if n else None,
                coveredPoints=covered, referencePoints=total, pointCoverage=covered/total)
        if feature != 'curb':
            assert results[split][feature]['before'] == results[split][feature]['after']
forest = json.loads((P / 'forest_validation.json').read_text())
for split in results:
    assert results[split]['curb']['after'] == forest['validation'][split]
runtime = pd.read_csv(P / 'runtime.csv')
paired = runtime.pivot(index=['frame', 'repeat'], columns='mode', values='milliseconds')
timing = {name: dict(samples=len(t), medianMilliseconds=float(t.milliseconds.median()),
                    meanMilliseconds=float(t.milliseconds.mean()),
                    p95Milliseconds=float(t.milliseconds.quantile(.95)))
          for name, t in runtime.groupby('mode')}
timing['pairedMedianDifferenceMilliseconds'] = float(
    (paired.curbRecovery - paired.previousPrecision).median())
tests = pd.read_csv(P / 'tests.csv')
summary = dict(dataset='Mississippi', splits=results, runtime=timing,
               tests={key: int(tests[key].sum()) for key in ['Passed', 'Failed', 'Incomplete']},
               limitations='Prefix and frame 500 are fitted examples; suffix is a reused recording '
                           'used to assess multiple alternatives, not an untouched independent test. '
                           'False fractions are pooled over selected pillars, not per-frame guarantees.')
(P / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
paths = ['config/mississippiCurbPillarPrecisionModel.json',
         'config/curbPillarPrecisionModel.json',
         'output/semantic_precision_20260927/mississippi_baseline.mat',
         'output/semantic_precision_20260927/mississippi_curb_features.csv',
         'output/curb_recovery_20260928/forest_predictions.csv']
artifacts = []
for path in paths:
    file = ROOT / path
    with file.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    artifacts.append(dict(path=path, bytes=file.stat().st_size, sha256=digest))
(P / 'artifact_hashes.json').write_text(json.dumps(artifacts, indent=2) + '\n')
print(json.dumps(dict(suffix=results['suffix']['curb'], frame500=results['frame500']['curb'],
                     runtime=timing, tests=summary['tests']), indent=2))
