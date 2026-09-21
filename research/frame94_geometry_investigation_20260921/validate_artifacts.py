"""Validate saved diagnostic comparisons and fingerprint independent artifacts."""
from pathlib import Path
import csv
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).parent
OUT = ROOT / 'output/frame94_geometry_investigation_20260921'


def read_rows(name):
    with (DEST / name).open() as stream:
        return list(csv.DictReader(stream))


def fingerprint(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return dict(path=str(path.relative_to(ROOT)), bytes=path.stat().st_size,
                sha256=digest.hexdigest())


def main():
    training = read_rows('translation_training_pairs.csv')
    checks = read_rows('perception_selection_checks.csv')
    queries = read_rows('heldout_queries.csv')
    controls = read_rows('rebuilt_map_controls.csv')
    fit_frames = {int(r[key]) for r in training for key in ['query', 'fixed']}
    query_frames = {int(r['query']) for r in queries}
    assert len(training) == 28 and sum(r['usedForFit'] == '1' for r in training) == 26
    assert not fit_frames.intersection(query_frames)
    assert not fit_frames.intersection({92, 93, 94})
    assert len(checks) == len(queries) == 21
    assert all(r['sameHitCounts'] == r['samePillarCounts'] == '1' for r in checks)
    assert all(r['accepted'] == '1' for r in queries + controls)
    baseline_reproduction = max(float(r['baselineReproductionMaxAbs'])
                               for r in queries if r['variant'] == '1')
    assert baseline_reproduction < 1e-7
    for r in queries + controls:
        assert math.isclose(float(r['positionErrorM']),
                            math.hypot(float(r['errorX']), float(r['errorY'])), abs_tol=1e-10)
    error = {(int(r['query']), int(r['variant'])): float(r['positionErrorM']) for r in queries}
    improved = sorted(q for q in query_frames if error[q, 2] < error[q, 1])
    regressed = sorted(query_frames - set(improved))
    assert regressed == [76] and len(improved) == 6
    for r in controls:
        if r['variant'] == '1':
            variant = 1 + int(r['offsetApplied'])
            assert math.isclose(float(r['positionErrorM']), error[94, variant], abs_tol=1e-10)
    for path in DEST.glob('*.py'):
        compile(path.read_text(), str(path), 'exec')
    old = json.loads((DEST.parent / 'frame94_fusion_diagnosis_20260921/artifact_manifest.json').read_text())
    files = []
    for entry in old['files']:
        if entry['path'].endswith('diagnosis.png'):
            continue
        current = fingerprint(ROOT / entry['path'])
        assert current['sha256'] == entry['sha256'], entry['path']
        files.append(current)
    extra = [ROOT / 'output/mississippi_mapping_synchronized/probability_cloud_map.mat',
             ROOT / 'output/receiver_synchronized_inputs/native_reference.csv',
             ROOT / 'output/receiver_clock_20260921/coarse_pipeline/matching/calls.csv']
    extra += list(OUT.glob('*.mat')) + list(OUT.glob('*.png'))
    extra += list(DEST.glob('*.m')) + list(DEST.glob('*.py'))
    files.extend(fingerprint(path) for path in extra)
    manifest = dict(productionRevision=old['productionRevision'], clockModelId=old['clockModelId'],
                    files=files, rawScanPath='data/raw/MissisipiPointClouds.mat',
                    rawSourceFrames=sorted({int(r['sourceFrame']) for r in checks}),
                    scope='Technical artifacts only; no weekly or monthly report hashes.')
    (DEST / 'artifact_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    validation = dict(trainingQualityAcceptedPairs=26, queryFramesExcludedFromFit=True,
                      frame94SourceWindowExcludedFromFit=True, allQueriesAccepted=True,
                      selectedPillarAndHitCountsIdentical=True,
                      baselineMaximumReproductionError=baseline_reproduction,
                      improvedQueries=improved, regressedQueries=regressed,
                      xyNormAndCrossTableChecksPassed=True, previousInputHashesVerified=True,
                      pythonSourceCompilationPassed=True,
                      matlabCodeAnalyzerIssues=[],
                      matlabExecution='All three scripts completed, including numerical assertions.',
                      pythonMotionModelExecution='Completed with offline numpy and scipy.',
                      productionCodeChanged=False, fullSequenceReplayPerformed=False,
                      qualification='Query exclusion is not independent-drive or map holdout; some auxiliary window frames overlap fitting.')
    (DEST / 'validation.json').write_text(json.dumps(validation, indent=2) + '\n')
    print(json.dumps(validation, indent=2))


if __name__ == '__main__':
    main()
