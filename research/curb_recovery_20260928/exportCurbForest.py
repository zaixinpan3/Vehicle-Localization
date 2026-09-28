"""Export the fitted Mississippi-only forest for toolbox-free MATLAB scoring."""
from pathlib import Path
import json
import pickle

ROOT = Path(__file__).resolve().parents[2]
with (ROOT / 'output/curb_recovery_20260928/forest.pickle').open('rb') as stream:
    fitted = pickle.load(stream)
model = dict(
    schemaVersion=1, kind='probabilityForest', semantic='curb',
    featureNames=fitted['names'], featureScale=1e8,
    decisionThreshold=fitted['threshold'],
    decisionThresholds={'mississippi': fitted['threshold']},
    roots=[], feature=[], threshold=[], left=[], right=[], value=[], leaf=[],
    training='Mississippi frames 1:780; five contiguous folds with 20-frame '
             'purge; seed 928; 120 extra trees, depth 18, leaf 3, 70% features; '
             'balanced classes; 5% prefix OOF false-pillar calibration; '
             'frozen original 0.3 m fine labels used only offline')
for estimator in fitted['model'].estimators_:
    tree = estimator.tree_
    offset = len(model['feature'])
    model['roots'].append(offset + 1)
    model['feature'] += (tree.feature + 1).tolist()
    model['threshold'] += tree.threshold.tolist()
    model['left'] += (tree.children_left + offset + 1).tolist()
    model['right'] += (tree.children_right + offset + 1).tolist()
    values = tree.value[:, 0, :]
    model['value'] += (values[:, 1] / values.sum(axis=1)).tolist()
    model['leaf'] += (tree.children_left < 0).tolist()
path = ROOT / 'config/mississippiCurbPillarPrecisionModel.json'
path.write_text(json.dumps(model, separators=(',', ':')) + '\n')
print(f'{len(model["roots"])} trees, {len(model["leaf"])} nodes, {path.stat().st_size} bytes')
