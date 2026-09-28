"""Export the raster ranker and reviewed chain operating point for MATLAB."""
from pathlib import Path
import json
import pickle
ROOT = Path(__file__).resolve().parents[2]
P = Path(__file__).resolve().parent
with (ROOT / 'output/curb_proposal_recovery_20260928/boosted_raster.pickle').open('rb') as stream:
    fitted = pickle.load(stream)
model = fitted['model']
choice = fitted['choice']
chain = json.loads((P / 'reviewed_chain_selection.json').read_text())['choice']['parameters']
result = dict(schemaVersion=1, semantic='curb', featureNames=fitted['names'], featureScale=1e8,
    initialLogit=float(model._baseline_prediction[0, 0]), decisionThreshold=choice['threshold'],
    decisionThresholds={'mississippi': choice['threshold']}, roots=[], feature=[],
    threshold=[], left=[], right=[], value=[], leaf=[],
    recovery=dict(**chain, minimumBaseEnergy=.25, minimumAnisotropy=.9, geometryScale=1e8),
    training='Mississippi 1:780; five temporal folds with 20-frame purge; seed 928; '
             '112 whole-pillar raster statistics. Chain policy tuned using reused suffix '
             'and inspected frame 900; no independent test claim; fine labels only offline.')
for predictors in model._predictors:
    nodes = predictors[0].nodes
    offset = len(result['feature'])
    result['roots'].append(offset+1)
    result['feature'] += (nodes['feature_idx'].astype(int)+1).tolist()
    result['threshold'] += nodes['num_threshold'].tolist()
    result['left'] += (nodes['left'].astype(int)+offset+1).tolist()
    result['right'] += (nodes['right'].astype(int)+offset+1).tolist()
    result['value'] += nodes['value'].tolist()
    result['leaf'] += nodes['is_leaf'].astype(bool).tolist()
path = ROOT / 'config/mississippiCurbRecoveryModel.json'
path.write_text(json.dumps(result, separators=(',', ':'))+'\n')
print(path, path.stat().st_size)
