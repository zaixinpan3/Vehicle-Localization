"""Calibrate recovery of omitted raw curb proposals on purged prefix folds."""
from pathlib import Path
import json
import os
import pickle
os.environ['OMP_NUM_THREADS'] = '1'
import numpy as np
import pandas as pd
from sklearn.ensemble import ExtraTreesClassifier, HistGradientBoostingClassifier

ROOT = Path(__file__).resolve().parents[2]
P = Path(__file__).resolve().parent
OUT = ROOT / 'output/curb_proposal_recovery_20260928'
t = pd.read_csv(OUT / 'extra_features.csv')
frame = t.frame.to_numpy()
dev = frame <= 780
fold = np.minimum(4, ((frame-1)*5/780).astype(int))
points = t.finePointCount.to_numpy()
y = points > 0
all_names = list(t.columns[3:])
raster_names = [n for n in all_names if not n.startswith('raw_')]
assert len(all_names) == 157 and len(raster_names) == 112
records = []
best = None

def factory(kind):
    if kind == 'forest':
        return ExtraTreesClassifier(n_estimators=80, max_depth=18,
            min_samples_leaf=3, max_features=.7, class_weight='balanced',
            n_jobs=2, random_state=928)
    return HistGradientBoostingClassifier(max_iter=180, max_depth=6,
        max_leaf_nodes=31, min_samples_leaf=25, learning_rate=.05,
        l2_regularization=2, early_stopping=False, class_weight='balanced', random_state=928)

for kind, raw in [('boosted', False), ('boosted', True), ('forest', True)]:
    names = all_names if raw else raster_names
    X = t[names].to_numpy()
    X = np.floor(X*1e8+.5)/1e8
    oof = np.full(len(t), -np.inf)
    for k in range(5):
        test = dev & (fold == k)
        f = frame[test]
        train = dev & ((frame < f.min()-20) | (frame > f.max()+20))
        m = factory(kind).fit(X[train], y[train])
        oof[test] = m.predict_proba(X[test])[:, 1]
    ix = np.flatnonzero(dev)
    order = ix[np.argsort(-oof[ix], kind='stable')]
    n = np.arange(1, len(order)+1)
    false = np.cumsum(~y[order])
    covered = np.cumsum(points[order])
    distinct = np.r_[oof[order][:-1] != oof[order][1:], True]
    valid = np.flatnonzero((n >= 30) & (false/n <= .05) & distinct)
    if not len(valid):
        records.append(dict(kind=kind, raw=raw, feasible=False))
        continue
    at = valid[np.argmax(covered[valid])]
    choice = dict(kind=kind, raw=raw, feasible=True, threshold=float(oof[order[at]]),
        selected=int(n[at]), false=int(false[at]), coveredPoints=int(covered[at]))
    records.append(choice)
    print('PREFIX', json.dumps(choice), flush=True)
    model = factory(kind).fit(X[dev], y[dev])
    score = model.predict_proba(X)[:, 1]
    label = kind + ('_raw' if raw else '_raster')
    t[['frame', 'pillar', 'finePointCount']].assign(score=score, oof=oof).to_csv(OUT / (label+'_predictions.csv'), index=False)
    with (OUT / (label+'.pickle')).open('wb') as stream:
        pickle.dump(dict(model=model, names=names, choice=choice), stream)
    if best is None or choice['coveredPoints'] > best['coveredPoints']:
        best = choice
    (P / 'prefix_search.json').write_text(json.dumps(records, indent=2)+'\n')
assert best is not None
base = pd.read_csv(ROOT / 'research/curb_recovery_20260928/replay.csv')
base = base[base.feature == 'curb']
results = {}
for choice in records:
    if not choice['feasible']:
        continue
    label = choice['kind'] + ('_raw' if choice['raw'] else '_raster')
    pred = pd.read_csv(OUT / (label+'_predictions.csv'))
    accept = pred.score.to_numpy() >= choice['threshold']
    results[label] = {}
    for split, mask, b in [('prefix', dev, base[base.frame <= 780]),
                           ('suffix', ~dev, base[base.frame > 780]),
                           ('all', np.ones(len(t), bool), base),
                           ('frame900', frame == 900, base[base.frame == 900]),
                           ('frame500', frame == 500, base[base.frame == 500])]:
        a = accept & mask
        added = int(a.sum())
        fp = int((a & ~y).sum())
        pts = int(points[a].sum())
        selected = added+int(b.candidatePillarCount.sum())
        false = fp+int(b.extraPillarCount.sum())
        total = int(b.finePointCountInRoi.sum())
        coverage = pts+int(b.coveredFinePointCount.sum())
        results[label][split] = dict(added=added, addedFalse=fp, addedPoints=pts,
            selected=selected, false=false, falseFraction=false/selected,
            coveredPoints=coverage, referencePoints=total, pointCoverage=coverage/total)
    print('VALIDATION', label, json.dumps(results[label]), flush=True)
(P / 'model_validation.json').write_text(json.dumps(dict(prefixChoice=best, alternatives=results), indent=2)+'\n')
