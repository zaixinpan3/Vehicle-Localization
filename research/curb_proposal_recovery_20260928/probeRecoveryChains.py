"""Test slender support chains in omitted raw proposals with prefix calibration."""
from pathlib import Path
import sys
import json
import numpy as np
import pandas as pd
ROOT = Path(__file__).resolve().parents[2]
P = Path(__file__).resolve().parent
OUT = ROOT / 'output/curb_proposal_recovery_20260928'
sys.path.insert(0, str(ROOT / 'research/curb_recovery_20260928'))
from probeCurbChains import chain_stats, choose_mask

t = pd.read_csv(OUT / 'boosted_raster_predictions.csv')
dev = t.frame.to_numpy() <= 780
y = t.finePointCount.to_numpy()
threshold = json.loads((P / 'prefix_search.json').read_text())[0]['threshold']
records = []
best = None
# The unchanged core forest's purged-prefix OOF counts, not fitted counts.
core = json.loads((ROOT / 'research/curb_recovery_20260928/forest_validation.json').read_text())['prefixOOF']
for weak in [.65, .75, .8, .85, .9]:
    for anchor in [.9, .95, .98]:
        stats = chain_stats(t, t.oof.to_numpy(), anchor, weak)
        for width in [.15, .3]:
            for length in [2.4, 4.2, 6.]:
                for seeds in [1, 2]:
                    for fraction in [.2, .4]:
                        r = dict(weak=weak, anchor=anchor, width=width, length=length,
                                 seeds=seeds, seedFraction=fraction)
                        take = dev & choose_mask(stats, t.oof.to_numpy() >= threshold, r)
                        count = int(take.sum())
                        false = int((take & (y == 0)).sum())
                        points = int(y[take].sum())
                        pooled = (false+core['false'])/(count+core['selected'])
                        record = dict(**r, added=count, addedFalse=false, addedPoints=points,
                                      combinedPrefixFalseFraction=pooled)
                        records.append(record)
                        if pooled <= .053 and (best is None or points > best['addedPoints']):
                            best = record
assert best
print('PREFIX', json.dumps(best), flush=True)
stats = chain_stats(t, t.score.to_numpy(), best['anchor'], best['weak'])
take = choose_mask(stats, t.score.to_numpy() >= threshold, best)
base = pd.read_csv(ROOT / 'research/curb_recovery_20260928/replay.csv')
base = base[base.feature == 'curb']
results = {}
for split, mask, b in [('suffix', ~dev, base[base.frame > 780]),
                       ('all', np.ones(len(t), bool), base),
                       ('frame900', t.frame.to_numpy() == 900, base[base.frame == 900]),
                       ('frame500', t.frame.to_numpy() == 500, base[base.frame == 500])]:
    a = take & mask
    added = int(a.sum()); fp = int((a & (y == 0)).sum()); pts = int(y[a].sum())
    selected = added+int(b.candidatePillarCount.sum()); false = fp+int(b.extraPillarCount.sum())
    total = int(b.finePointCountInRoi.sum()); covered = pts+int(b.coveredFinePointCount.sum())
    results[split] = dict(added=added, addedFalse=fp, addedPoints=pts, selected=selected,
                         false=false, falseFraction=false/selected, coveredPoints=covered,
                         referencePoints=total, pointCoverage=covered/total)
print('VALIDATION', json.dumps(results), flush=True)
(P / 'chain_validation.json').write_text(json.dumps(dict(prefixCap=.053, prefixChoice=best,
    validation=results, alternatives=records), indent=2)+'\n')
t[['frame', 'pillar', 'finePointCount']].assign(accepted=take).to_csv(OUT / 'chain_predictions.csv', index=False)
