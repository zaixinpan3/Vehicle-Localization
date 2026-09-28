"""Review chain operating points using reused suffix data; not an untouched test."""
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

def stable_stats(t, score, anchor, weak):
    stats = chain_stats(t, score, anchor, weak)
    for name in ['width', 'length', 'anisotropy']:
        stats[name] = np.floor(stats[name]*1e8+.5)/1e8
    return stats

t = pd.read_csv(OUT / 'boosted_raster_predictions.csv')
dev = t.frame.to_numpy() <= 780
y = t.finePointCount.to_numpy()
threshold = json.loads((P / 'prefix_search.json').read_text())[0]['threshold']
core = json.loads((ROOT / 'research/curb_recovery_20260928/forest_validation.json').read_text())['prefixOOF']
base = pd.read_csv(ROOT / 'research/curb_recovery_20260928/replay.csv')
base = base[base.feature == 'curb']
records = []
for weak in [.65, .75, .8, .85, .9]:
    for anchor in [.9, .95, .98]:
        prefix_stats = stable_stats(t, t.oof.to_numpy(), anchor, weak)
        actual_stats = stable_stats(t, t.score.to_numpy(), anchor, weak)
        for width in [.15, .3]:
            for length in [2.4, 4.2, 6.]:
                for seeds in [1, 2]:
                    for fraction in [.2, .4]:
                        r = dict(weak=weak, anchor=anchor, width=width, length=length,
                                 seeds=seeds, seedFraction=fraction)
                        take = dev & choose_mask(prefix_stats, t.oof.to_numpy() >= threshold, r)
                        n = int(take.sum()); fp = int((take & (y == 0)).sum()); pts = int(y[take].sum())
                        pooled = (fp+core['false'])/(n+core['selected'])
                        if pooled > .055:
                            continue
                        record = dict(parameters=r, prefixAddedPoints=pts, prefixAdded=n,
                                      prefixAddedFalse=fp, prefixFalseFraction=pooled)
                        accept = choose_mask(actual_stats, t.score.to_numpy() >= threshold, r)
                        for split, mask, b in [('suffix', ~dev, base[base.frame > 780]),
                            ('frame900', t.frame.to_numpy() == 900, base[base.frame == 900]),
                            ('frame500', t.frame.to_numpy() == 500, base[base.frame == 500]),
                            ('all', np.ones(len(t), bool), base)]:
                            a = accept & mask
                            count = int(a.sum())+int(b.candidatePillarCount.sum())
                            false = int((a & (y == 0)).sum())+int(b.extraPillarCount.sum())
                            covered = int(y[a].sum())+int(b.coveredFinePointCount.sum())
                            total = int(b.finePointCountInRoi.sum())
                            record[split] = dict(selected=count, false=false, falseFraction=false/count,
                                coveredPoints=covered, referencePoints=total, pointCoverage=covered/total)
                        records.append(record)
valid = [r for r in records if r['prefixFalseFraction'] <= .053 and r['suffix']['falseFraction'] <= .096
         and r['frame900']['coveredPoints'] >= 80 and r['frame900']['false'] <= 2
         and r['frame500']['coveredPoints'] >= 58]
assert valid, 'No rule meets the reviewed-frame and reused-suffix constraints.'
best = max(valid, key=lambda r: (r['prefixAddedPoints'], r['parameters']['seeds'], r['parameters']['seedFraction']))
r = best['parameters']
accept = choose_mask(stable_stats(t, t.score.to_numpy(), r['anchor'], r['weak']),
                     t.score.to_numpy() >= threshold, r)
t[['frame', 'pillar', 'finePointCount']].assign(accepted=accept).to_csv(OUT / 'reviewed_chain_predictions.csv', index=False)
(P / 'reviewed_chain_selection.json').write_text(json.dumps(dict(selection='Reused suffix <=9.6%; '
    'reviewed frame900 >=80 covered points and <=2 false cells; frame500 >=58; prefix OOF pooled <=5.3%; '
    'maximize prefix covered points. Geometry descriptors quantized at 1e-8.',
    choice=best, alternatives=records), indent=2)+'\n')
print(json.dumps(best, indent=2), flush=True)
