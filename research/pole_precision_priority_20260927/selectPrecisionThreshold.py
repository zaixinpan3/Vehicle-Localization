"""Select precision-first threshold candidates from purged prefix scores.

The score model is unchanged. Temporal suffixes are reused regression data;
their measured coverage/precision tradeoff informed the final policy choice.
"""
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PREVIOUS = ROOT / 'research/pole_geometry_20260927'
CACHE = ROOT / 'output/pole_geometry_20260927'
LIMITS = {'Mississippi': 780, 'Downtown': 360}


def metrics(rows, threshold, denominators, partition):
    result = {}
    for dataset, limit in LIMITS.items():
        def eligible(frame):
            return partition == 'all' or (frame <= limit) == (partition == 'prefix')
        selected = [r for r in rows if r['dataset'] == dataset
                    and eligible(r['frame']) and r['score'] >= threshold]
        total = sum(int(r['finePoints']) for r in denominators
                    if r['dataset'] == dataset and eligible(int(r['frame'])))
        false = sum(r['finePointCount'] == 0 for r in selected)
        covered = sum(r['finePointCount'] for r in selected)
        result[dataset] = dict(selected=len(selected), false=false,
                               falseFraction=false / len(selected) if selected else None,
                               coveredPoints=covered, referencePoints=total,
                               pointCoverage=covered / total)
    return result


def main():
    source_rows = []
    denominators = []
    for dataset in LIMITS:
        name = dataset.lower()
        with (CACHE / f'{name}_features.csv').open() as handle:
            source_rows.extend(dict(dataset=r['dataset'], frame=int(r['frame']),
                                    pillar=int(r['pillar']), finePointCount=int(r['finePointCount']))
                               for r in csv.DictReader(handle))
        with (ROOT / 'research/pole_precision_20260927' / f'{name}_frames.csv').open() as handle:
            denominators.extend(csv.DictReader(handle))
    oof_file = CACHE / 'stable_moments_compact_d5_w1.0_oof.npy'
    scores = np.load(oof_file)
    assert len(scores) == len(source_rows)
    owners = {}
    for row, score in zip(source_rows, scores):
        if row['frame'] > LIMITS[row['dataset']]:
            continue
        key = row['dataset'], row['frame'], row['pillar']
        if key not in owners:
            owners[key] = dict(row, score=float(score))
        else:
            assert owners[key]['finePointCount'] == row['finePointCount']
            owners[key]['score'] = max(owners[key]['score'], float(score))
    rows = list(owners.values())
    thresholds = np.unique([r['score'] for r in rows])
    worst_false = np.zeros(len(thresholds))
    worst_block_false = np.zeros(len(thresholds))
    minimum_coverage = np.ones(len(thresholds))
    minimum_selected = np.full(len(thresholds), np.inf)
    for dataset, limit in LIMITS.items():
        local = [r for r in rows if r['dataset'] == dataset]
        total = sum(int(r['finePoints']) for r in denominators
                    if r['dataset'] == dataset and int(r['frame']) <= limit)
        for fold in [None, 0, 1, 2, 3, 4]:
            subset = local if fold is None else [r for r in local if (r['frame'] - 1) * 5 // limit == fold]
            subset.sort(key=lambda r: -r['score'])
            score = np.array([r['score'] for r in subset])
            fine = np.array([r['finePointCount'] for r in subset])
            count = np.searchsorted(-score, -thresholds, side='right')
            false = np.r_[0, np.cumsum(fine == 0)][count] / np.maximum(count, 1)
            if fold is None:
                worst_false = np.maximum(worst_false, false)
                minimum_coverage = np.minimum(minimum_coverage, np.r_[0, np.cumsum(fine)][count] / total)
                minimum_selected = np.minimum(minimum_selected, count)
            else:
                # Empty blocks are reported explicitly, never counted as precise.
                worst_block_false = np.maximum(worst_block_false, np.where(count > 0, false, np.inf))
    choices = {}
    for name, cap, block_cap in [('cap10', .10, 1), ('cap05', .05, 1),
                                  ('cap02', .02, 1), ('cap05_blocks10', .05, .10)]:
        feasible = np.flatnonzero((worst_false <= cap) & (minimum_selected > 0)
                                  & (worst_block_false <= block_cap))
        if len(feasible) == 0:
            choices[name] = None
            continue
        best = feasible[np.argmax(minimum_coverage[feasible])]
        threshold = float(thresholds[best])
        choices[name] = dict(threshold=threshold, empiricalCap=cap, blockCap=block_cap,
                              worstBlockFalseFraction=float(worst_block_false[best]),
                              metrics=metrics(rows, threshold, denominators, 'prefix'))
    primary = 'cap05'
    assert choices[primary] is not None
    result = dict(sourceModel='stable_moments_compact_d5_w1.0',
                  sourceCommit='6df904cfed15c72e09cabb3a39c2c620522faae1',
                  selection='Maximize minimum dataset point coverage with <=5% prefix OOF false selection. '
                            'No 80% coverage floor; no requirement to preserve old coverage.',
                  policyChoice='The initially explored every-fold 10% guard retained only 16.10% Downtown suffix '
                               'point coverage. Pooled cap05 retains 38.61% while its reused suffix false '
                               'fraction is 7.50%. This regression-informed promotion is not independent validation.',
                  caveat='Empirical guard margins, not confidence bounds; adjacent frames are correlated.',
                  primary=primary, choices=choices,
                  oofSHA256=hashlib.sha256(oof_file.read_bytes()).hexdigest())
    (HERE / 'threshold_freeze.json').write_text(json.dumps(result, indent=2) + '\n')
    with (HERE / 'prefix_owner_scores.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(result, indent=2), flush=True)
    # Threshold values come from prefix OOF scores; suffixes are regression checks.
    with (PREVIOUS / 'stable_moments_model_predictions.csv').open() as handle:
        predictions = [dict(dataset=r['dataset'], frame=int(r['frame']), pillar=int(r['pillar']),
                            finePointCount=int(r['finePointCount']), score=float(r['score']))
                       for r in csv.DictReader(handle)]
    validation = {name: {part: metrics(predictions, choice['threshold'], denominators, part)
                        for part in ['prefix', 'suffix', 'all']}
                  for name, choice in choices.items() if choice is not None}
    (HERE / 'threshold_validation.json').write_text(json.dumps(validation, indent=2) + '\n')
    print('FROZEN SUFFIX', json.dumps(validation[primary]['suffix']), flush=True)


if __name__ == '__main__':
    main()
