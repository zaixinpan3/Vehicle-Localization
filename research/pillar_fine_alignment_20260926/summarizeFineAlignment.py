"""Aggregate exact fine-point coverage, pillar errors and paired runtime."""
from pathlib import Path
import csv
import json
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent


def read(name):
    with (ROOT / name).open() as stream:
        return list(csv.DictReader(stream))


def aggregate(rows):
    keys = ['finePointCountInRoi', 'coveredFinePointCount', 'finePillarCount',
            'coveredFinePillarCount', 'candidatePillarCount', 'extraPillarCount']
    result = {k: sum(int(r[k]) for r in rows) for k in keys}
    result['frames'] = len(rows)
    result['pointCoverage'] = result['coveredFinePointCount'] / result['finePointCountInRoi']
    result['pointMissFraction'] = 1 - result['pointCoverage']
    result['pillarRecall'] = result['coveredFinePillarCount'] / result['finePillarCount']
    result['extraPillarFraction'] = result['extraPillarCount'] / result['candidatePillarCount']
    result['pillarPrecision'] = 1 - result['extraPillarFraction']
    result['emptyReferenceFrames'] = sum(int(r['finePillarCount']) == 0 for r in rows)
    result['extraOnEmptyReferenceFrames'] = sum(int(r['extraPillarCount']) for r in rows if int(r['finePillarCount']) == 0)
    return result


def main():
    baseline = read('baseline_frames.csv')
    final = read('final_full_frames.csv')
    assert len(final) == 1170 and [int(r['frame']) for r in final] == list(range(1, 1171))
    assert all(r['nonPoleComponentsUnchanged'] == '1' for r in final)
    rows = {}
    for label, predicate in [('full', lambda f: True), ('development', lambda f: f <= 780),
                             ('temporal', lambda f: f > 780)]:
        rows[label] = {k: aggregate([r for r in data if predicate(int(r['frame']))])
                       for k, data in [('baseline', baseline), ('final', final)]}
    rows['downtown'] = {'baseline': aggregate(read('baseline_downtown_frames.csv')),
                        'final': aggregate(read('final_downtown_frames.csv'))}
    for pair in rows.values():
        pair['extraPillarReduction'] = 1 - pair['final']['extraPillarCount'] / pair['baseline']['extraPillarCount']
    timing = read('paired_runtime.csv')
    runtime = {}
    for dataset in ['Mississippi', 'Downtown']:
        runtime[dataset] = {}
        for variant, label in [('1', 'baseline'), ('2', 'final')]:
            values = [float(r['milliseconds']) for r in timing if r['dataset'] == dataset and r['variant'] == variant]
            assert len(values) == 72
            runtime[dataset][label] = dict(calls=len(values), medianMs=statistics.median(values), meanMs=statistics.mean(values))
    tests = read('tests.csv')
    result = dict(reference='Original offline 0.3 m fine pole point indices', quality=rows, runtime=runtime,
                  validation=dict(tests=len(tests), passed=sum(r['Passed'] == '1' for r in tests),
                                  failed=sum(r['Failed'] == '1' for r in tests), incomplete=sum(r['Incomplete'] == '1' for r in tests)),
                  limitation='Sparse temporal and Downtown checks were reused to select the final coverage profile; not untouched test sets.')
    (ROOT / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), layout='constrained')
    labels=['Mississippi\n1,170 frames', 'Downtown\n24 frames']
    for ax, metric, title in zip(axes, ['pointCoverage', 'pillarRecall', 'extraPillarFraction'],
                                ['Fine pole point coverage', 'Positive pillar recall', 'Extra fraction of selected pillars']):
        for i, (variant, color) in enumerate([('baseline', '#9aa7b4'), ('final', '#007b83')]):
            values = [100 * rows[s][variant][metric] for s in ['full', 'downtown']]
            bars = ax.bar([x + (i-.5)*.34 for x in range(2)], values, .34, label=variant, color=color)
            ax.bar_label(bars, fmt='%.1f', padding=3, fontsize=9)
        ax.set_xticks([0, 1], labels);ax.set_ylim(0, 110);ax.set_title(title, fontsize=10)
        ax.set_ylabel('%');ax.spines[['top','right']].set_visible(False)
    axes[0].legend(frameon=False, loc='lower left')
    fig.savefig(ROOT/'quality_comparison.png', dpi=180)
    fig.savefig(ROOT/'quality_comparison.pdf')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
