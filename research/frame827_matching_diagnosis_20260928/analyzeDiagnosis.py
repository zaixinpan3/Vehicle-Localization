"""Audit exported frame-827 diagnostics and plot the map-association failure."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DEST = Path(__file__).resolve().parent


def main():
    controls = pd.read_csv(DEST / 'controls.csv').set_index('variant')
    pairs = pd.read_csv(DEST / 'coarse_pairs.csv').set_index('source')
    original = pd.read_csv(DEST / 'original_map.csv').set_index('globalId')
    good = pd.read_csv(DEST / 'fine_from_prediction_pairs.csv').set_index('source')
    bad = pd.read_csv(DEST / 'fine_from_coarse_pairs.csv').set_index('source')
    summary = json.loads((DEST / 'summary.json').read_text())
    assert summary['reproductionMaxAbs'] < 1e-7
    assert abs(controls.loc['production', 'errorM'] - 0.563037887) < 1e-7
    assert controls.loc['no_map_merge', 'errorM'] < .04
    assert controls.loc['unlimited_refinement', 'errorM'] > .81
    assert abs(controls.loc['reference_seed', 'errorM'] - controls.loc['production', 'errorM']) < 1e-6
    xy = original[['x', 'y']]
    point = pairs.loc[17, ['sourceBody_1', 'sourceBody_2']].to_numpy(dtype=float)
    merged = pairs.loc[17, ['targetReferenceBody_1', 'targetReferenceBody_2']].to_numpy(dtype=float)
    facts = dict(
        mapModesSeparationM=float(np.linalg.norm(xy.loc[1263] - xy.loc[1264])),
        source17ToOriginal1263M=float(np.linalg.norm(point - xy.loc[1263])),
        source17ToMergedTargetM=float(np.linalg.norm(point - merged)),
        originalGoodTarget=int(good.loc[17, 'globalTarget']),
        originalBadTarget=int(bad.loc[17, 'globalTarget']),
        reproducedPoseMaxAbs=summary['reproductionMaxAbs'],
        neighborhoodFramesReproduced=16,
        fullRouteAlternativeTested=False,
        productionChanged=False,
    )
    assert facts['originalGoodTarget'] == 1263 and facts['originalBadTarget'] == 1264
    analysis = json.loads((DEST / 'code_analysis.json').read_text())
    assert all(not issue for issue in analysis['issues'])
    facts['codeAnalysisCleanFiles'] = len(analysis['files'])
    viewer = json.loads((DEST / 'features_0827.json').read_text())
    assert viewer['displayedPoints'] == viewer['sourcePoints'] == 65536
    assert not viewer['featureOverlay'] and viewer['markerSize'] == 4
    facts['viewerAllPointsColoredWithoutOverlay'] = True
    (DEST / 'validation.json').write_text(json.dumps(facts, indent=2) + '\n')

    fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    n = pd.read_csv(DEST / 'neighborhood_controls.csv')
    axes[0, 0].plot(n.frame, n.productionErrorM * 100, 'o-', label='Deployed recursive output')
    axes[0, 0].plot(n.frame, n.noMapMergeErrorM * 100, 's--', label='No map merge; same stored seeds')
    axes[0, 0].axvline(827, color='gray', lw=1)
    axes[0, 0].set(xlabel='Frame', ylabel='Position error (cm)', title='Local controls are not a new recursive replay')
    axes[0, 0].legend(fontsize=8)

    ax = axes[0, 1]
    for index, color in [(1263, '#147BA1'), (1264, '#A84D24')]:
        v = xy.loc[index]
        ax.scatter(*v, s=100, facecolors='none', edgecolors=color, label=f'Original map {index}')
        ax.annotate(str(index), v + [.025, .055])
    ax.scatter(*merged, color='black', marker='X', s=80, label='Merged map center')
    ax.scatter(*point, color='#C12EAF', marker='+', s=100, label='Observed mean at reference pose')
    for label, x, y, color in [('Deployed', 'sourceAtSolutionBody_1', 'sourceAtSolutionBody_2', '#D95531')]:
        q = pairs.loc[17, [x, y]].to_numpy(dtype=float)
        ax.scatter(*q, color=color, marker='x', s=80, label=label)
        ax.annotate('', q, point, arrowprops=dict(arrowstyle='->', color=color))
    ax.plot([xy.loc[1263, 'x'], xy.loc[1264, 'x']], [xy.loc[1263, 'y'], xy.loc[1264, 'y']], ':', color='gray')
    ax.set(xlabel='Forward at reference pose (m)', ylabel='Left (m)', title='1.5 m merging combines modes 0.964 m apart')
    ax.set_aspect('equal', adjustable='box'); ax.set(xlim=(12.85, 14.05), ylim=(-3.45, -2.45)); ax.legend(fontsize=8)

    ax = axes[1, 0]
    names = ['production', 'reference_seed', 'unlimited_refinement', 'no_map_merge', 'without_direction', 'single_scan']
    labels = ['Deployed', 'Exact reference seed (oracle)', 'Allow fine to leave coarse basin', 'No map merge', 'Disable new direction factor', 'Current scan only']
    ax.barh(labels, controls.loc[names, 'errorM'].to_numpy() * 100, color=['#D95531'] * 3 + ['#147BA1'] + ['#777777'] * 2)
    ax.invert_yaxis(); ax.set(xlabel='Position error (cm)', title='Same-frame causal controls')

    ax = axes[1, 1]
    targets = pd.read_csv(DEST / 'map_axes.csv')
    for _, row in targets[(targets['class'] == 'curb') & targets.x.between(0, 20) & targets.y.between(-5, 6)].iterrows():
        center = np.array([row.x, row.y]); tangent = np.array([row.tangentX, row.tangentY])
        segment = center + np.array([[-2], [2]]) * row.majorStd * tangent
        ax.plot(segment[:, 0], segment[:, 1], color='black', lw=1)
    curb = pairs[pairs.semanticName == 'curb']
    ax.scatter(curb.sourceBody_1, curb.sourceBody_2, s=22, color='#159CA9', label='Selected curb means at reference')
    ax.scatter(curb.sourceAtSolutionBody_1, curb.sourceAtSolutionBody_2, s=24, marker='x', color='#D95531', label='Same means at deployed pose')
    ax.set(xlim=(0, 20), ylim=(-5, 6), xlabel='Forward (m)', ylabel='Left (m)', title='Curved curb support versus map line segments')
    ax.legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle('Mississippi frame 827: map merging changes the association basin\nNo production settings changed in this diagnosis')
    for extension in ['png', 'pdf']:
        fig.savefig(DEST / f'diagnosis.{extension}', dpi=160)
    plt.close(fig)
    print(json.dumps(facts, indent=2))


if __name__ == '__main__':
    main()
