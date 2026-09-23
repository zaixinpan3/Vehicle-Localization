"""Plot recorded height-association ablations; no inference is run here."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent

def read(name):
    with (root / name).open() as stream:
        return list(csv.DictReader(stream))

frames, metrics, probes = read('frames.csv'), read('metrics.csv'), read('frozen_probes.csv')
variants = ['xy', 'relative_z_objects', 'marginal_z_objects']
labels = ['XY baseline', 'Conditional Z: pole + sign', 'Marginal Z: pole + sign']
colors = ['#225ea8', '#d95f0e', '#238b45']
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
fig, axs = plt.subplots(2, 2, figsize=(13, 8.5), layout='constrained')
for variant, label, color in zip(variants, labels, colors):
    rows = [r for r in frames if r['variant'] == variant]
    x = np.array([int(r['frame']) for r in rows])
    y = np.array([float(r['matchingM'])*100 if r['accepted']=='1' else np.nan for r in rows])
    axs[0, 0].plot(x, y, label=label, color=color, linewidth=.8, alpha=.85)
    keep = (x >= 150) & (x <= 180)
    axs[0, 1].plot(x[keep], y[keep], '-o', color=color, markersize=3, linewidth=1.2)
for ax in axs[0]:
    ax.set(xlabel='Mississippi frame', ylabel='Position error (cm)')
    ax.grid(alpha=.18)
axs[0, 0].set_title('Accepted LiDAR measurements in causal GNSS-aided replay')
axs[0, 0].legend(fontsize=9, loc='upper left')
axs[0, 1].set_title('Frame 166 control region; rejected measurements omitted')

for model, label, color in zip(['xy', 'conditional', 'marginal'], labels, colors):
    rows = [r for r in probes if r['model']==model and int(r['frame'])>=950]
    axs[1, 0].plot([int(r['frame']) for r in rows], [100*float(r['errorM']) for r in rows],
                   '-o', color=color, markersize=4, label=label)
axs[1, 0].set(title='Historical bad seeds held fixed; GNSS aid disabled',
              xlabel='Mississippi frame', ylabel='Position error (cm)')
axs[1, 0].grid(alpha=.18)
x = np.arange(4)
for i, (variant, label, color) in enumerate(zip(variants, labels, colors)):
    row = next(r for r in metrics if r['variant']==variant and r['population']=='matching_common')
    values = [float(row[k])*100 for k in ['rmseM','p95M','p99M','maxM']]
    axs[1, 1].bar(x + (i-1)*.25, values, .24, color=color, label=label)
axs[1, 1].set(xticks=x, xticklabels=['RMSE', '95th pct.', '99th pct.', 'Maximum'],
              ylabel='Position error (cm)', title=f"Common accepted population: {row['frames']} frames")
axs[1, 1].grid(axis='y', alpha=.18)
fig.suptitle('Relative height for semantic distribution association — 2026-09-23', fontsize=14)
fig.savefig(root/'height_comparison.png', dpi=180)
fig.savefig(root/'height_comparison.pdf')
