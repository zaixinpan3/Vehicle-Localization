"""Plot causal online results, including the common startup error."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
read = lambda name: np.genfromtxt(root / name, delimiter=',', names=True)
robust = read('switchable_frames.csv')
plain = read('quadratic_frames.csv')
guided = read('guided_frames.csv')
fig, axes = plt.subplots(3, 1, figsize=(11, 9), layout='constrained')
frame = robust['frame']
axes[0].plot(frame, 100 * plain['errors'], color='#bbb', lw=1, label='Quadratic graph')
axes[0].plot(frame, 100 * robust['errors'], color='#d48c21', lw=1, label='Switchable graph')
axes[0].plot(frame, 100 * robust['before'], color='#126bb5', lw=1, label='Current observer baseline')
axes[0].axhline(30, color='#c44', ls=':', lw=1)
axes[0].set(title='Full 1169-frame fusion replay (common 64 cm startup retained)')
axes[0].legend(ncol=3, loc='upper left')
axes[1].plot(frame, 100 * robust['before'], color='#126bb5', lw=1, label='Current observer baseline')
axes[1].plot(frame, 100 * guided['fusionErrorM'], color='#16835d', lw=1, ls='--', label='Graph proposals + existing observer')
axes[1].set(title='Conservative integration: nearly identical accuracy, higher computation')
axes[1].legend(loc='upper left', ncol=2)
zoom = (frame >= 830) & (frame <= 855)
axes[2].plot(frame[zoom], 100 * robust['errors'][zoom], color='#d48c21', marker='.', label='Switchable graph fusion')
axes[2].plot(frame[zoom], 100 * guided['fusionErrorM'][zoom], color='#16835d', marker='.', label='Graph-guided observer fusion')
axes[2].plot(frame[zoom], 100 * guided['baselineMatchingErrorM'][zoom], color='#777', marker='.', label='Baseline LiDAR matching')
axes[2].plot(frame[zoom], 100 * guided['matchingErrorM'][zoom], color='#9b4d9d', ls='--', label='Graph-guided LiDAR matching')
axes[2].set(title='Remaining matching peak around frame 840', xlabel='Mississippi frame')
axes[2].legend(ncol=2, loc='upper right', fontsize=9)
for ax in axes:
    ax.set_ylabel('Position error (cm)')
    ax.grid(alpha=.2)
fig.savefig(root / 'comparison.png', dpi=170)
output = root.parents[1] / 'output' / 'robust_pose_graph_20260923'
fig.savefig(output / 'comparison.pdf')
