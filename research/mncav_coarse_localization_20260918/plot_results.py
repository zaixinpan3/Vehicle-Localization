#!/usr/bin/env python3
"""Render recorded localization errors without rerunning any estimator.

uv run --offline --with numpy --with h5py --with matplotlib python research/mncav_coarse_localization_20260918/plot_results.py
"""
from pathlib import Path
import h5py
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[2]
out = root / 'output/mncav_coarse_localization_20260918'
with h5py.File(out / 'observer/experiment.mat') as f:
    time = f['data/highRate/time'][()].ravel()
    reference = f['reference'][()].T
    poses = [f[f['runs'][0, k]]['estimate/pose'][()].T for k in range(3)]

fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True, layout='constrained')
colors = ['#1764aa', '#d47912', '#44843b']
labels = ['BESTPOS + coarse LiDAR', 'Coarse LiDAR only', 'BESTPOS only']
for pose, label, color in zip(poses, labels, colors):
    error = np.linalg.norm(pose[:, :2] - reference[:, :2], axis=1)
    angle = np.rad2deg(np.arctan2(np.sin(pose[:, 2]-reference[:, 2]),
                                np.cos(pose[:, 2]-reference[:, 2])))
    axes[0].plot(time, 100*error, color=color, lw=1.1,
                 label=f'{label} (RMSE {100*np.sqrt(np.mean(error**2)):.2f} cm)')
    axes[1].plot(time, angle, color=color, lw=1.1, label=label)
for ax in axes:
    ax.grid(alpha=.23)
    ax.spines[['top', 'right']].set_visible(False)
    ax.set_xlim(time[0], time[-1])
axes[0].set_ylabel('Position discrepancy (cm)')
axes[1].set_ylabel('Heading discrepancy (degrees)')
axes[1].set_xlabel('Receiver time from first scan (s)')
axes[0].legend(frameon=False, fontsize=9, loc='upper center')
fig.suptitle('Mississippi: raw coarse perception + recursive matching\n'
             '1169 observer frames; same-drive map and shared INSPVA reference', fontsize=12)
fig.savefig(out / 'localization_errors.png', dpi=180)
fig.savefig(out / 'localization_errors.pdf')
plt.close(fig)
