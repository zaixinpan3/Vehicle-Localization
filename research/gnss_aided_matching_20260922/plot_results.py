"""Export the paired localization result without changing any measurements."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
data = np.genfromtxt(root / 'frame_errors.csv', delimiter=',', names=True).view(np.recarray)
rejected = np.genfromtxt(root / 'rejected_frames.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')['frame']
accepted = ~np.isin(data.frame, rejected)
fig, axes = plt.subplots(3, 1, figsize=(11, 9), layout='constrained')
axes[0].plot(data.frame, 100 * data.originalM, color='#999999', lw=1, label='Original recursive matching')
axes[0].plot(data.frame[accepted], 100 * data.closedAidedM[accepted], color='#126bb5', lw=1, label='GNSS-aided matching (accepted)')
axes[0].axhline(30, color='#c44', ls=':', lw=1)
axes[0].set(ylabel='Position error (cm)', title='LiDAR matching: fixed perception, five-scan horizon and map')
axes[0].legend(loc='upper left', ncol=2)
axes[1].plot(data.frame, 100 * data.originalFusionM, color='#999999', lw=1, label='Original GNSS + LiDAR fusion')
axes[1].plot(data.frame, 100 * data.aidedFusionM, color='#16835d', lw=1, label='Closed-loop aided fusion')
axes[1].set(ylabel='Position error (cm)', title='Fusion: identical gains, GNSS, motion and common startup offset')
axes[1].legend(loc='upper right', ncol=2)
zoom = (data.frame >= 945) & (data.frame <= 965)
axes[2].plot(data.frame[zoom], 100 * data.originalM[zoom], 'o-', ms=3, color='#999999', label='Original matching')
axes[2].plot(data.frame[zoom], 100 * data.closedAidedM[zoom], 'o-', ms=3, color='#126bb5', label='Aided matching')
axes[2].plot(data.frame[zoom], 100 * data.aidedFusionM[zoom], 'o-', ms=3, color='#16835d', label='Aided fusion')
axes[2].axvspan(950, 959, color='#e9eff5', zorder=-1)
axes[2].set(xlabel='Mississippi frame', ylabel='Position error (cm)', title='Previously biased sign association, frames 950-959')
axes[2].legend(loc='upper left', ncol=3)
for ax in axes:
    ax.grid(alpha=.2)
fig.savefig(root / 'comparison.png', dpi=170)
output = root.parents[1] / 'output' / 'gnss_aided_matching_20260922'
output.mkdir(parents=True, exist_ok=True)
fig.savefig(output / 'comparison.pdf')
