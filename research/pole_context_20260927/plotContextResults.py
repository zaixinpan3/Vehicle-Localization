"""Export the measured quality tradeoff as a standalone research figure."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
P=Path(__file__).resolve().parent
results=json.loads((P/'summary.json').read_text())['results']
keys=['Mississippi_full','Downtown'];labels=['Mississippi\n1170 frames','Downtown\n24 frames']
fig,axes=plt.subplots(1,3,figsize=(11,3.8),layout='constrained')
for ax,metric,title in zip(axes,['extraPillars','pointCoverage','pillarRecall'],['Extra selected pillars','Original fine point coverage','Positive pillar recall']):
 for k,(variant,label,color) in enumerate([('baseline','Previous', '#a6b1bd'),('current','Current','#2878b5')]):
  values=[results[key][variant][metric] for key in keys]
  if metric!='extraPillars':values=[100*v for v in values]
  bars=ax.bar(np.arange(2)+(k-.5)*.36,values,.36,label=label,color=color)
  ax.bar_label(bars,fmt='%.0f' if metric=='extraPillars' else '%.2f',fontsize=9,padding=3)
 ax.set_xticks([0,1],labels);ax.set_title(title,fontsize=11);ax.spines[['top','right']].set_visible(False)
 if metric!='extraPillars':
  ax.set_ylim(0,108);ax.axhline(80,color='#9c3030',ls=':',lw=1);ax.set_ylabel('%')
 else:ax.set_ylim(0,3700)
axes[0].legend(frameon=False,fontsize=9)
fig.suptitle('0.6 m pillar selection against frozen original 0.3 m fine pole points',fontsize=12)
fig.savefig(P/'quality_comparison.png',dpi=180);fig.savefig(P/'quality_comparison.pdf')
