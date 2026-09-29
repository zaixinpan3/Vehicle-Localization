"""Export compact diagnostic evidence without including raw point-cloud data."""
from pathlib import Path
import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
DEST=Path(__file__).resolve().parent
p=pd.read_csv(DEST/'pole_deskew_diagnosis_806.csv');p=p[p.phase==.1]
fit=json.loads((DEST/'timing_origin_fit.json').read_text())[0]
fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
ax=axes[0];ax.plot(p.frame,p.beforeX,'o-',ms=3,label='Original fine-map observations');ax.plot(p.frame,p.afterX,'o-',ms=3,label='Deskew diagnostic (100 ms phase)')
ax.set(xlabel='Frame',ylabel='Fixed reference-frame X (m)',title='A narrow pole drifts across mapping observations');ax.legend(fontsize=8);ax.grid(alpha=.2)
ax=axes[1];x=np.arange(2);before=[fit[s]['originalRmsM'] for s in ['training','heldOut']];after=[fit[s]['correctedRmsM'] for s in ['training','heldOut']]
ax.bar(x-.18,np.array(before)*100,.36,label='Original');ax.bar(x+.18,np.array(after)*100,.36,label='95 ms phase fit on first half')
ax.set(xticks=x,xticklabels=['Frames 1–585 (fit)','Frames 586–1170 (held out)'],ylabel='Within-track XY scatter RMS (cm)',title='Timing correction transfers within this recording');ax.legend(fontsize=8);ax.grid(axis='y',alpha=.2)
fig.savefig(DEST/'scan_motion_diagnosis.png',dpi=170)
fig.savefig(DEST/'scan_motion_diagnosis.pdf')
