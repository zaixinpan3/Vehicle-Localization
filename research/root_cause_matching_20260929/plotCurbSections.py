"""Display diagnostic fine labels; these are never inputs to online estimation."""
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[2]
fig,axes=plt.subplots(1,3,figsize=(14,4),constrained_layout=True)
for ax,k,x,y in zip(axes,[178,854,1096],[8.,9.14,16.98],[6.6,-2.44,-3.04]):
 a=pd.read_csv(root/f'output/root_cause_matching_20260929/ground_section_{k}.csv')
 a=a[(abs(a.x-x)<1.2)&(abs(a.y-y)<.9)]
 ax.scatter(a.y,a.z,s=8,c='#88949f',alpha=.5,label='Ground returns')
 b=a[a.fineCurb==1];ax.scatter(b.y,b.z,s=13,c='#e33b3b',label='Original fine curb')
 ax.set(title=f'Frame {k}; X = {x:.2f} ± 1.2 m',xlabel='Body Y (m)',ylabel='Tilt-corrected Z (m)')
 ax.grid(alpha=.2)
axes[0].legend(fontsize=8)
fig.savefig(root/'research/root_cause_matching_20260929/curb_sections.png',dpi=170)
