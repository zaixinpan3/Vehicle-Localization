"""Diagnostic continuous profile moments over existing 0.6 m pillar neighborhoods."""
from pathlib import Path
import numpy as np,pandas as pd
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent;OUT=ROOT/'output/frame827_perception_20260928'
t=pd.read_csv(DEST/'curb_cells.csv');p=pd.read_csv(OUT/'ground_points.csv');xyz=p[['x','y','z']].to_numpy();tree=cKDTree(xyz[:,:2]);rows=[]
for radius in [.75,1.05,1.5]:
 for _,r in t.iterrows():
  center=np.array([r.x,r.y]);idx=tree.query_ball_point(center,radius);q=xyz[idx]-np.r_[center,0]
  if len(q)<15:continue
  linear=np.column_stack([np.ones(len(q)),q[:,:2]]);b=np.linalg.lstsq(linear,q[:,2],rcond=None)[0];normal=b[1:]/max(np.linalg.norm(b[1:]),1e-10);v=q[:,:2]@normal;u=q[:,:2]@np.array([-normal[1],normal[0]])
  A=np.column_stack([np.ones(len(q)),u,v,v*v,v**3]);coef=np.linalg.lstsq(A,q[:,2],rcond=None)[0];offset=-coef[3]/(3*coef[4]) if abs(coef[4])>1e-5 else 99
  own=np.all(abs(q[:,:2])<.3,axis=1);limit=.3*np.sum(abs(normal));slope=coef[2]+2*coef[3]*offset+3*coef[4]*offset**2
  rows.append(dict(radius=radius,pillar=int(r.pillar),fine=int(r.finePointCount),selected=int(r.selected),offset=offset,extent=limit,normalizedOffset=abs(offset)/limit,negativeCubic=coef[4]<0,slope=slope,rms=float(np.sqrt(np.mean((q[:,2]-A@coef)**2))),support=len(q)))
a=pd.DataFrame(rows);a.to_csv(DEST/'inflection_probe.csv',index=False)
for radius in [.75,1.05,1.5]:
 x=a[(a.radius==radius) & ((a.fine>0)|(a.selected==1))];print(radius);print(x[['pillar','fine','selected','normalizedOffset','negativeCubic','slope']].to_string(index=False))
