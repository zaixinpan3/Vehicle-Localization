"""Explore physical-distribution neighbor evidence without frame/XY identifiers."""
from pathlib import Path
import os
os.environ['OMP_NUM_THREADS']='2';os.environ['OPENBLAS_NUM_THREADS']='2'
import numpy as np,pandas as pd,json
from sklearn.neighbors import NearestNeighbors
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent;OUT=ROOT/'output/frame827_perception_20260928'
t=pd.read_csv(ROOT/'output/semantic_precision_20260927/mississippi_curb_features.csv');train=t.frame<=780;query=t.frame==827;y=t.finePointCount.to_numpy()>0
basic=['range','ground_countMap','ground_heightMap','ground_roughnessMap','energy_heightStepMeters','energy_relativeHeightMeters','energy_fullCellReliefMeters','raw_spanZ','raw_planeRms','raw_zQ05','raw_zQ25','raw_zQ50','raw_zQ75','raw_zQ95','raw_contextSlope','raw_contextRms','raw_ownerContextQ10','raw_ownerContextQ50','raw_ownerContextQ90','raw_stepMax','raw_stepBestSideRms','raw_stepOwnMiddleFraction']
records=[]
for schema,names in [('basic',basic),('all',list(t.columns[3:]))]:
 X=t[names].to_numpy();center=np.median(X[train],axis=0);scale=np.quantile(X[train],.75,axis=0)-np.quantile(X[train],.25,axis=0);scale=np.where(scale>1e-6,scale,np.maximum(np.std(X[train],axis=0),1e-6));X=(X-center)/scale;X=np.clip(X,-10,10)
 m=NearestNeighbors(n_neighbors=50,n_jobs=2).fit(X[train]);dist,idx=m.kneighbors(X[query]);labels=y[train][idx]
 for k in [5,15,30,50]:
  score=labels[:,:k].mean(axis=1)
  for threshold in [.5,.7,.8,.9,.95,1]:
   take=score>=threshold;record=dict(schema=schema,k=k,threshold=threshold,selected=int(take.sum()),false=int((take&~y[query]).sum()),covered=int(t.loc[query,'finePointCount'].to_numpy()[take].sum()));records.append(record)
  if k==15:t.loc[query,['frame','pillar','finePointCount']].assign(score=score).to_csv(OUT/f'neighbor_{schema}_827.csv',index=False)
 print(schema,'done',flush=True)
(DEST/'neighbor_probe.json').write_text(json.dumps(records,indent=2)+'\n');print(json.dumps(records),flush=True)
