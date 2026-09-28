"""Evaluate a bounded distribution bank as a conservative curb support gate."""
from pathlib import Path
import os
os.environ['OMP_NUM_THREADS']='2';os.environ['OPENBLAS_NUM_THREADS']='2'
import numpy as np,pandas as pd,json,pickle
from sklearn.neighbors import NearestNeighbors
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent;OUT=ROOT/'output/frame827_perception_20260928'
t=pd.read_csv(ROOT/'output/semantic_precision_20260927/mississippi_curb_features.csv');names=list(t.columns[3:]);X=t[names].to_numpy();y=t.finePointCount.to_numpy()>0;frame=t.frame.to_numpy();dev=frame<=780;fold=np.minimum(4,((frame-1)*5/780).astype(int));oof=np.full(len(t),-np.inf)
base=pd.read_csv(ROOT/'output/curb_recovery_20260928/forest_predictions.csv');assert np.array_equal(t[['frame','pillar']].values,base[['frame','pillar']].values);threshold=json.load(open(ROOT/'config/mississippiCurbPillarPrecisionModel.json'))['decisionThreshold']
def fit_predict(train,test):
 center=np.median(X[train],axis=0);scale=np.quantile(X[train],.75,axis=0)-np.quantile(X[train],.25,axis=0);scale=np.where(scale>1e-6,scale,np.maximum(np.std(X[train],axis=0),1e-6));xx=np.clip((X-center)/scale,-10,10)
 rng=np.random.default_rng(928);bank=rng.choice(np.flatnonzero(train),min(10000,int(train.sum())),replace=False);nn=NearestNeighbors(n_neighbors=30,n_jobs=2).fit(xx[bank]);idx=nn.kneighbors(xx[test],return_distance=False);score=y[bank][idx].mean(axis=1)
 return score,dict(center=center,scale=scale,bank=xx[bank],labels=y[bank],names=names)
for k in range(5):
 test=dev&(fold==k);f=frame[test];train=dev&((frame<f.min()-20)|(frame>f.max()+20));oof[test],_=fit_predict(train,test);print('Fold',k,'done',flush=True)
score,bundle=fit_predict(dev,np.ones(len(t),bool));records=[]
for minimum in [0,.5,.7,.8,.9,.95,.97,1]:
 take=score>=minimum;old=base.score.to_numpy()>=threshold;prefix=dev&(oof>=minimum)&(base.oof.to_numpy()>=threshold)
 result={'minimumAgreement':minimum,'prefixOOF':{'selected':int(prefix.sum()),'false':int((prefix&~y).sum()),'covered':int(t.finePointCount[prefix].sum())}}
 for label,mask in [('suffix',~dev),('frame827',frame==827)]:
  selected=take&old&mask;result[label]={'selected':int(selected.sum()),'false':int((selected&~y).sum()),'covered':int(t.finePointCount[selected].sum())}
 records.append(result)
with (OUT/'consensus.pickle').open('wb') as f:pickle.dump(bundle,f)
t[['frame','pillar','finePointCount']].assign(score=score,oof=oof).to_csv(OUT/'consensus_predictions.csv',index=False)
(DEST/'consensus_probe.json').write_text(json.dumps(records,indent=2)+'\n');print(json.dumps(records),flush=True)
