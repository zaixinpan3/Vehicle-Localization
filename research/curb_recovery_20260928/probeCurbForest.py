"""Compare a randomized-tree ranker with the boosted-tree operating point."""
from pathlib import Path
import os,json,pickle
os.environ['OMP_NUM_THREADS']='1'
import numpy as np,pandas as pd
from sklearn.ensemble import ExtraTreesClassifier
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent;OLD=ROOT/'output/semantic_precision_20260927';OUT=ROOT/'output/curb_recovery_20260928'
OUT.mkdir(parents=True,exist_ok=True)
t=pd.read_csv(OLD/'mississippi_curb_features.csv');names=list(t.columns[3:]);X=np.floor(t[names].to_numpy()*1e8+.5)/1e8;y=t.finePointCount.to_numpy()>0;points=t.finePointCount.to_numpy();frame=t.frame.to_numpy();dev=frame<=780;fold=np.minimum(4,((frame-1)*5/780).astype(int));oof=np.full(len(t),-np.inf)
def factory():return ExtraTreesClassifier(n_estimators=120,max_depth=18,min_samples_leaf=3,max_features=.7,n_jobs=2,random_state=928,class_weight='balanced')
for k in range(5):
 test=dev&(fold==k);frames=frame[test];train=dev&((frame<frames.min()-20)|(frame>frames.max()+20));m=factory().fit(X[train],y[train]);oof[test]=m.predict_proba(X[test])[:,1];print('Fold',k,'complete',flush=True)
ix=np.flatnonzero(dev);order=ix[np.argsort(-oof[ix],kind='stable')];n=np.arange(1,len(order)+1);f=np.cumsum(~y[order]);covered=np.cumsum(points[order]);last=np.r_[oof[order][:-1]!=oof[order][1:],True];valid=(n>=30)&(f/n<=.05)&last;assert valid.any();choices=np.flatnonzero(valid);at=choices[np.argmax(covered[choices])];threshold=float(oof[order[at]]);prefix=dict(threshold=threshold,selected=int(n[at]),false=int(f[at]),coveredPoints=int(covered[at]));print('PREFIX',json.dumps(prefix),flush=True)
m=factory().fit(X[dev],y[dev]);score=m.predict_proba(X)[:,1];take=score>=threshold;den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];validation={}
for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',frame==500,den.frame==500)]:
 s=take&mask;n=int(s.sum());false=int((s&~y).sum());pts=int(points[s].sum());total=int(den.finePointCountInRoi[dmask].sum());validation[name]=dict(selected=n,false=false,falseFraction=false/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
print('VALIDATION',json.dumps(validation),flush=True)
(P/'forest_validation.json').write_text(json.dumps(dict(prefixOOF=prefix,validation=validation),indent=2)+'\n');t[['frame','pillar','finePointCount']].assign(score=score,oof=oof).to_csv(OUT/'forest_predictions.csv',index=False)
with (OUT/'forest.pickle').open('wb') as f:pickle.dump(dict(model=m,names=names,threshold=threshold),f)
