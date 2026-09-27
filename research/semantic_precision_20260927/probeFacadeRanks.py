"""Compare conservative ensemble and neighborhood ranks on purged prefix folds."""
from trainSemanticModels import OUT,P
import numpy as np,pandas as pd,json,pickle
from sklearn.ensemble import ExtraTreesClassifier,RandomForestClassifier
from sklearn.preprocessing import RobustScaler
from sklearn.neighbors import NearestNeighbors
from sklearn.pipeline import make_pipeline
from sklearn.neighbors import KNeighborsClassifier

t=pd.read_csv(OUT/'downtown_facade_features.csv');y=t.finePointCount.to_numpy()>0;frame=t.frame.to_numpy();dev=frame<=360;folds=np.minimum(4,((frame-1)*5/360).astype(int));records=[];best=None
cases=[('extra_raw',False,'extra'),('extra_wide',True,'extra'),('forest_wide',True,'forest'),('knn_raw',False,'knn'),('knn_wide',True,'knn')]
for label,wide,kind in cases:
 names=[n for n in t.columns[3:] if wide or not n.startswith('wide')];X=t[names].to_numpy();score=np.full(len(t),-np.inf)
 def factory():
  if kind=='extra':return ExtraTreesClassifier(n_estimators=250,max_depth=12,min_samples_leaf=3,max_features=.7,class_weight='balanced',n_jobs=2,random_state=927)
  if kind=='forest':return RandomForestClassifier(n_estimators=250,max_depth=12,min_samples_leaf=3,max_features=.7,class_weight='balanced',n_jobs=2,random_state=927)
  return make_pipeline(RobustScaler(quantile_range=(10,90)),KNeighborsClassifier(n_neighbors=9,weights='uniform',algorithm='brute',n_jobs=2))
 for fold in range(5):
  test=dev&(folds==fold);fs=frame[test];train=dev&((frame<fs.min()-10)|(frame>fs.max()+10));model=factory();model.fit(X[train],y[train]);score[test]=model.predict_proba(X[test])[:,1]
 indices=np.flatnonzero(dev);order=indices[np.argsort(-score[indices],kind='stable')];n=np.arange(1,len(order)+1);fp=np.cumsum(~y[order]);covered=np.cumsum(t.finePointCount.to_numpy()[order]);last=np.r_[score[order][:-1]!=score[order][1:],True]
 valid=(n>=30)&(fp/n<=.05)&last;result=None
 if valid.any():
  choices=np.flatnonzero(valid);at=choices[np.argmax(covered[choices])];threshold=float(score[order[at]]);result=dict(selected=int(n[at]),false=int(fp[at]),covered=int(covered[at]),threshold=threshold)
  if best is None or result['covered']>best['result']['covered']:
   model=factory();model.fit(X[dev],y[dev]);best=dict(model=model,names=names,result=result,label=label,oof=score.copy())
 at=np.argmin(np.where((n>=30)&last,fp/n,np.inf));records.append(dict(label=label,prefix=result,bestFalseFraction=float(fp[at]/n[at]),bestSelected=int(n[at])));print(json.dumps(records[-1]),flush=True)
 np.save(OUT/(label+'_facade_oof.npy'),score);(P/'facade_rank_probe.json').write_text(json.dumps(records,indent=2)+'\n')
 if best:
  with (OUT/'facade_rank_winner.pickle').open('wb') as f:pickle.dump(best,f)
if best:print('WINNER',best['label'],best['result'],flush=True)
