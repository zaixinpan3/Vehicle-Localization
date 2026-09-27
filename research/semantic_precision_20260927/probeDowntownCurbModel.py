"""Test a dedicated Downtown curb ranker on the same purged prefix folds."""
from trainSemanticModels import OUT,P,LIMITS,export
import numpy as np,pandas as pd,json,pickle,sys
from sklearn.ensemble import HistGradientBoostingClassifier

grouped='--groups' in sys.argv;prefix='downtown_curb_groups' if grouped else 'downtown_curb'
t=pd.read_csv(OUT/('downtown_curb_group_augmented.csv' if grouped else 'downtown_curb_features.csv'));names=list(t.columns[3:]);X=t[names].to_numpy();y=t.finePointCount.to_numpy()>0;frame=t.frame.to_numpy();dev=frame<=360;folds=np.minimum(4,((frame-1)*5/360).astype(int));best=None;records=[]
for depth in [2,3,4,5,6]:
 for leaf in [10,25]:
  score=np.full(len(t),-np.inf)
  def factory():return HistGradientBoostingClassifier(max_iter=250,max_depth=depth,max_leaf_nodes=31,min_samples_leaf=leaf,learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=927)
  def weights(use):
   w=np.ones(use.sum());pos=y[use];w[pos]=np.sqrt(1+np.minimum(t.finePointCount.to_numpy()[use][pos],100)/10);w[pos]/=w[pos].sum();w[~pos]/=(~pos).sum();return w/w.mean()
  for fold in range(5):
   test=dev&(folds==fold);fs=frame[test];train=dev&((frame<fs.min()-10)|(frame>fs.max()+10));model=factory();model.fit(X[train],y[train],sample_weight=weights(train));score[test]=model.predict_proba(X[test])[:,1]
  indices=np.flatnonzero(dev);order=indices[np.argsort(-score[indices],kind='stable')];n=np.arange(1,len(order)+1);fp=np.cumsum(~y[order]);covered=np.cumsum(t.finePointCount.to_numpy()[order]);last=np.r_[score[order][:-1]!=score[order][1:],True]
  valid=(n>=30)&(fp/n<=.05)&last;result=None
  if valid.any():
   choices=np.flatnonzero(valid);at=choices[np.argmax(covered[choices])];threshold=float(score[order[at]]);result=dict(selected=int(n[at]),false=int(fp[at]),covered=int(covered[at]),threshold=threshold)
   if best is None or result['covered']>best['result']['covered']:
    model=factory();model.fit(X[dev],y[dev],sample_weight=weights(dev));best=dict(model=model,names=names,result=result,depth=depth,leaf=leaf,oof=score.copy())
  records.append(dict(depth=depth,leaf=leaf,prefix=result));print(json.dumps(records[-1]),flush=True)
  (P/(prefix+'_model_probe.json')).write_text(json.dumps(records,indent=2)+'\n')
if best:
 with (OUT/(prefix+'_dedicated.pickle')).open('wb') as f:pickle.dump(best,f)
 score=best['model'].predict_proba(X)[:,1];t[['frame','pillar','finePointCount']].assign(score=score,oof=best['oof']).to_csv(OUT/(prefix+'_dedicated_predictions.csv'),index=False)
 print('WINNER',json.dumps({k:v for k,v in best.items() if k not in ['model','names','oof']}),flush=True)
else:print('No dedicated model passes the prefix criterion',flush=True)
