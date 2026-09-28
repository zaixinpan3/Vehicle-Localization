"""Test complete-pillar joint XYZ statistics on a purged training prefix."""
from pathlib import Path
import os
os.environ['OMP_NUM_THREADS']='2'
import json,pickle
import numpy as np,pandas as pd
from sklearn.ensemble import HistGradientBoostingClassifier
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent;OUT=ROOT/'output/frame827_perception_20260928'

def main():
 t=pd.read_csv(ROOT/'output/semantic_precision_20260927/mississippi_curb_features.csv');extra=pd.read_csv(OUT/'joint_features.csv');t=t.merge(extra,on=['frame','pillar'],validate='one_to_one');assert len(t)==99361 and not t.isna().any().any();names=list(t.columns[3:]);X=t[names].to_numpy();frame=t.frame.to_numpy();label=t.finePointCount.to_numpy();y=label>0;dev=frame<=780;fold=np.minimum(4,((frame-1)*5/780).astype(int));oof=np.full(len(t),-np.inf)
 def fit(mask):
  w=np.ones(mask.sum());z=y[mask];w[z]=np.sqrt(1+np.minimum(label[mask][z],100)/10);w[z]/=w[z].sum();w[~z]/=(~z).sum();w/=w.mean()
  return HistGradientBoostingClassifier(max_iter=250,max_depth=6,max_leaf_nodes=31,min_samples_leaf=25,learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=928).fit(X[mask],y[mask],sample_weight=w)
 for k in range(5):
  test=dev&(fold==k);fs=frame[test];train=dev&((frame<fs.min()-20)|(frame>fs.max()+20));m=fit(train);oof[test]=m.predict_proba(X[test])[:,1];print('Fold',k,'done',flush=True)
 order=np.flatnonzero(dev);order=order[np.argsort(-oof[order],kind='stable')];count=np.arange(1,len(order)+1);false=np.cumsum(~y[order]);covered=np.cumsum(label[order]);last=np.r_[oof[order][:-1]!=oof[order][1:],True];choices=np.flatnonzero((false/count<=.05)&(count>=30)&last);at=choices[np.argmax(covered[choices])];threshold=float(oof[order[at]])
 choice=dict(threshold=threshold,selected=int(count[at]),false=int(false[at]),covered=int(covered[at]))
 m=fit(dev);score=m.predict_proba(X)[:,1];t[['frame','pillar','finePointCount']].assign(score=score,oof=oof).to_csv(OUT/'joint_predictions.csv',index=False)
 with (OUT/'joint.pickle').open('wb') as f:pickle.dump(dict(model=m,names=names,choice=choice),f)
 results={}
 for name,mask in [('prefixOOF',dev),('suffix',~dev),('frame827',frame==827)]:
  v=oof if name=='prefixOOF' else score;s=mask&(v>=threshold);results[name]=dict(selected=int(s.sum()),false=int((s&~y).sum()),covered=int(label[s].sum()))
 (DEST/'joint_probe.json').write_text(json.dumps(dict(choice=choice,results=results),indent=2)+'\n');print(json.dumps(results),flush=True)
if __name__=='__main__':main()
