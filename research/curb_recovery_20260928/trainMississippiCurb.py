"""Compare whole-pillar and connected-curve evidence on purged prefixes only."""
from pathlib import Path
import os,json,pickle
os.environ['OMP_NUM_THREADS']='1'
import numpy as np,pandas as pd
from sklearn.ensemble import HistGradientBoostingClassifier
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent;OUT=ROOT/'output/curb_recovery_20260928';OLD=ROOT/'output/semantic_precision_20260927'

def main():
 t=pd.read_csv(OLD/'mississippi_curb_features.csv');groups=pd.read_csv(OLD/'mississippi_curb_group_features.csv');raw=list(t.columns[3:]);t=t.merge(groups,on=['frame','pillar'],validate='one_to_one');assert not t.isna().any().any()
 allnames=list(t.columns[3:]);frame=t.frame.to_numpy();dev=frame<=780;folds=np.minimum(4,((frame-1)*5/780).astype(int));labels=t.finePointCount.to_numpy();y=labels>0;best=None;records=[]
 for label,names,depth in [('raw',raw,6),('groups',allnames,6),('groups_deeper',allnames,8)]:
  X=np.floor(t[names].to_numpy()*1e8+.5)/1e8;oof=np.full(len(t),-np.inf)
  def factory():return HistGradientBoostingClassifier(max_iter=200,max_depth=depth,max_leaf_nodes=31,min_samples_leaf=25,learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=928)
  def weights(mask):
   z=y[mask];w=np.ones(mask.sum());w[z]=np.sqrt(1+np.minimum(labels[mask][z],100)/10);w[z]/=w[z].sum();w[~z]/=(~z).sum();return w/w.mean()
  for fold in range(5):
   test=dev&(folds==fold);f=frame[test];train=dev&((frame<f.min()-20)|(frame>f.max()+20));model=factory().fit(X[train],y[train],sample_weight=weights(train));oof[test]=model.predict_proba(X[test])[:,1]
  ix=np.flatnonzero(dev);order=ix[np.argsort(-oof[ix],kind='stable')];n=np.arange(1,len(order)+1);fp=np.cumsum(~y[order]);covered=np.cumsum(labels[order]);last=np.r_[oof[order][:-1]!=oof[order][1:],True];valid=(n>=30)&(fp/n<=.05)&last;assert valid.any();choices=np.flatnonzero(valid);at=choices[np.argmax(covered[choices])]
  choice=dict(label=label,features=len(names),depth=depth,threshold=float(oof[order[at]]),selected=int(n[at]),false=int(fp[at]),coveredPoints=int(covered[at]),referencePoints=91697);records.append(choice);print('PREFIX',json.dumps(choice),flush=True);np.save(OUT/(label+'_oof.npy'),oof)
  if best is None or choice['coveredPoints']>best['choice']['coveredPoints']:
   model=factory().fit(X[dev],y[dev],sample_weight=weights(dev));best=dict(model=model,names=names,choice=choice,oof=oof)
  (P/'classifier_search.json').write_text(json.dumps(records,indent=2)+'\n')
 choice=best['choice'];model=best['model'];names=best['names'];X=np.floor(t[names].to_numpy()*1e8+.5)/1e8;score=model.predict_proba(X)[:,1];take=score>=choice['threshold'];den=pd.read_csv(ROOT/'research/semantic_precision_20260927/mississippi_baseline.csv');den=den[den.feature=='curb'];validation={}
 for name,mask,dmask in [('prefix',dev,den.frame<=780),('suffix',~dev,den.frame>780),('all',np.ones(len(t),bool),np.ones(len(den),bool)),('frame500',frame==500,den.frame==500)]:
  selected=mask&take;n=int(selected.sum());f=int((selected&~y).sum());pts=int(labels[selected].sum());total=int(den.finePointCountInRoi[dmask].sum());validation[name]=dict(selected=n,false=f,falseFraction=f/n,coveredPoints=pts,referencePoints=total,pointCoverage=pts/total)
 print('VALIDATION',json.dumps(validation),flush=True)
 result=dict(schemaVersion=1,featureNames=names,semantic='curb',featureScale=1e8,initialLogit=float(model._baseline_prediction[0,0]),decisionThreshold=choice['threshold'],decisionThresholds={'mississippi':choice['threshold']},roots=[],feature=[],threshold=[],left=[],right=[],value=[],leaf=[],training='Mississippi <=780; 5 temporal folds with 20 frame purge; seed 928; original frozen 0.3 m fine labels used offline only')
 for predictors in model._predictors:
  nodes=predictors[0].nodes;offset=len(result['feature']);result['roots'].append(offset+1);result['feature']+=(nodes['feature_idx'].astype(int)+1).tolist();result['threshold']+=nodes['num_threshold'].tolist();result['left']+=(nodes['left'].astype(int)+offset+1).tolist();result['right']+=(nodes['right'].astype(int)+offset+1).tolist();result['value']+=nodes['value'].tolist();result['leaf']+=nodes['is_leaf'].astype(bool).tolist()
 (OUT/'mississippi_curb_model.json').write_text(json.dumps(result,separators=(',',':'))+'\n')
 with (OUT/'mississippi_curb_model.pickle').open('wb') as f:pickle.dump(best,f)
 t[['frame','pillar','finePointCount']].assign(score=score,oof=best['oof']).to_csv(OUT/'classifier_predictions.csv',index=False)
 (P/'classifier_validation.json').write_text(json.dumps(dict(choice=choice,validation=validation),indent=2)+'\n')
if __name__=='__main__':main()
