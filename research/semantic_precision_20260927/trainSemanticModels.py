"""Select per-class precision gates using purged prefix predictions only."""
from pathlib import Path
import os,json,pickle,hashlib,sys
os.environ['OMP_NUM_THREADS']='1'
import numpy as np
import pandas as pd
from sklearn.ensemble import HistGradientBoostingClassifier
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent;OUT=ROOT/'output/semantic_precision_20260927'
LIMITS={'mississippi':780,'downtown':360};PURGE={'mississippi':20,'downtown':10}

def metrics(table,score,threshold,den,mask):
 result={}
 for ds in table.dataset.unique():
  use=mask & (table.dataset.to_numpy()==ds);take=use & (score>=threshold)
  frames=np.unique(table.frame.to_numpy()[use]);base=den[(den.dataset.str.lower()==ds)&den.frame.isin(frames)]
  # Keep all frame denominators, including frames without candidates.
  if np.array_equal(mask,table.frame.to_numpy()<=table.dataset.map(LIMITS).to_numpy()):base=den[(den.dataset.str.lower()==ds)&(den.frame<=LIMITS[ds])]
  count=int(take.sum());false=int((take & (table.finePointCount.to_numpy()==0)).sum());covered=int(table.finePointCount.to_numpy()[take].sum())
  result[ds]=dict(selected=count,false=false,falseFraction=false/count if count else None,coveredPoints=covered,referencePoints=int(base.finePointCountInRoi.sum()),pointCoverage=covered/max(int(base.finePointCountInRoi.sum()),1))
 return result

def select(t,score,dev,den,cap=.05):
 thresholds={};result={}
 for dataset in t.dataset.unique():
  subset=dev & (t.dataset.to_numpy()==dataset);indices=np.flatnonzero(subset);order=indices[np.argsort(-score[indices],kind='stable')]
  n=np.arange(1,len(order)+1);labels=t.finePointCount.to_numpy()[order];false=np.cumsum(labels==0);covered=np.cumsum(labels)
  last=np.r_[score[order][:-1]!=score[order][1:],True];valid=(n>=30)&(false/n<=cap)&last
  if not valid.any():return None
  choices=np.flatnonzero(valid);at=choices[np.argmax(covered[choices])];thresholds[dataset]=float(score[order[at]])
  base=den[(den.dataset.str.lower()==dataset)&(den.frame<=LIMITS[dataset])];total=int(base.finePointCountInRoi.sum())
  result[dataset]=dict(selected=int(n[at]),false=int(false[at]),falseFraction=float(false[at]/n[at]),coveredPoints=int(covered[at]),referencePoints=total,pointCoverage=float(covered[at]/max(total,1)))
 rank=(min(v['pointCoverage'] for v in result.values()),sum(v['coveredPoints'] for v in result.values()))
 return rank,thresholds,result

def export(model,names,threshold,semantic,filename=None):
 result=dict(schemaVersion=1,featureNames=list(names),decisionThreshold=max(threshold.values()),decisionThresholds=threshold,featureScale=1e8,initialLogit=float(model._baseline_prediction[0,0]),roots=[],feature=[],threshold=[],left=[],right=[],value=[],leaf=[],semantic=semantic,training='Mississippi <=780, Downtown <=360; five purged temporal folds; seed 927; frozen original 0.3 m fine labels only offline')
 for predictors in model._predictors:
  n=predictors[0].nodes;offset=len(result['feature']);result['roots'].append(offset+1)
  result['feature']+=(n['feature_idx'].astype(int)+1).tolist();result['threshold']+=n['num_threshold'].tolist()
  result['left']+=(n['left'].astype(int)+offset+1).tolist();result['right']+=(n['right'].astype(int)+offset+1).tolist();result['value']+=n['value'].tolist();result['leaf']+=n['is_leaf'].astype(bool).tolist()
 (OUT/(filename or f'{semantic}_model.json')).write_text(json.dumps(result,separators=(',',':'))+'\n')
 return result

def main():
 summaries=json.loads((P/'model_validation.json').read_text()) if (P/'model_validation.json').exists() else {};comparisons=[]
 for semantic in (sys.argv[1:] or ['curb','trafficSign','facade']):
  pieces=[];dens=[]
  for ds in LIMITS:
   source=OUT/f'{ds}_{semantic}_features.csv'
   if not source.exists():continue
   t=pd.read_csv(source).copy();t['dataset']=ds;pieces.append(t)
   den=pd.read_csv(P/f'{ds}_baseline.csv');dens.append(den[den.feature==semantic])
  table=pd.concat(pieces,ignore_index=True);den=pd.concat(dens,ignore_index=True)
  names=[c for c in table.columns if c not in ['dataset','frame','pillar','finePointCount']]
  X=table[names].to_numpy(float);X=np.floor(np.nan_to_num(X)*1e8+.5)/1e8;y=table.finePointCount.to_numpy()>0;ds=table.dataset.to_numpy();frame=table.frame.to_numpy();dev=frame<=table.dataset.map(LIMITS).to_numpy()
  folds=np.minimum(4,((frame-1)*5/table.dataset.map(LIMITS).to_numpy()).astype(int));best=None
  for depth in [4,6]:
   def factory():return HistGradientBoostingClassifier(max_iter=200,max_depth=depth,max_leaf_nodes=31,min_samples_leaf=25,learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=927)
   def weights(use):
    w=np.ones(use.sum());dataset=ds[use];labels=y[use];npoints=table.finePointCount.to_numpy()[use]
    for d in np.unique(dataset):
     pos=(dataset==d)&labels;neg=(dataset==d)&~labels
     w[pos]=np.sqrt(1+np.minimum(npoints[pos],100)/10);w[pos]/=max(w[pos].sum(),1);w[neg]/=max(neg.sum(),1)
    return w/w.mean()
   score=np.full(len(table),-np.inf)
   for fold in range(5):
    test=dev&(folds==fold);train=dev&~test
    for d in np.unique(ds):
     fs=frame[test&(ds==d)]
     if len(fs):train &= ~((ds==d)&(frame>=fs.min()-PURGE[d])&(frame<=fs.max()+PURGE[d]))
    model=factory();model.fit(X[train],y[train],sample_weight=weights(train));score[test]=model.predict_proba(X[test])[:,1]
   np.save(OUT/f'{semantic}_depth{depth}_oof.npy',score)
   choice=select(table,score,dev,den)
   entry=dict(semantic=semantic,depth=depth,features=len(names),choice=None if choice is None else dict(threshold=choice[1],metrics=choice[2]));comparisons.append(entry);print(json.dumps(entry),flush=True)
   if choice is not None and (best is None or choice[0]>best[0]):
    model=factory();model.fit(X[dev],y[dev],sample_weight=weights(dev));best=(choice[0],choice[1],model,score.copy(),depth,choice[2])
  (P/'model_comparison.json').write_text(json.dumps(comparisons,indent=2)+'\n')
  if best is None:
   summaries[semantic]=dict(status='No prefix operating point');print('NO OPERATING POINT',semantic,flush=True);continue
  _,threshold,model,oof,depth,prefix=best;export(model,names,threshold,semantic)
  with (OUT/f'{semantic}_model.pickle').open('wb') as f:pickle.dump((model,names),f)
  score=model.predict_proba(X)[:,1];prediction=table[['dataset','frame','pillar','finePointCount']].copy();prediction['score']=score;prediction['oof']=oof;prediction.to_csv(OUT/f'{semantic}_predictions.csv',index=False)
  # Report all frame denominators, not just frames represented in the candidate table.
  result={}
  for d in np.unique(ds):
   result[d]={}
   for split in ['prefix','suffix','all']:
    include=ds==d;denmask=den.dataset.str.lower()==d
    if split=='prefix':include &= frame<=LIMITS[d];denmask &= den.frame<=LIMITS[d]
    if split=='suffix':include &= frame>LIMITS[d];denmask &= den.frame>LIMITS[d]
    take=include&(score>=threshold[d]);count=int(take.sum());fp=int((take&~y).sum());covered=int(table.finePointCount.to_numpy()[take].sum());base=den[denmask]
    result[d][split]=dict(selected=count,false=fp,falseFraction=fp/count if count else None,coveredPoints=covered,referencePoints=int(base.finePointCountInRoi.sum()),pointCoverage=covered/max(int(base.finePointCountInRoi.sum()),1),baselineSelected=int(base.candidatePillarCount.sum()),baselineFalse=int(base.extraPillarCount.sum()),baselineCovered=int(base.coveredFinePointCount.sum()))
  summaries[semantic]=dict(thresholds=threshold,depth=depth,features=len(names),prefixOOF=prefix,validation=result)
  (P/'model_validation.json').write_text(json.dumps(summaries,indent=2)+'\n');print('VALIDATION',semantic,json.dumps(result),flush=True)
  importance=np.zeros(len(names))
  for predictors in model._predictors:
   nodes=predictors[0].nodes
   for node in nodes:
    if not node['is_leaf']:importance[int(node['feature_idx'])]+=max(float(node['gain']),0)
  pd.DataFrame({'feature':names,'gain':importance}).sort_values('gain',ascending=False).to_csv(P/f'{semantic}_feature_importance.csv',index=False)
 (P/'model_validation.json').write_text(json.dumps(summaries,indent=2)+'\n')
 (P/'training_protocol.json').write_text(json.dumps(dict(seed=927,limits=LIMITS,purge=PURGE,folds=5,selection='maximize minimum dataset reference-point coverage with profile-specific thresholds at <=5% prefix OOF reference-empty selection and at least 30 selections per dataset',features='whole 0.6 m pillar distributions and neighboring raster summaries; no frame identity or absolute XY coordinates',suffix='reused recordings; suffix labels are excluded from fitting and operating-point selection'),indent=2)+'\n')
if __name__=='__main__':main()
