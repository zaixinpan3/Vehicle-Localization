"""Fit point-distribution classifiers with purged temporal development folds.

Frame/pillar/hypothesis IDs and dataset identity are excluded from inference.
Model selection uses prefixes only. Suffix results are opened after freezing.
"""
from pathlib import Path
import csv,json,hashlib,pickle,os
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier,RandomForestClassifier,ExtraTreesClassifier
P=Path(__file__).resolve().parent;OUT=P.parents[1]/'output/pole_precision_20260927'
def load():
 rows=[];den=[]
 for name in ['mississippi','downtown']:
  rows+=list(csv.DictReader((P/(name+'_features.csv')).open()));den+=list(csv.DictReader((P/(name+'_frames.csv')).open()))
 names=[k for k in rows[0] if k not in {'dataset','frame','pillar','hypothesis','finePointCount'}]
 X=np.nan_to_num(np.array([[float(r[k]) for k in names] for r in rows]),nan=0,posinf=0,neginf=0)
 ds=np.array([r['dataset'] for r in rows]);frames=np.array([int(r['frame']) for r in rows]);labels=np.array([int(r['finePointCount']) for r in rows]);y=labels>0
 keys=np.array([f"{r['dataset']}:{r['frame']}:{r['pillar']}" for r in rows]);_,ix,groups=np.unique(keys,return_index=True,return_inverse=True)
 development=((ds=='Mississippi')&(frames<=780))|((ds=='Downtown')&(frames<=360))
 return rows,den,names,X,ds,frames,labels,y,ix,groups,development

def metrics(ownerScore,threshold,ix,ds,frames,labels,den,part):
 result={}
 for name,limit in [('Mississippi',780),('Downtown',360)]:
  select=ds[ix]==name
  if part=='development':select &= frames[ix]<=limit
  if part=='suffix':select &= frames[ix]>limit
  d=[r for r in den if r['dataset']==name and (part=='all' or (int(r['frame'])<=limit)==(part=='development'))]
  npoints=sum(int(r['finePoints']) for r in d);npillars=sum(int(r['finePillars']) for r in d)
  selected=select & (ownerScore>=threshold);tp=int(np.count_nonzero(labels[ix][selected]));n=int(selected.sum());covered=int(labels[ix][selected].sum())
  result[name]=dict(selected=n,matched=tp,extra=n-tp,extraFraction=(n-tp)/n if n else 0,coveredPoints=covered,finePoints=npoints,pointCoverage=covered/max(npoints,1),pillarRecall=tp/max(npillars,1))
 return result

def main():
 rows,den,names,X,ds,frames,labels,y,ix,groups,development=load()
 def weights(train):
  w=np.ones(train.sum());data=ds[train];target=y[train]
  for name in np.unique(data):
   positive=(data==name)&target;negative=(data==name)&~target
   w[positive]=np.sqrt(1+labels[train][positive]/50)
   w[positive]*=.5/max(w[positive].sum(),1);w[negative]=.5/max(negative.sum(),1)
  w/=np.bincount(groups[train],minlength=len(ix))[groups[train]]
  return w/w.mean()
 folds=np.zeros(len(y),int)
 for name,limit in [('Mississippi',780),('Downtown',360)]:
  mask=ds==name;folds[mask]=np.minimum(4,((frames[mask]-1)*5/limit).astype(int))
 factories={'gb2_160':lambda:GradientBoostingClassifier(n_estimators=160,max_depth=2,min_samples_leaf=12,learning_rate=.05,random_state=927),
            'gb3_160':lambda:GradientBoostingClassifier(n_estimators=160,max_depth=3,min_samples_leaf=12,learning_rate=.05,random_state=927),
            'gb3_320':lambda:GradientBoostingClassifier(n_estimators=320,max_depth=3,min_samples_leaf=12,learning_rate=.05,random_state=927),
            'gb4_160':lambda:GradientBoostingClassifier(n_estimators=160,max_depth=4,min_samples_leaf=12,learning_rate=.05,random_state=927),
            'forest':lambda:RandomForestClassifier(n_estimators=250,max_depth=10,min_samples_leaf=5,max_features=.7,random_state=927,n_jobs=1),
            'extra':lambda:ExtraTreesClassifier(n_estimators=250,max_depth=12,min_samples_leaf=4,max_features=.8,random_state=927,n_jobs=1)}
 result=[];candidates={};bypass=(X[:,names.index('radialRms')]<=.035)&(X[:,names.index('isolation')]>=.95)
 for name,factory in factories.items():
  score=np.full(len(y),-np.inf)
  for fold in range(5):
   test=development&(folds==fold);train=development&~test
   for dataset,purge in [('Mississippi',20),('Downtown',10)]:
    fs=frames[test&(ds==dataset)];train &= ~((ds==dataset)&(frames>=fs.min()-purge)&(frames<=fs.max()+purge))
   model=factory();model.fit(X[train],y[train],sample_weight=weights(train));score[test]=model.predict_proba(X[test])[:,1]
  score[development&bypass]=1
  ownerScore=np.full(len(ix),-np.inf);np.maximum.at(ownerScore,groups,score)
  thresholds=np.unique(ownerScore[np.isfinite(ownerScore)])
  valid=np.ones(len(thresholds),bool);minimumCoverage=np.ones(len(thresholds))
  for dataset,limit in [('Mississippi',780),('Downtown',360)]:
   mask=(ds[ix]==dataset)&(frames[ix]<=limit);localScore=ownerScore[mask];localFine=labels[ix][mask]
   order=np.argsort(-localScore,kind='stable');localScore=localScore[order];localFine=localFine[order]
   n=np.searchsorted(-localScore,-thresholds,side='right')
   tp=np.r_[0,np.cumsum(localFine>0)][n];covered=np.r_[0,np.cumsum(localFine)][n]
   total=sum(int(r['finePoints']) for r in den if r['dataset']==dataset and int(r['frame'])<=limit)
   valid &= (n>0)&((n-tp)/np.maximum(n,1)<=.10)
   minimumCoverage=np.minimum(minimumCoverage,covered/max(total,1))
  validRows=np.flatnonzero(valid);best=None
  if len(validRows):
   chosen=validRows[np.argmax(minimumCoverage[validRows])];threshold=float(thresholds[chosen])
   best=dict(threshold=threshold,metrics=metrics(ownerScore,threshold,ix,ds,frames,labels,den,'development'),minimumCoverage=float(minimumCoverage[chosen]))
  result.append(dict(model=name,best=best));print(name,best,flush=True)
  model=factory();model.fit(X[development],y[development],sample_weight=weights(development))
  candidates[name]=(model,score,best)
  with (OUT/(name+'.pickle')).open('wb') as f:pickle.dump(dict(model=model,names=names,threshold=best['threshold'] if best else None),f)
  np.save(OUT/(name+'_oof.npy'),score)
  (P/'prefix_model_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
  (P/(name+'_importance.json')).write_text(json.dumps(sorted(zip(names,model.feature_importances_.tolist()),key=lambda p:-p[1]),indent=2)+'\n')
 winner=max((r for r in result if r['best']),key=lambda r:r['best']['minimumCoverage'])
 frozen=dict(featureNames=names,training='Mississippi <=780, Downtown <=360',purge={'Mississippi':20,'Downtown':10},folds=5,seed=927,winner=winner,analyticBypass='radialRms <= 0.035 m and isolation >= 0.95, after original physical support gates',featureFiles={n:hashlib.sha256((P/n).read_bytes()).hexdigest() for n in ['mississippi_features.csv','downtown_features.csv']})
 (P/'model_freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
 # Only now open predictions/labels from the temporal suffixes.
 model,_,best=candidates[winner['model']];score=model.predict_proba(X)[:,1];score[bypass]=1
 ownerScore=np.full(len(ix),-np.inf);np.maximum.at(ownerScore,groups,score)
 validation={part:metrics(ownerScore,best['threshold'],ix,ds,frames,labels,den,part) for part in ['development','suffix','all']}
 (P/'model_validation.json').write_text(json.dumps(validation,indent=2)+'\n');print('FROZEN VALIDATION',validation,flush=True)
 with (P/'model_predictions.csv').open('w') as f:
  w=csv.writer(f,lineterminator='\n');w.writerow(['dataset','frame','pillar','finePointCount','score'])
  w.writerows([[rows[j]['dataset'],rows[j]['frame'],rows[j]['pillar'],int(labels[j]),float(ownerScore[k])] for k,j in enumerate(ix)])
if __name__=='__main__':main()
