"""Development-only high-precision frontiers with purged temporal folds."""
from pathlib import Path
import csv,json
import numpy as np
from sklearn.ensemble import RandomForestClassifier,ExtraTreesClassifier,GradientBoostingClassifier
P=Path(__file__).resolve().parent;OLD=P.parent/'pole_context_20260927'
def main():
 rows=list(csv.DictReader((OLD/'context_continuity_features.csv').open()))
 d={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
 keys=np.array([(r['frame'],r['pillar']) for r in rows]);_,ix,groups=np.unique(keys,axis=0,return_index=True,return_inverse=True)
 fine=d['finePointCount'][ix];y=d['finePointCount']>0;blocks=np.array_split(np.unique(d['frame']),5)
 base=list(next(csv.DictReader((OLD/'development_features.csv').open())).keys())
 excluded={'frame','pillar','hypothesis','finePointCount','coreMinimumZ','coreMaximumZ'}
 featureSets={'base':[k for k in base if k not in excluded], 'expanded':[k for k in d if k not in excluded and not k.startswith(('contextHeight','maximumContextHeight'))], 'all':[k for k in d if k not in excluded]}
 factories={'gb2':lambda:GradientBoostingClassifier(n_estimators=150,max_depth=2,min_samples_leaf=8,learning_rate=.05,random_state=927), 'gb3':lambda:GradientBoostingClassifier(n_estimators=150,max_depth=3,min_samples_leaf=8,learning_rate=.05,random_state=927), 'rf':lambda:RandomForestClassifier(n_estimators=200,max_depth=7,min_samples_leaf=3,max_features=.7,random_state=927,n_jobs=1), 'extra':lambda:ExtraTreesClassifier(n_estimators=200,max_depth=8,min_samples_leaf=3,max_features=.8,random_state=927,n_jobs=1)}
 output=[]
 for setName,features in featureSets.items():
  X=np.nan_to_num(np.column_stack([d[k] for k in features]),nan=0,posinf=0,neginf=0)
  for modelName,factory in factories.items():
   for weighting in ['balanced','mass']:
    score=np.zeros(len(y))
    for block in blocks:
     test=np.isin(d['frame'],block);train=(d['frame']<min(block)-20)|(d['frame']>max(block)+20)
     w=np.ones(train.sum());v=y[train]
     if weighting=='mass':w[v]=np.sqrt(1+d['finePointCount'][train][v]/50)
     w[v]*=(~v).sum()/w[v].sum();w/=np.bincount(groups[train],minlength=len(fine))[groups[train]]
     model=factory();model.fit(X[train],y[train],sample_weight=w);score[test]=model.predict_proba(X[test])[:,1]
    ownerScore=np.full(len(fine),-np.inf);np.maximum.at(ownerScore,groups,score)
    order=np.argsort(-ownerScore,kind='stable');tp=np.cumsum(fine[order]>0);covered=np.cumsum(fine[order]);n=np.arange(1,len(order)+1);fp=1-tp/n
    valid=(fp<=.1)&np.r_[ownerScore[order][1:]!=ownerScore[order][:-1],True]
    index=np.flatnonzero(valid);best=index[np.argmax(covered[index])] if len(index) else None
    r=dict(features=setName,model=modelName,weighting=weighting,featureCount=len(features),pointCoverage=0,extraFraction=None)
    if best is not None:r.update(pointCoverage=float(covered[best]/13464),extraFraction=float(fp[best]),selected=int(n[best]),matched=int(tp[best]),threshold=float(ownerScore[order][best]))
    output.append(r);print(r,flush=True)
    (P/'development_model_frontiers.json').write_text(json.dumps(output,indent=2)+'\n')
    np.save(P.parent.parent/'output/pole_precision_20260927'/f'probe_{setName}_{modelName}_{weighting}.npy',score)
if __name__=='__main__':main()
