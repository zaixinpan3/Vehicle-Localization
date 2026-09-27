"""Export a conservative forest/neighborhood agreement gate, selected on prefixes.

All fitting and the conjunction search exclude Downtown frames after 360.
The suffix is a reused-recording diagnostic, not an independent validation set.
"""
from trainSemanticModels import OUT,P
import json,pickle
import numpy as np
import pandas as pd
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import RobustScaler
from sklearn.neighbors import KNeighborsClassifier


def main():
    t=pd.read_csv(OUT/'downtown_facade_features.csv')
    names=[c for c in t.columns[3:] if not c.startswith(('wide','placement_'))]
    assert len(names)==178,'Expected the whole-pillar and line-group schema'
    t=t[['frame','pillar','finePointCount']+names]
    X=np.floor(t[names].to_numpy()*1e8+.5)/1e8
    y=t.finePointCount.to_numpy()>0;frame=t.frame.to_numpy();dev=frame<=360
    folds=np.minimum(4,((frame-1)*5/360).astype(int))
    def forest_factory():
        return ExtraTreesClassifier(n_estimators=250,max_depth=12,min_samples_leaf=3,
            max_features=.7,class_weight='balanced',n_jobs=2,random_state=927)
    def neighbor_factory():
        return make_pipeline(RobustScaler(quantile_range=(10,90)),
            KNeighborsClassifier(n_neighbors=9,weights='uniform',algorithm='brute',n_jobs=2))
    a=np.full(len(t),-np.inf);b=a.copy()
    for fold in range(5):
        test=dev&(folds==fold);f=frame[test];train=dev&((frame<f.min()-10)|(frame>f.max()+10))
        forest=forest_factory().fit(X[train],y[train]);knn=neighbor_factory().fit(X[train],y[train])
        a[test]=forest.predict_proba(X[test])[:,1];b[test]=knn.predict_proba(X[test])[:,1]
        print('Prefix fold',fold,'complete',flush=True)
    best=None
    for quantile in [.5,.65,.75,.85,.90,.925,.95,.975,.99]:
        threshold=float(np.quantile(a[dev],quantile))
        ix=np.flatnonzero(dev&(a>=threshold));order=ix[np.argsort(-b[ix],kind='stable')]
        n=np.arange(1,len(order)+1);fp=np.cumsum(~y[order]);covered=np.cumsum(t.finePointCount.to_numpy()[order])
        last=np.r_[b[order][:-1]!=b[order][1:],True];valid=(n>=30)&(fp/n<=.05)&last
        for at in np.flatnonzero(valid):
            if best is None or covered[at]>best['coveredPoints']:
                best=dict(selected=int(n[at]),false=int(fp[at]),coveredPoints=int(covered[at]),
                    threshold=threshold,neighborThreshold=float(b[order[at]]))
    assert best is not None,'No prefix operating point'
    print('PREFIX OOF',best,flush=True)
    forest=forest_factory().fit(X[dev],y[dev]);knn=neighbor_factory().fit(X[dev],y[dev])
    result=dict(schemaVersion=1,kind='forestNeighborConsensus',semantic='facade',featureNames=names,
        featureScale=1e8,decisionThreshold=best['threshold'],decisionThresholds={'downtown':best['threshold']},
        roots=[],feature=[],threshold=[],left=[],right=[],value=[],leaf=[],
        neighborCount=9,neighborThreshold=best['neighborThreshold'],
        neighborCenter=knn[0].center_.tolist(),neighborScale=knn[0].scale_.tolist(),
        neighborVectors=knn[-1]._fit_X.tolist(),neighborPositive=y[dev].tolist(),
        training='Downtown <=360; five purged temporal folds; seed 927; original 0.3 m fine labels used only offline')
    for estimator in forest.estimators_:
        tree=estimator.tree_;offset=len(result['feature']);result['roots'].append(offset+1)
        result['feature']+=(tree.feature+1).tolist();result['threshold']+=tree.threshold.tolist()
        result['left']+=(tree.children_left+offset+1).tolist();result['right']+=(tree.children_right+offset+1).tolist()
        values=tree.value[:,0,:];result['value']+=(values[:,1]/values.sum(axis=1)).tolist()
        result['leaf']+=(tree.children_left<0).tolist()
    (OUT/'facade_model.json').write_text(json.dumps(result,separators=(',',':'))+'\n')
    with (OUT/'facade_model.pickle').open('wb') as f:pickle.dump((forest,knn,names),f)
    score=forest.predict_proba(X)[:,1];agreement=knn.predict_proba(X)[:,1]
    score[agreement<best['neighborThreshold']]=0
    prediction=t[['frame','pillar','finePointCount']].copy();prediction.insert(0,'dataset','downtown')
    prediction['score']=score;prediction['oof']=np.where(b>=best['neighborThreshold'],a,0)
    prediction.to_csv(OUT/'facade_predictions.csv',index=False)
    den=pd.read_csv(P/'downtown_baseline.csv');den=den[den.feature=='facade'];validation={}
    for split in ['prefix','suffix','all']:
        mask=np.ones(len(t),bool);base=np.ones(len(den),bool)
        if split=='prefix':mask=dev;base=den.frame<=360
        if split=='suffix':mask=~dev;base=den.frame>360
        take=mask&(score>=best['threshold']);n=int(take.sum());fp=int((take&~y).sum())
        covered=int(t.finePointCount.to_numpy()[take].sum());total=int(den.finePointCountInRoi[base].sum())
        validation[split]=dict(selected=n,false=fp,falseFraction=fp/n if n else None,coveredPoints=covered,
            referencePoints=total,pointCoverage=covered/max(total,1))
    best['referencePoints']=int(den.finePointCountInRoi[den.frame<=360].sum())
    best['pointCoverage']=best['coveredPoints']/best['referencePoints'];best['falseFraction']=best['false']/best['selected']
    summary=json.loads((P/'model_validation.json').read_text())
    summary['facade']=dict(kind=result['kind'],features=len(names),thresholds=result['decisionThresholds'],
        prefixOOF={'downtown':best},validation={'downtown':validation},
        limitation='Very sparse suffix selections; 0 observed false selections does not establish a population false-selection rate below 10%.')
    (P/'model_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('VALIDATION',json.dumps(validation),flush=True)

if __name__=='__main__':main()
