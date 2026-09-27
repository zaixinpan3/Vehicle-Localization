"""Test continuous vertical context and full-owner placement statistics.

Only prefix folds select models and thresholds. Temporal suffixes are a reused
regression set, not an independent unseen benchmark. No analytic bypass.
"""
from pathlib import Path
import csv, json, pickle, hashlib, os, sys
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
from trainPrecisionModel import load, metrics

P=Path(__file__).resolve().parent
OUT=P.parents[1]/'output/pole_precision_20260927'
KEY=['dataset','frame','pillar','hypothesis']

def expanded(includeWhole=False):
    rows,den,names,X,ds,frames,labels,y,ix,groups,development=load()
    keys=[tuple(r[k] for k in KEY) for r in rows]
    for kind in ['vertical','placement']+(['whole'] if includeWhole else []):
        extra=[]
        for dataset in ['mississippi','downtown']:
            extra+=list(csv.DictReader((P/f'{dataset}_{kind}_features.csv').open()))
        fields=[n for n in extra[0] if n not in KEY]
        lookup={tuple(r[k] for k in KEY):r for r in extra}
        assert len(lookup)==len(rows) and set(lookup)==set(keys)
        value=np.array([[float(lookup[k][f]) for f in fields] for k in keys])
        X=np.column_stack([X,np.nan_to_num(value,nan=0,posinf=0,neginf=0)])
        names+=fields
    return rows,den,names,X,ds,frames,labels,y,ix,groups,development

def frontier(score,ds,frames,labels,ix,den):
    thresholds=np.unique(score[np.isfinite(score)])
    valid=np.ones(len(thresholds),bool);minimum=np.ones(len(thresholds));worst=np.zeros(len(thresholds))
    for dataset,limit in [('Mississippi',780),('Downtown',360)]:
        mask=(ds[ix]==dataset)&(frames[ix]<=limit)
        order=np.argsort(-score[mask]);v=score[mask][order];fine=labels[ix][mask][order]
        n=np.searchsorted(-v,-thresholds,side='right')
        tp=np.r_[0,np.cumsum(fine>0)][n];covered=np.r_[0,np.cumsum(fine)][n]
        total=sum(int(r['finePoints']) for r in den if r['dataset']==dataset and int(r['frame'])<=limit)
        fp=(n-tp)/np.maximum(n,1)
        valid &= (n>0)&(fp<=.1);minimum=np.minimum(minimum,covered/total);worst=np.maximum(worst,fp)
    best=None;at80=None
    if np.any(valid):
        choices=np.flatnonzero(valid);i=choices[np.argmax(minimum[choices])]
        best=dict(threshold=float(thresholds[i]),minimumCoverage=float(minimum[i]),metrics=metrics(score,thresholds[i],ix,ds,frames,labels,den,'development'))
    if np.any(minimum>=.80):
        choices=np.flatnonzero(minimum>=.80);i=choices[np.argmin(worst[choices])]
        at80=dict(threshold=float(thresholds[i]),worstExtraFraction=float(worst[i]),metrics=metrics(score,thresholds[i],ix,ds,frames,labels,den,'development'))
    return best,at80

def main():
    whole='--whole' in sys.argv;prefix='whole' if whole else 'context'
    rows,den,names,X,ds,frames,labels,y,ix,groups,development=expanded(whole)
    folds=np.zeros(len(y),int)
    for dataset,limit in [('Mississippi',780),('Downtown',360)]:
        mask=ds==dataset;folds[mask]=np.minimum(4,((frames[mask]-1)*5/limit).astype(int))
    def weight(train,power):
        w=np.ones(train.sum());data=ds[train];target=y[train]
        for dataset in np.unique(data):
            positive=(data==dataset)&target;negative=(data==dataset)&~target
            w[positive]=(1+labels[train][positive]/20)**power
            w[positive]*=.5/w[positive].sum();w[negative]=.5/negative.sum()
        w/=np.bincount(groups[train],minlength=len(ix))[groups[train]]
        return w/w.mean()
    results=[];models={}
    for placement in ([True] if whole else [False,True]):
        fields=np.arange(len(names)) if placement else np.arange(names.index('axisOwnerX'))
        features=X[:,fields]
        for depth in [3,5,8]:
            for power in [.5,1.0]:
                name=f'{prefix}_p{int(placement)}_d{depth}_w{power}'
                def factory():
                    return HistGradientBoostingClassifier(max_iter=300,max_depth=depth,max_leaf_nodes=31,
                        min_samples_leaf=15,learning_rate=.05,l2_regularization=1,early_stopping=False,random_state=927)
                score=np.full(len(y),-np.inf)
                for fold in range(5):
                    test=development&(folds==fold);train=development&~test
                    for dataset,purge in [('Mississippi',20),('Downtown',10)]:
                        fs=frames[test&(ds==dataset)];train &= ~((ds==dataset)&(frames>=fs.min()-purge)&(frames<=fs.max()+purge))
                    model=factory();model.fit(features[train],y[train],sample_weight=weight(train,power));score[test]=model.predict_proba(features[test])[:,1]
                owners=np.full(len(ix),-np.inf);np.maximum.at(owners,groups,score)
                best,at80=frontier(owners,ds,frames,labels,ix,den)
                result=dict(model=name,best=best,at80=at80);results.append(result);print(name,best,'AT80',at80,flush=True)
                model=factory();model.fit(features[development],y[development],sample_weight=weight(development,power))
                models[name]=(model,fields,best)
                with (OUT/f'{name}.pickle').open('wb') as f:pickle.dump(dict(model=model,names=[names[i] for i in fields],threshold=best['threshold'] if best else None),f)
                np.save(OUT/f'{name}_oof.npy',score)
                (P/f'{prefix}_model_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
    winner=max((r for r in results if r['best']),key=lambda r:r['best']['minimumCoverage'])
    frozen=dict(winner=winner,inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in P.glob('*_features.csv')},
        split='Mississippi <=780 and Downtown <=360; 5 proportional blocked folds; purge 20/10 frames; seed 927',
        exclusions='No dataset, frame, pillar, hypothesis identity or global XY coordinate; no 0.3 m online partition')
    (P/f'{prefix}_model_freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    model,fields,best=models[winner['model']];score=model.predict_proba(X[:,fields])[:,1]
    owners=np.full(len(ix),-np.inf);np.maximum.at(owners,groups,score)
    validation={part:metrics(owners,best['threshold'],ix,ds,frames,labels,den,part) for part in ['development','suffix','all']}
    (P/f'{prefix}_model_validation.json').write_text(json.dumps(validation,indent=2)+'\n');print('VALIDATION',validation,flush=True)
    with (P/f'{prefix}_model_predictions.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['dataset','frame','pillar','finePointCount','score'])
        w.writerows([[rows[j]['dataset'],rows[j]['frame'],rows[j]['pillar'],int(labels[j]),float(owners[k])] for k,j in enumerate(ix)])

if __name__=='__main__':main()
