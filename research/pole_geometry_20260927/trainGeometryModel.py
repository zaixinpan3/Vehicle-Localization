"""Select a joint distribution rule using purged prefix predictions only.

The two objectives use exact 0.6 m owner unions, never individual row accuracy.
Temporal suffixes are reused regression checks, not a new-sequence benchmark.
"""
from pathlib import Path
import os, sys, csv, json, pickle
os.environ['OMP_NUM_THREADS']='1'
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pole_precision_20260927'))
from trainPrecisionModel import metrics

P=Path(__file__).resolve().parent;OUT=P.parents[1]/'output/pole_geometry_20260927'
META={'dataset','frame','pillar','hypothesis','finePointCount'}
COMPACT='''supportHeight longestHeight meanRatio massRatio supportedCount tilt radialRms maximumStd aspect isolation acceptedCount
fraction010 fraction015 fraction025 fraction075 coreHeight maximumGap gapVariation
peakFloor15 peakFloor30 anisotropy30 peakShift30 minimumQuarterCount maximumQuarterFraction quarterCenterStep quarterRmsVariation
axisRange structuralFraction circleRelativeResidual transverseViewAlignment std40 aspect40 shift40
ownerCount ownerHeight ownerFraction pointScore lineScore ownerTotalCount ownerTotalHeight ownerCoreFraction ownerSupportFraction
axisOwnerDistance axisOwnerX axisOwnerY ownerVarianceX ownerVarianceY ownerCovarianceXY
whole_height18_60 whole_longest18_60 whole_mean18_60 whole_mass18_60 whole_height25_75 whole_longest25_75
whole_probeFraction45 whole_probePointScore45 whole_probeFraction90
ownerContinuousHeight ownerContextHeightFraction neighborContinuousHeight'''.split()

def load(moments=False,stable=False):
    rows=[];den=[]
    for name in ['mississippi','downtown']:
        rows+=list(csv.DictReader((OUT/f'{name}_features.csv').open()))
        den+=list(csv.DictReader((P.parent/'pole_precision_20260927'/f'{name}_frames.csv').open()))
    names=[k for k in rows[0] if k not in META]
    X=np.nan_to_num(np.array([[float(r[k]) for k in names] for r in rows]),nan=0,posinf=0,neginf=0)
    if moments:
        extra=[]
        for name in ['mississippi','downtown']:extra+=list(csv.DictReader((OUT/f'{name}_moments.csv').open()))
        key=['dataset','frame','pillar','hypothesis'];lookup={tuple(r[k] for k in key):r for r in extra}
        fields=[k for k in extra[0] if k not in key]
        value=np.array([[float(lookup[tuple(r[k] for k in key)][f]) for f in fields] for r in rows])
        X=np.column_stack([X,np.nan_to_num(value,nan=0,posinf=0,neginf=0)]);names+=fields
    if stable:X=np.floor(X*1e8+.5)/1e8
    ds=np.array([r['dataset'] for r in rows]);frames=np.array([int(r['frame']) for r in rows]);labels=np.array([int(r['finePointCount']) for r in rows]);y=labels>0
    keys=np.array([f"{r['dataset']}:{r['frame']}:{r['pillar']}" for r in rows]);_,ix,groups=np.unique(keys,return_index=True,return_inverse=True)
    dev=((ds=='Mississippi')&(frames<=780))|((ds=='Downtown')&(frames<=360))
    return rows,den,names,X,ds,frames,labels,y,ix,groups,dev

def baseline(rows,ix):
    lookup={(r['dataset'].lower(),int(r['frame']),int(r['pillar'])):int(r['current']) for r in csv.DictReader((P/'broad_owner_diagnosis.csv').open())}
    return np.array([lookup.get((rows[j]['dataset'].lower(),int(rows[j]['frame']),int(rows[j]['pillar'])),0) for j in ix])

def select(score,ds,frames,labels,ix,den,targets):
    thresholds=np.unique(score[np.isfinite(score)])
    valid=np.ones(len(thresholds),bool);worst=np.zeros(len(thresholds));minimum=np.ones(len(thresholds))
    at80=np.ones(len(thresholds),bool);at10=at80.copy()
    for dataset,limit in [('Mississippi',780),('Downtown',360)]:
        mask=(ds[ix]==dataset)&(frames[ix]<=limit);order=np.argsort(-score[mask]);v=score[mask][order];fine=labels[ix][mask][order]
        n=np.searchsorted(-v,-thresholds,side='right');tp=np.r_[0,np.cumsum(fine>0)][n];covered=np.r_[0,np.cumsum(fine)][n]
        total=sum(int(r['finePoints']) for r in den if r['dataset']==dataset and int(r['frame'])<=limit)
        fp=(n-tp)/np.maximum(n,1);coverage=covered/total
        valid &= (n>0)&(coverage>=targets[dataset]['pointCoverage']+.002)&(fp<targets[dataset]['extraFraction'])
        at80 &= (n>0)&(coverage>=.8);at10 &= (n>0)&(fp<=.1)
        worst=np.maximum(worst,fp);minimum=np.minimum(minimum,coverage)
    result={}
    for name,mask,objective in [('pareto',valid,worst),('at80',at80,worst),('at10',at10,-minimum)]:
        if np.any(mask):
            choices=np.flatnonzero(mask);i=choices[np.argmin(objective[choices])]
            result[name]=dict(threshold=float(thresholds[i]),worstExtraFraction=float(worst[i]),minimumCoverage=float(minimum[i]),
                metrics=metrics(score,thresholds[i],ix,ds,frames,labels,den,'development'))
        else:result[name]=None
    return result

def main():
    moments='--moments' in sys.argv;stable='--stable' in sys.argv;prefix=('stable_' if stable else '')+('moments_' if moments else '')
    rows,den,names,X,ds,frames,labels,y,ix,groups,dev=load(moments,stable)
    old=baseline(rows,ix);targets=metrics(old,.5,ix,ds,frames,labels,den,'development')
    (P/'baseline_checks.json').write_text(json.dumps({part:metrics(old,.5,ix,ds,frames,labels,den,part) for part in ['development','suffix','all']},indent=2)+'\n')
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
    result=[]
    schemas={'compact':COMPACT,'base':names[:names.index('height12_45')],'full':names}
    if moments:
        additions=names[names.index('moment001'):]
        schemas={'moments_compact':COMPACT+additions,'moments_full':names}
    if stable:schemas={'stable_moments_compact':COMPACT+additions} if moments else {'stable_full':names}
    for schema,fields in schemas.items():
        features=X[:,[names.index(n) for n in fields]]
        for depth in ([5] if stable else [3,5]):
            for power in ([1.0] if stable else [.5,1.0]):
                name=f'{schema}_d{depth}_w{power}'
                if '--resume' in sys.argv and (OUT/f'{name}.pickle').exists():
                    bundle=pickle.load((OUT/f'{name}.pickle').open('rb'));result.append(bundle['result']);continue
                def factory():
                    return HistGradientBoostingClassifier(max_iter=250,max_depth=depth,max_leaf_nodes=31,min_samples_leaf=20,
                        learning_rate=.05,l2_regularization=2,early_stopping=False,random_state=927)
                score=np.full(len(y),-np.inf)
                for fold in range(5):
                    test=dev&(folds==fold);train=dev&~test
                    for dataset,purge in [('Mississippi',20),('Downtown',10)]:
                        fs=frames[test&(ds==dataset)];train &= ~((ds==dataset)&(frames>=fs.min()-purge)&(frames<=fs.max()+purge))
                    model=factory();model.fit(features[train],y[train],sample_weight=weight(train,power));score[test]=model.predict_proba(features[test])[:,1]
                owners=np.full(len(ix),-np.inf);np.maximum.at(owners,groups,score)
                chosen=select(owners,ds,frames,labels,ix,den,targets)
                r=dict(model=name,features=len(fields),**chosen);result.append(r);print(json.dumps(r),flush=True)
                model=factory();model.fit(features[dev],y[dev],sample_weight=weight(dev,power))
                with (OUT/f'{name}.pickle').open('wb') as f:pickle.dump(dict(model=model,names=fields,result=r),f)
                np.save(OUT/f'{name}_oof.npy',score)
                (P/f'{prefix}model_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    (P/f'{prefix}model_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    valid=[r for r in result if r['pareto']]
    if not valid:print('No prefix Pareto improvement');return
    winner=min(valid,key=lambda r:r['pareto']['worstExtraFraction'])
    (P/f'{prefix}model_freeze.json').write_text(json.dumps(dict(winner=winner,seed=927,split='Mississippi <=780; Downtown <=360; five blocks; purge 20/10 frames',
        objective='Minimize worst false selection while both prefix OOF point coverages exceed production by at least 0.2 percentage points'),indent=2)+'\n')
    bundle=pickle.load((OUT/(winner['model']+'.pickle')).open('rb'));model=bundle['model'];features=X[:,[names.index(n) for n in bundle['names']]]
    score=model.predict_proba(features)[:,1];owners=np.full(len(ix),-np.inf);np.maximum.at(owners,groups,score);threshold=winner['pareto']['threshold']
    validation={part:metrics(owners,threshold,ix,ds,frames,labels,den,part) for part in ['development','suffix','all']}
    (P/f'{prefix}model_validation.json').write_text(json.dumps(validation,indent=2)+'\n');print('FROZEN VALIDATION',json.dumps(validation),flush=True)
    with (P/f'{prefix}model_predictions.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['dataset','frame','pillar','finePointCount','score'])
        w.writerows([[rows[j]['dataset'],rows[j]['frame'],rows[j]['pillar'],int(labels[j]),float(owners[k])] for k,j in enumerate(ix)])

if __name__=='__main__':main()
