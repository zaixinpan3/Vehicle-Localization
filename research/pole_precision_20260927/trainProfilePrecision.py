"""Check separate calibrations for the existing semantic-channel profiles.

Mississippi omits facade semantics; Downtown includes them in the frozen fine
reference. Splitting these profiles tests label-policy conflict, not frame ID.
"""
from pathlib import Path
import json,pickle,os,sys
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
from trainContextPrecision import expanded
from trainPrecisionModel import metrics

P=Path(__file__).resolve().parent;OUT=P.parents[1]/'output/pole_precision_20260927'

def main():
    rows,den,names,X,ds,frames,labels,y,ix,groups,development=expanded(True)
    path=P/'profile_model_comparison.json'
    # Resumption is explicit; a normal reproduction always fits fresh models.
    results=json.loads(path.read_text()) if '--resume' in sys.argv and path.exists() else []
    chosen={};validation={}
    for dataset,limit,purge in [('Mississippi',780,20),('Downtown',360,10)]:
        use=development&(ds==dataset);folds=np.minimum(4,((frames-1)*5/limit).astype(int))
        def weights(train):
            w=np.ones(train.sum());positive=y[train];w[positive]=1+labels[train][positive]/20
            w[positive]*=.5/w[positive].sum();w[~positive]=.5/np.count_nonzero(~positive)
            w/=np.bincount(groups[train],minlength=len(ix))[groups[train]]
            return w/w.mean()
        total=sum(int(r['finePoints']) for r in den if r['dataset']==dataset and int(r['frame'])<=limit)
        candidates=[]
        for depth in [3,5]:
            name=f'profile_{dataset.lower()}_d{depth}'
            saved=next((r for r in results if r['model']==name),None)
            if saved:
                with (OUT/f'{name}.pickle').open('rb') as f:bundle=pickle.load(f)
                candidates.append((saved,bundle['model']));continue
            def factory():return HistGradientBoostingClassifier(max_iter=300,max_depth=depth,max_leaf_nodes=31,
                min_samples_leaf=8,learning_rate=.05,l2_regularization=1,early_stopping=False,random_state=927)
            score=np.full(len(y),-np.inf)
            for fold in range(5):
                test=use&(folds==fold);fs=frames[test];train=use&((frames<fs.min()-purge)|(frames>fs.max()+purge))
                model=factory();model.fit(X[train],y[train],sample_weight=weights(train));score[test]=model.predict_proba(X[test])[:,1]
            owners=np.full(len(ix),-np.inf);np.maximum.at(owners,groups,score)
            mask=(ds[ix]==dataset)&(frames[ix]<=limit);fine=labels[ix][mask];sc=owners[mask]
            order=np.argsort(-sc,kind='stable');sc=sc[order];fine=fine[order]
            thresholds=np.unique(sc);n=np.searchsorted(-sc,-thresholds,side='right')
            tp=np.r_[0,np.cumsum(fine>0)][n];covered=np.r_[0,np.cumsum(fine)][n]
            good=np.flatnonzero((n-tp)/n<=.1)
            assert len(good)>0
            best=good[np.argmax(covered[good])];threshold=float(thresholds[best])
            result=dict(dataset=dataset,model=name,threshold=threshold,metrics=metrics(owners,threshold,ix,ds,frames,labels,den,'development')[dataset])
            print(result,flush=True);results.append(result);np.save(OUT/f'{name}_oof.npy',score)
            model=factory();model.fit(X[use],y[use],sample_weight=weights(use));candidates.append((result,model))
            with (OUT/f'{name}.pickle').open('wb') as f:pickle.dump(dict(model=model,names=names,threshold=threshold),f)
            (P/'profile_model_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
        result,model=max(candidates,key=lambda t:t[0]['metrics']['pointCoverage']);chosen[dataset]=(result,model)
    (P/'profile_model_freeze.json').write_text(json.dumps({d:r for d,(r,m) in chosen.items()},indent=2)+'\n')
    for dataset,(result,model) in chosen.items():
        sc=np.full(len(y),-np.inf);mask=ds==dataset;sc[mask]=model.predict_proba(X[mask])[:,1]
        owners=np.full(len(ix),-np.inf);np.maximum.at(owners,groups,sc)
        validation[dataset]={part:metrics(owners,result['threshold'],ix,ds,frames,labels,den,part)[dataset] for part in ['development','suffix','all']}
    (P/'profile_model_validation.json').write_text(json.dumps(validation,indent=2)+'\n');print('VALIDATION',validation,flush=True)

if __name__=='__main__':main()
