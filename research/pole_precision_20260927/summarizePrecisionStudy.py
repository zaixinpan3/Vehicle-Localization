"""Summarize genuine validation separately from training-fit replay."""
from pathlib import Path
import os
os.environ.setdefault('OMP_NUM_THREADS','1')
import csv,json,hashlib,platform
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sklearn
from trainPrecisionModel import load

P=Path(__file__).resolve().parent;ROOT=P.parents[1];OUT=ROOT/'output/pole_precision_20260927'

def baseline(path,limit=None):
    rows=list(csv.DictReader(path.open()))
    if limit is not None:rows=[r for r in rows if int(r['frame'])<=limit]
    def total(name):return sum(int(r[name]) for r in rows)
    n=total('candidatePillarCount');tp=total('coveredFinePillarCount');covered=total('coveredFinePointCount');points=total('finePointCountInRoi')
    return dict(frames=len(rows),selected=n,matched=tp,extra=n-tp,extraFraction=(n-tp)/n,
        coveredPoints=covered,finePoints=points,pointCoverage=covered/points,pillarRecall=tp/total('finePillarCount'))

def main():
    records=[];families={}
    for prefix in ['context','whole']:
        freeze=json.loads((P/f'{prefix}_model_freeze.json').read_text());validation=json.loads((P/f'{prefix}_model_validation.json').read_text())
        winner=freeze['winner'];families[prefix]=dict(model=winner['model'],threshold=winner['best']['threshold'])
        for ds,m in winner['best']['metrics'].items():records.append(dict(family=prefix,model=winner['model'],dataset=ds,split='prefix_oof',**m))
        for split,data in validation.items():
            for ds,m in data.items():records.append(dict(family=prefix,model=winner['model'],dataset=ds,
                split={'development':'prefix_training_fit','suffix':'temporal_suffix','all':'all_mixed_fit_and_suffix'}[split],**m))
    profiles=json.loads((P/'profile_model_freeze.json').read_text());validation=json.loads((P/'profile_model_validation.json').read_text())
    for ds,winner in profiles.items():
        records.append(dict(family='profile',model=winner['model'],dataset=ds,split='prefix_oof',**winner['metrics']))
        for split,m in validation[ds].items():records.append(dict(family='profile',model=winner['model'],dataset=ds,
            split={'development':'prefix_training_fit','suffix':'temporal_suffix','all':'all_mixed_fit_and_suffix'}[split],**m))
    for r in records:r['passesBothTargets']=r['extraFraction']<=.10 and r['pointCoverage']>=.80
    with (P/'acceptance_checks.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]),lineterminator='\n');w.writeheader();w.writerows(records)
    old=ROOT/'research/pole_context_20260927'
    a=list(csv.DictReader((old/'development_full_frames.csv').open()));b=list(csv.DictReader((old/'temporal_frames.csv').open()))
    path=OUT/'mississippi_production_baseline.csv'
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(a[0]),lineterminator='\n');w.writeheader();w.writerows(a+b)
    base={'Mississippi':baseline(path),'Downtown':baseline(P/'downtown_production_baseline.csv')}
    _,den,_,_,ds,frames,labels,_,ix,groups,_=load()
    fig,axes=plt.subplots(1,2,figsize=(10.8,4.1),constrained_layout=True)
    for ax,dataset,limit in zip(axes,['Mississippi','Downtown'],[780,360]):
        choices=[('Local + placement',families['context']['model']),('Whole height',families['whole']['model']),('Separate profile',profiles[dataset]['model'])]
        for title,name in choices:
            score=np.load(OUT/f'{name}_oof.npy');owner=np.full(len(ix),-np.inf);np.maximum.at(owner,groups,score)
            mask=(ds[ix]==dataset)&(frames[ix]<=limit);v=owner[mask];fine=labels[ix][mask]
            order=np.argsort(-v,kind='stable');v=v[order];fine=fine[order]
            n=np.searchsorted(-v,-np.unique(v)[::-1],side='right');tp=np.r_[0,np.cumsum(fine>0)][n];cover=np.r_[0,np.cumsum(fine)][n]
            total=sum(int(r['finePoints']) for r in den if r['dataset']==dataset and int(r['frame'])<=limit)
            ax.plot(100*(n-tp)/n,100*cover/total,label=title,linewidth=1.4)
        ax.axvspan(0,10,ymin=.8,ymax=1,color='#75b798',alpha=.2)
        ax.axvline(10,color='#b22222',linestyle='--',linewidth=1);ax.axhline(80,color='#b22222',linestyle='--',linewidth=1)
        ax.set(title=dataset,xlabel='Zero-reference-point pillars / selected pillars (%)',ylabel='Reference pole point coverage (%)',xlim=(0,65),ylim=(0,100))
        ax.grid(alpha=.2);ax.legend(loc='lower right',fontsize=8)
    fig.suptitle('Purged temporal cross-validation on development prefixes')
    fig.savefig(P/'precision_coverage_frontier.pdf');fig.savefig(P/'precision_coverage_frontier.png',dpi=170);plt.close(fig)
    summary=dict(status='Target not achieved; production algorithm unchanged',requirements=dict(maximumFalseSelectionFraction=.10,minimumReferencePointCoverage=.80),
        definition='A selected 0.6 m pillar is false iff it contains zero original offline 0.3 m fine pole points. One point is sufficient; exact owner projection; no spatial tolerance.',
        productionCommit='c4d7c864f3e4edf8efe489fa50ca0ee02b12057f',referenceCommit='35cdb88388300ba1b8bb215905435bde670dff03',
        productionBaseline=base,selectedCommonCandidate=families['context'],checks=records,
        fullCapture=dict(MississippiFrames=1170,DowntownFrames=128,MississippiOwners=5784,DowntownOwners=1611,hypothesisOwnerRows=8710),
        primaryModels=6,localContextModels=12,wholeContextModels=6,profileModels=4,
        scope='Research prototype, exported models, original fine-stage diagnostics and reproducible measurements only. No default threshold change.',
        validationLimit='Temporal suffixes have been reused in prior experiments. They are regression checks, not an independent new-sequence benchmark. Training-fit and mixed replay are not acceptance evidence.',
        environment=dict(python=platform.python_version(),sklearn=sklearn.__version__),
        inference={p:json.loads((P/f'{p}_inference_validation.json').read_text()) for p in ['context','whole']})
    (P/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    files=[f for f in P.iterdir() if f.is_file() and f.name not in {'artifact_hashes.csv'}]
    with (P/'artifact_hashes.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['file','sha256'])
        w.writerows((str(p.relative_to(ROOT)),hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(files))
    print(json.dumps(dict(productionBaseline=base,targetAchieved=False),indent=2))

if __name__=='__main__':main()
