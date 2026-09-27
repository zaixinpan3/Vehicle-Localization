"""Report exact-owner metrics, validation partitions and selection transitions."""
from pathlib import Path
import csv,json,sys,hashlib
import numpy as np
from trainGeometryModel import load,P,OUT,baseline
from trainPrecisionModel import metrics

def table(path,rows):
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)

def main():
    rows,den,names,X,ds,frames,labels,y,ix,groups,dev=load(True,True)
    freeze=json.loads((P/'stable_moments_model_freeze.json').read_text());winner=freeze['winner'];threshold=winner['pareto']['threshold']
    oof=np.load(OUT/(winner['model']+'_oof.npy'));scores=np.full(len(ix),-np.inf);np.maximum.at(scores,groups,oof)
    before=baseline(rows,ix);raw=list(csv.DictReader((P/'stable_moments_model_predictions.csv').open()))
    lookup={(r['dataset'],r['frame'],r['pillar']):float(r['score']) for r in raw}
    actual=np.array([lookup[tuple(rows[j][k] for k in ['dataset','frame','pillar'])] for j in ix]);selected=actual>=threshold
    b={part:metrics(before,.5,ix,ds,frames,labels,den,part) for part in ['development','suffix','all']}
    a={'prefixOOF':metrics(scores,threshold,ix,ds,frames,labels,den,'development'),
       'suffix':metrics(actual,threshold,ix,ds,frames,labels,den,'suffix'),'all':metrics(actual,threshold,ix,ds,frames,labels,den,'all')}
    changes=[];examples=[];checks=[]
    for dataset,limit in [('Mississippi',780),('Downtown',360)]:
        for part in ['development','suffix','all']:
            mask=ds[ix]==dataset
            if part!='all':mask &= (frames[ix]<=limit)==(part=='development')
            gained=mask & selected & (before==0);lost=mask & ~selected & (before>0);fine=labels[ix]
            changes.append(dict(dataset=dataset,partition=part,addedPositivePillars=int(np.count_nonzero(fine[gained])),
                removedPositivePillars=int(np.count_nonzero(fine[lost])),addedFalsePillars=int(np.count_nonzero(gained & (fine==0))),
                removedFalsePillars=int(np.count_nonzero(lost & (fine==0))),gainedPoints=int(fine[gained].sum()),lostPoints=int(fine[lost].sum())))
            if part=='suffix':
                for name,eligible in [('restored',gained&(fine>0)),('removed_false',lost&(fine==0)),('lost_positive',lost&(fine>0))]:
                    choices=np.flatnonzero(eligible);choices=choices[np.argsort(-fine[choices],kind='stable')][:3]
                    for j in choices:examples.append(dict(dataset=dataset,frame=int(frames[ix[j]]),pillar=int(rows[ix[j]]['pillar']),
                        change=name,finePointCount=int(fine[j]),score=float(actual[j])))
        for part in ['prefixOOF','suffix','all']:
            checks.append(dict(dataset=dataset,partition=part,variant='previous',**b['development' if part=='prefixOOF' else part][dataset]))
            checks.append(dict(dataset=dataset,partition=part,variant='joint_distribution',**a[part][dataset]))
        replay=list(csv.DictReader((P/f'stable_moments_{dataset.lower()}_replay.csv').open()))
        current=a['all'][dataset]
        assert sum(int(r['extraPillarCount']) for r in replay)==current['extra']
        assert sum(int(r['coveredFinePointCount']) for r in replay)==current['coveredPoints']
        assert sum(int(r['candidatePillarCount']) for r in replay)==current['selected']
        assert all(r['nonPoleComponentsUnchanged']=='1' for r in replay)
        baseReplay=list(csv.DictReader((P/f'{dataset.lower()}_baseline_replay.csv').open()))
        assert sum(int(r['extraPillarCount']) for r in baseReplay)==b['all'][dataset]['extra']
        assert sum(int(r['coveredFinePointCount']) for r in baseReplay)==b['all'][dataset]['coveredPoints']
    table(P/'acceptance_checks.csv',checks);table(P/'selection_transitions.csv',changes);table(P/'temporal_examples.csv',examples)
    runtime=list(csv.DictReader((P/'paired_runtime.csv').open()));timing={}
    for dataset in ['Mississippi','Downtown']:
        timing[dataset]={}
        for variant,label in [('1','previous'),('2','jointDistribution')]:
            values=np.array([float(r['milliseconds']) for r in runtime if r['dataset']==dataset and r['variant']==variant])
            timing[dataset][label]=dict(samples=len(values),medianMs=float(np.median(values)),meanMs=float(values.mean()))
    tests=list(csv.DictReader((P/'tests.csv').open()))
    summary=dict(model=winner['model'],threshold=threshold,featureCount=143,trees=250,featureScale=1e8,
        reference='Original offline 0.3 m fine pole points; exact any-point ownership in 0.6 m pillars; fixed denominator',
        baseline=b,actual=a,transitions=changes,runtime=timing,
        validation=dict(passed=sum(r['Passed']=='1' for r in tests),failed=sum(r['Failed']=='1' for r in tests),
            incomplete=sum(r['Incomplete']=='1' for r in tests),rawReplayFrames=1298,
            inference=json.loads((P/'stable_moments_inference_checks.json').read_text())),
        jointTargetMet=False,limitations='Full replay contains training prefixes. Prefix OOF and reused temporal suffixes improve both objectives, but <=10% false selection is not established. No independent new recording or manual object truth is claimed.')
    (P/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(10,4.2),constrained_layout=True)
    for ax,(dataset,limit) in zip(axes,[('Mississippi',780),('Downtown',360)]):
        mask=(ds[ix]==dataset)&(frames[ix]<=limit);v=scores[mask];fine=labels[ix][mask]
        order=np.argsort(-v);v=v[order];fine=fine[order];ends=np.r_[v[1:]!=v[:-1],True]
        count=np.arange(1,len(v)+1)[ends];tp=np.cumsum(fine>0)[ends];covered=np.cumsum(fine)[ends]
        total=sum(int(r['finePoints']) for r in den if r['dataset']==dataset and int(r['frame'])<=limit)
        ax.fill_between([0,10],[80,80],[100,100],color='#c9e8ca',alpha=.7,label='Requested region')
        ax.plot(100*(count-tp)/count,100*covered/total,lw=1.7,label='Prefix out-of-fold')
        base=b['development'][dataset];new=a['prefixOOF'][dataset]
        ax.scatter(100*base['extraFraction'],100*base['pointCoverage'],marker='s',s=55,label='Previous default')
        ax.scatter(100*new['extraFraction'],100*new['pointCoverage'],marker='*',s=130,label='Selected setting')
        ax.set(title=dataset,xlabel='False selected pillars (%)',ylabel='Reference point coverage (%)',xlim=(0,80),ylim=(0,100));ax.grid(alpha=.2)
    axes[0].legend(fontsize=8,loc='lower right')
    fig.savefig(P/'precision_coverage.png',dpi=160);fig.savefig(P/'precision_coverage.pdf');plt.close(fig)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
