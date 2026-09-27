"""Compare physically constrained continuous support gates on development only."""
import argparse
import csv
import itertools
import json
from pathlib import Path
import numpy as np

FOLDER=Path(__file__).resolve().parent

def base_gate(d):
    short=d['supportHeight']<=2
    wide=(d['maximumStd']>np.where(short,.09,.12))&(d['aspect']>3)
    dense=(d['coreCount']>=100)&(d['densityContrast']>=10)&(d['tilt']<=1)&(d['radialRms']<=.06)
    gate=(d['supportHeight']>=1.5)&(d['meanRatio']>=.7)&(d['massRatio']>=.7)&(d['tilt']<=20)&~wide
    gate &= ~short|((d['longestSupportedHeight']>=1.5)&(d['tilt']<=6)&(d['radialRms']<=.1))
    gate &= ~((d['meanRatio']<.8)|short)|(d['isolation']>=.8)|dense
    return gate&(d['acceptedCount']>=6)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--labels',nargs='*',default=['continuous_structural','connected_density']+
                        [f'context{r}_count{n}' for r in [45,60,75] for n in [3,4]])
    args=parser.parse_args();best={};per_variant={}
    frames=list(csv.DictReader((FOLDER/'candidate_features_frames.csv').open()))
    total_points=sum(int(r['finePointCount']) for r in frames if int(r['frame'])<=780)
    total_pillars=sum(int(r['finePillarCount']) for r in frames if int(r['frame'])<=780)
    controls={'radius':[.20,.25,.30,.35,0],'minimumOwnerCount':[3,6,10,15],
              'minimumOwnerFraction':[0,.10,.20],'minimumHeight':[1.5,1.75,2.0],
              'minimumIsolation':[0,.6,.8],'maximumTilt':[6,10,20],
              'maximumRms':[.10,.12,.20],'minimumPointScore':[0,.7,.8]}
    for label in args.labels:
        rows=list(csv.DictReader((FOLDER/(label+'_hypotheses.csv')).open()))
        assert all(float(r['frame'])<=780 for r in rows)
        d={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
        keys,first,inv=np.unique(np.c_[d['frame'],d['ownerId']],axis=0,return_index=True,return_inverse=True)
        w=d['finePointCount'][first];base=base_gate(d);local={}
        prepared=[]
        tests={'radius':lambda v:(d['radius']==v)|(v==0),
               'minimumOwnerCount':lambda v:d['ownerCount']>=v,
               'minimumOwnerFraction':lambda v:d['ownerCount']>=v*d['acceptedCount'],
               'minimumHeight':lambda v:d['supportHeight']>=v,
               'minimumIsolation':lambda v:d['isolation']>=v,
               'maximumTilt':lambda v:d['tilt']<=v,
               'maximumRms':lambda v:d['radialRms']<=v,
               'minimumPointScore':lambda v:d['pointScore']>=v}
        for name,values in controls.items():prepared.append([(v,tests[name](v)) for v in values])
        for combination in itertools.product(*prepared):
            keep=base.copy()
            for _,mask in combination:keep &=mask
            selected=np.bincount(inv,weights=keep,minlength=len(keys))>0
            matched=int(np.count_nonzero(selected&(w>0)));covered=int(w[selected].sum())
            coverage=covered/total_points;recall=matched/total_pillars
            if coverage<.90 or recall<.80:continue
            count=int(selected.sum());extra=count-matched
            row={'variant':label,**dict(zip(controls,[v for v,_ in combination])),
                 'selected':count,'matched':matched,'extra':extra,'coveredPoints':covered,
                 'pointCoverage':coverage,'pillarRecall':recall,'precision':matched/max(count,1)}
            for target in [.90,.95,.97]:
                key=str(target)
                if coverage>=target and (key not in local or (extra,-covered)<(local[key]['extra'],-local[key]['coveredPoints'])):local[key]=row
                if coverage>=target and (key not in best or (extra,-covered)<(best[key]['extra'],-best[key]['coveredPoints'])):best[key]=row
        per_variant[label]=local
        print(label,json.dumps(local.get('.95',local.get('0.95',{}))),flush=True)
    output={'partition':'Development frames <=780 only','minimumPillarRecall':.80,
            'finePointDenominator':total_points,'finePillarDenominator':total_pillars,
            'controls':controls,'best':best,'perVariant':per_variant,
            'limitation':'Development tuning, not validation; temporal and Downtown holdouts not used.'}
    (FOLDER/'continuous_controls_sweep.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(best,indent=2))

if __name__=='__main__':main()
