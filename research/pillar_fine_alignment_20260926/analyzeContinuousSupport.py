"""Evaluate continuous support rules on the development partition only."""
import csv
import json
import argparse
from pathlib import Path
import numpy as np

FOLDER=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--label',default='continuous_support')
    args=parser.parse_args()
    rows=list(csv.DictReader((FOLDER/(args.label+'_hypotheses.csv')).open()))
    assert all(float(r['frame'])<=780 for r in rows)
    d={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
    short=d['supportHeight']<=2
    wide=(d['maximumStd']>np.where(short,.09,.12)) & (d['aspect']>3)
    dense=(d['coreCount']>=100)&(d['densityContrast']>=10)&(d['tilt']<=1)&(d['radialRms']<=.06)
    base=(d['supportHeight']>=1.5)&(d['meanRatio']>=.70)&(d['massRatio']>=.70)&(d['tilt']<=20)&~wide
    base &= ~short|((d['longestSupportedHeight']>=1.5)&(d['tilt']<=6)&(d['radialRms']<=.10))
    base &= ~((d['meanRatio']<.8)|short)|(d['isolation']>=.8)|dense
    base &= d['acceptedCount']>=6
    gt=list(csv.DictReader((FOLDER/'candidate_features_frames.csv').open()))
    total_points=sum(int(r['finePointCount']) for r in gt if int(r['frame'])<=780)
    total_pillars=sum(int(r['finePillarCount']) for r in gt if int(r['frame'])<=780)
    def metric(keep):
        chosen={}
        for i in np.flatnonzero(keep):chosen[(int(d['frame'][i]),int(d['ownerId'][i]))]=int(d['finePointCount'][i])
        return {'selected':len(chosen),'matched':sum(v>0 for v in chosen.values()),'coveredPoints':sum(chosen.values()),
                'precision':sum(v>0 for v in chosen.values())/max(len(chosen),1),
                'pillarRecall':sum(v>0 for v in chosen.values())/total_pillars,'pointCoverage':sum(chosen.values())/total_points}
    output={}
    for radius in [.20,.25,.30,.35,0]:
        for own in [1,3,6]:
            keep=base&(d['ownerCount']>=own)&((d['radius']==radius)|(radius==0))
            output[f'radius{radius}_owner{own}']=metric(keep)
    (FOLDER/(args.label+'_initial.json')).write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))

if __name__=='__main__':main()
