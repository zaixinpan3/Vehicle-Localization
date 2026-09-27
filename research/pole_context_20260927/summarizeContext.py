"""Aggregate exact original-fine coverage and extra-owner counts."""
import csv,json,statistics
from pathlib import Path
P=Path(__file__).resolve().parent
OLD=P.parent/'pillar_fine_alignment_20260926'
def read(p):
 with p.open() as f:return list(csv.DictReader(f))
def summarize(rows):
 keys=['finePointCountInRoi','coveredFinePointCount','finePillarCount','coveredFinePillarCount','candidatePillarCount']
 v={k:int(sum(float(x[k]) for x in rows)) for k in keys}
 fp=v['finePointCountInRoi'];covered=v['coveredFinePointCount'];target=v['finePillarCount'];tp=v['coveredFinePillarCount'];selected=v['candidatePillarCount']
 return dict(frames=len(rows),finePoints=fp,coveredPoints=covered,pointCoverage=covered/fp,pointMissRate=1-covered/fp,
             targetPillars=target,matchedPillars=tp,pillarRecall=tp/target,selectedPillars=selected,extraPillars=selected-tp,
             extraFraction=(selected-tp)/selected,pillarPrecision=tp/selected)
def main():
 dev=read(P/'development_full_frames.csv');temporal=read(P/'temporal_frames.csv');new=dev+temporal
 assert [int(x['frame']) for x in new]==list(range(1,1171))
 with (P/'full_frames.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(new[0]),lineterminator='\n');w.writeheader();w.writerows(new)
 old=read(OLD/'final_full_frames.csv');dt=read(P/'downtown_frames.csv');olddt=read(OLD/'final_downtown_frames.csv')
 results={}
 for name,a,b in [('Mississippi_full',old,new),('Mississippi_development',old[:780],dev),('Mississippi_temporal',old[780:],temporal),('Downtown',olddt,dt)]:
  a,b=summarize(a),summarize(b)
  results[name]=dict(baseline=a,current=b,extraCountReduction=1-b['extraPillars']/a['extraPillars'],pointCoverageChange=b['pointCoverage']-a['pointCoverage'],pillarRecallChange=b['pillarRecall']-a['pillarRecall'])
 timing={}
 if (P/'paired_runtime.csv').exists():
  r=read(P/'paired_runtime.csv')
  for ds in ['Mississippi','Downtown']:
   timing[ds]={}
   for v in [1,2]:
    samples=[float(x['milliseconds']) for x in r if x['dataset']==ds and int(x['variant'])==v]
    timing[ds][str(v)]=dict(n=len(samples),medianMs=statistics.median(samples),meanMs=statistics.mean(samples))
 tests={}
 if (P/'tests.csv').exists():
  t=read(P/'tests.csv');tests={k:sum(int(x[k]) for x in t) for k in ['Passed','Failed','Incomplete']}
 out=dict(reference='Frozen original offline 0.3 m fine point identities; exact 0.6 m owner projection',baselineCommit='cb8c76b80ec22e0c6daca7455e348eb7e8e8e64a',results=results,timing=timing,tests=tests)
 (P/'summary.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':main()
