"""Aggregate exact per-frame counts, with explicit temporal-suffix reports."""
from pathlib import Path
import csv,json,statistics,hashlib
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent;OUT=ROOT/'output/semantic_precision_20260927'
limits={'mississippi':780,'downtown':360}
def aggregate(rows):
 keys=['candidatePillarCount','extraPillarCount','finePointCountInRoi','coveredFinePointCount','finePointCountOutsideRoi']
 out={k:sum(int(r[k]) for r in rows) for k in keys}
 out['falseSelectionFraction']=out['extraPillarCount']/out['candidatePillarCount'] if out['candidatePillarCount'] else None
 out['referencePointCoverage']=out['coveredFinePointCount']/out['finePointCountInRoi'] if out['finePointCountInRoi'] else None
 out['frames']=len(rows);out['framesWithReference']=sum(int(r['finePointCountInRoi'])>0 for r in rows)
 out['referenceFramesWithNoSelection']=sum(int(r['finePointCountInRoi'])>0 and int(r['candidatePillarCount'])==0 for r in rows)
 out['framesWithFalseSelection']=sum(int(r['extraPillarCount'])>0 for r in rows)
 return out
result={}
for dataset,limit in limits.items():
 before=list(csv.DictReader((P/f'{dataset}_baseline.csv').open()));after=list(csv.DictReader((P/f'{dataset}_replay.csv').open()));assert len(before)==len(after)
 result[dataset]={}
 for feature in sorted({r['feature'] for r in before}):
  result[dataset][feature]={}
  for split in ['prefix','suffix','all']:
   keep=lambda r:r['feature']==feature and (split=='all' or (int(r['frame'])<=limit if split=='prefix' else int(r['frame'])>limit))
   a=aggregate([r for r in before if keep(r)]);b=aggregate([r for r in after if keep(r)])
   assert a['finePointCountInRoi']==b['finePointCountInRoi']
   result[dataset][feature][split]={'baseline':a,'precision':b}
validation=json.loads((P/'model_validation.json').read_text());summary={'counts':result,'modelSelection':validation,'definition':'A selected pillar is false iff it contains zero original 0.3 m fine reference points of that class. Denominators include every reference point inside the coarse XY ROI.','scope':'Full replay includes fitted prefixes. Temporal suffixes are from reused recordings, not new independent recordings.'}
summary['allFullAndSuffixFalseFractionsBelow10Percent']=all(result[ds][name][split]['precision']['falseSelectionFraction'] is not None and result[ds][name][split]['precision']['falseSelectionFraction']<=.1 for ds in result for name in result[ds] for split in ['all','suffix'])
if (P/'runtime_paired.csv').exists():
 rows=list(csv.DictReader((P/'runtime_paired.csv').open()));summary['runtimeMedianMilliseconds']={}
 for dataset in ['Mississippi','Downtown']:
  summary['runtimeMedianMilliseconds'][dataset]={mode:statistics.median(float(r['milliseconds']) for r in rows if r['dataset']==dataset and r['mode']==mode) for mode in ['baseline','precision']}
if (P/'tests.csv').exists():
 rows=list(csv.DictReader((P/'tests.csv').open()));summary['tests']={k:sum(r[k] in ['1','true'] for r in rows) for k in ['Passed','Failed','Incomplete']}
(P/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
manifest=[]
for pattern in ['*_baseline.mat','*_model.json','*_predictions.csv']:
 for p in OUT.glob(pattern):manifest.append({'file':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(P/'artifact_hashes.json').write_text(json.dumps(manifest,indent=2)+'\n')
for ds in result:
 for feature in result[ds]:
  r=result[ds][feature]['suffix'];print(ds,feature,'suffix',r['baseline']['falseSelectionFraction'],'->',r['precision']['falseSelectionFraction'],'coverage',r['precision']['referencePointCoverage'])
print('All full/suffix rates <=10%:',summary['allFullAndSuffixFalseFractionsBelow10Percent'])
