"""Reproduce matching comparisons from the retained original output folders."""
from pathlib import Path
import csv,json,math,statistics,shutil,hashlib
root=Path(__file__).resolve().parents[2];dest=root/'research/inspva_mapping_rebuild_20260915'
oldroot=root/'output/saved_perception_baseline_20260914';newroot=root/'output/saved_perception_inspva_20260915';maproot=root/'output/mississippi_mapping_inspva_20260915'
def read(p):
 with p.open(newline='') as f:return list(csv.DictReader(f))
def metrics(rows):
 e=[float(x['pvaErrorM']) for x in rows]
 return {'samples':len(e),'rmse_m':math.sqrt(sum(x*x for x in e)/len(e)),'median_m':statistics.median(e),'maximum_m':max(e),'errors_at_least_1m':sum(x>=1 for x in e)}
old=read(oldroot/'calls.csv');new=read(newroot/'calls.csv')
assert len(old)==len(new)==4680
assert [(r['frame'],r['mode']) for r in old]==[(r['frame'],r['mode']) for r in new]
reported=read(newroot/'metrics.csv');comparison={}
for mode in ['per_frame_zero','per_frame_positive','per_frame_negative','lidar_only_recursive']:
 a=[x for x in old if x['mode']==mode];b=[x for x in new if x['mode']==mode]
 fa=[x for x in a if x['fullPose']=='1'];fb=[x for x in b if x['fullPose']=='1']
 shared=set(x['frame'] for x in fa)&set(x['frame'] for x in fb)
 m={'old_full':metrics(fa),'new_full':metrics(fb),'common_full_old':metrics([x for x in fa if x['frame'] in shared]),'common_full_new':metrics([x for x in fb if x['frame'] in shared]),'new_directional':sum(x['directionalPose']=='1' for x in b),'new_rejected':sum(x['measurementType']=='rejected' for x in b),'old_all_outputs':metrics(a),'new_all_outputs':metrics(b)}
 for population,key in [('full_measurements','new_full'),('all_outputs','new_all_outputs')]:
  row=next(x for x in reported if x['mode']==mode and x['population']==population and x['reference']=='INSPVA')
  assert abs(float(row['positionRmseM'])-m[key]['rmse_m'])<1e-9
  assert abs(float(row['positionMedianM'])-m[key]['median_m'])<1e-9
 for row in b:
  if row['measurementType']=='rejected':assert all(math.isnan(float(row[f])) for f in ['measurementX','measurementY','measurementPsi'])
 comparison[mode]=m
frame={}
for name,rows in [('old',old),('new',new)]:
 r=next(x for x in rows if x['frame']=='1086' and x['mode']=='per_frame_zero');frame[name]={'full':r['fullPose']=='1','pva_error_m':float(r['pvaErrorM']),'mapping_error_m':float(r['mappingErrorM'])}
recursive=[x for x in new if x['mode']=='lidar_only_recursive']
failures={str(level):next({'frame':int(x['frame']),'time_s':float(x['time']),'error_m':float(x['pvaErrorM'])} for x in recursive if float(x['pvaErrorM'])>level) for level in [.5,1,5,10]}
result={'comparison':comparison,'frame_1086':frame,'recursive_first_crossings':failures,'recursive_final_error_m':float(recursive[-1]['pvaErrorM']),'validation':'All 4680 ordered calls present; all full/all-output RMSE and median values independently reproduced within 1e-9 m; rejected measurement coordinates remain NaN. Common-accepted comparisons are supplemental, not replacements for full populations.','scope':'All frames retained; no gain or matcher tuning. Same perception points and map hyperparameters; new INSPVA source, time interpolation, grid-north orientation and ellipsoidal height. Both map and queries use this drive; no independent ground truth.'}
(dest/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
for src,name in [(newroot/'metrics.csv','matching_metrics.csv'),(newroot/'summary.json','matching_summary.json'),(maproot/'validation.json','map_validation.json'),(maproot/'map_layer_summary.csv','map_layer_summary.csv')]:(dest/name).write_text(src.read_text())
fields=['frame','time','mode','fullPose','directionalPose','measurementType','reason','pvaErrorM','mappingErrorM','yawErrorDeg','registrationSeconds']
with (dest/'frame_results.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows({k:r[k] for k in fields} for r in new)
manifest=[]
for p in [root/'output/mississippi_perception_video_20260912/feature_observations.mat',root/'output/mississippi_mapping_20260912/probability_cloud.mat',maproot/'feature_observations.mat',maproot/'probability_cloud.mat',newroot/'calls.csv']:
 with p.open('rb') as stream:d=hashlib.file_digest(stream,'sha256').hexdigest()
 manifest.append({'path':str(p.relative_to(root)),'bytes':p.stat().st_size,'sha256':d})
(dest/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(result,indent=2))
