"""Compare interpretable one-variable gates on prefix data only."""
from trainSemanticModels import ROOT,P,OUT,LIMITS
import pandas as pd
import numpy as np

def main():
 rows=[]
 for semantic,fields in {'curb':['energy_total','energy_totalBase','energy_heightStepMeters','ground_roughnessMap','energy_linearity'],'facade':['planeDistance','normalVariance','height','lineScore'],'trafficSign':['intensityMax','count','height']}.items():
  ts=[]
  for dataset in LIMITS:
   file=OUT/f'{dataset}_{semantic}_features.csv'
   if not file.exists():continue
   t=pd.read_csv(file);t=t[t.frame<=LIMITS[dataset]];t['dataset']=dataset;ts.append(t)
  t=pd.concat(ts,ignore_index=True)
  for field in fields:
   for direction in ['above','below']:
    best=None
    for threshold in np.unique(np.quantile(t[field],np.linspace(0,1,251))):
     take=(t[field]>=threshold) if direction=='above' else (t[field]<=threshold)
     metrics=[]
     for dataset in t.dataset.unique():
      a=t[take & (t.dataset==dataset)];n=len(a);fp=int((a.finePointCount==0).sum());covered=int(a.finePointCount.sum());metrics.append((n,fp,covered))
     if all(n>=30 and fp/n<=.05 for n,fp,c in metrics):
      objective=sum(c for n,fp,c in metrics)
      if best is None or objective>best[0]:best=(objective,float(threshold),metrics)
    rows.append(dict(semantic=semantic,feature=field,direction=direction,passes=best is not None,threshold=None if best is None else best[1],coveredPoints=0 if best is None else best[0]))
 pd.DataFrame(rows).to_csv(P/'single_feature_gates.csv',index=False)
if __name__=='__main__':main()
