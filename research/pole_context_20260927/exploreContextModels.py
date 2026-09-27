"""Five purged temporal folds; no frame/pillar IDs or global XY input.

Sensor-relative support heights are features in these rejected model probes.
"""
import csv,json
import numpy as np
from sklearn.ensemble import RandomForestClassifier,GradientBoostingClassifier
from sklearn.tree import DecisionTreeClassifier,export_text
from analyzeContext import load,metric,P

def main():
 d,index,group,fine,den,pden=load()
 excluded={'frame','pillar','hypothesis','finePointCount'}
 features=[k for k in d if k not in excluded]
 X=np.column_stack([d[k] for k in features]);X=np.nan_to_num(X,nan=0,posinf=0,neginf=0)
 y=d['finePointCount']>0
 blocks=np.array_split(np.unique(d['frame']),5)
 models={'tree3':lambda:DecisionTreeClassifier(max_depth=3,min_samples_leaf=10,random_state=927),
         'tree4':lambda:DecisionTreeClassifier(max_depth=4,min_samples_leaf=10,random_state=927),
         'forest':lambda:RandomForestClassifier(n_estimators=100,max_depth=4,min_samples_leaf=5,max_features=.7,random_state=927,n_jobs=1),
         'boost2':lambda:GradientBoostingClassifier(n_estimators=80,max_depth=2,min_samples_leaf=10,learning_rate=.05,random_state=927),
         'boost3':lambda:GradientBoostingClassifier(n_estimators=60,max_depth=3,min_samples_leaf=10,learning_rate=.05,random_state=927)}
 def weights(ids):
  w=np.ones(np.count_nonzero(ids));target=y[ids]
  w[target]=np.sqrt(1+d['finePointCount'][ids][target]/50)
  w[target]*=np.count_nonzero(~target)/w[target].sum()
  # Avoid counting multiple hypotheses for one owner as separate observations.
  countsByOwner=np.bincount(group[ids],minlength=len(fine))
  return w/countsByOwner[group[ids]]
 def frontier(score,target):
  best=None
  for threshold in np.unique(score):
   m=metric(score>=threshold,group,fine,den,pden)
   if m['coverage']>=target and m['recall']>=.8 and (best is None or m['extra']<best['extra']):
    best=dict(threshold=float(threshold),**m)
  return best
 results=[];scores={}
 for name,factory in models.items():
  score=np.zeros(len(y))
  for block in blocks:
   test=np.isin(d['frame'],block);train=(d['frame']<min(block)-20)|(d['frame']>max(block)+20)
   model=factory();model.fit(X[train],y[train],sample_weight=weights(train));score[test]=model.predict_proba(X[test])[:,1]
  scores[name]=score
  full=factory();full.fit(X,y,sample_weight=weights(np.ones(len(y),bool)))
  for target in [.90,.93,.95]:
   f=frontier(score,target)
   if f:results.append(dict(model=name,target=target,**f))
  print(name,[r for r in results if r['model']==name],flush=True)
  if name.startswith('tree'):(P/(name+'.txt')).write_text(export_text(full,feature_names=features,decimals=5))
  imp=sorted(zip(features,full.feature_importances_),key=lambda x:-x[1])
  (P/(name+'_importance.json')).write_text(json.dumps(imp,indent=2)+'\n')
 with (P/'model_comparison.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(results[0]),lineterminator='\n');w.writeheader();w.writerows(results)
 with (P/'model_predictions.csv').open('w') as f:
  w=csv.writer(f,lineterminator='\n');w.writerow(['frame','pillar','finePointCount']+list(scores))
  w.writerows([[int(d['frame'][i]),int(d['pillar'][i]),int(d['finePointCount'][i])]+[float(s[i]) for s in scores.values()] for i in range(len(y))])
 (P/'model_protocol.json').write_text(json.dumps(dict(features=features,folds=[b.tolist() for b in blocks],purge=20,seed=927,results=results),indent=2)+'\n')
if __name__=='__main__':main()
