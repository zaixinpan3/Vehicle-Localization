"""Diagnostic weak-label classifier; never called by production perception.

Run from the repository root with scikit-learn 1.9.1 and NumPy 2.5.3.
First run captureWholePillarDescriptors in MATLAB. Train frames are 1 mod 4;
validation frames are 3 mod 4; the final test set contains even frames.
Metrics use only teacher-positive cells present in the candidate table
(n >= 6, height >= 1 m): 8996 of 9129 projected baseline positives.
Thus these results do not replace the full-sequence acceptance calculation.
"""
import numpy as np,json
from sklearn.ensemble import HistGradientBoostingClassifier
from threadpoolctl import threadpool_limits
file='output/coarse_lattice_20260924/whole_pillar_descriptors_all.csv'
with open(file) as h:names=h.readline().strip().split(',')
A=np.loadtxt(file,delimiter=',',skiprows=1,dtype=np.float32);f=A[:,0].astype(int);ids=A[:,1].astype(int);base=A[:,2]>0
print('rows',len(A),'positive',sum(base),flush=True)
key=(f-1)*10000+ids; lookup=np.full(1170001*10,-1,np.int32);lookup[key]=np.arange(len(A));bkeys=key[base]
ref=np.zeros(len(lookup),bool);ref[bkeys]=True;near=np.zeros(len(A),bool);nr=[]
row=(ids-1)%100;col=(ids-1)//100
for r in [-1,0,1]:
 for c in [-1,0,1]:
  valid=(row+r>=0)&(row+r<100)&(col+c>=0)&(col+c<100);kk=np.clip(key+r+100*c,0,len(ref)-1)
  near|=valid&ref[kk]; q=lookup[kk[base]];q[~valid[base]]=-1;nr.append(q)
nidx=np.array(nr).T;nidx[nidx<0]=len(A)
X=A[:,3:];X[~np.isfinite(X)]=np.nan;train=f%4==1;val=f%4==3;test=f%2==0

def score(p,m):
 pp=p&m;pr=np.sum(pp&near)/max(np.sum(pp),1);rc=np.mean(np.r_[p,False][nidx[m[base]]].any(1));return [float(2*pr*rc/max(pr+rc,1e-15)),float(pr),float(rc),int(np.sum(pp))]
with threadpool_limits(limits=4):
 model=HistGradientBoostingClassifier(max_iter=250,max_leaf_nodes=31,min_samples_leaf=25,l2_regularization=5,random_state=42,class_weight={False:1,True:10},early_stopping=False)
 model.fit(X[train],base[train]);prob=model.predict_proba(X)[:,1]
results=[]
for th in np.arange(.2,.91,.025):
 sv=score(prob>th,val);results.append((min(sv[1:3]),float(th),sv))
best=max(results); threshold=best[1]
out=dict(rows=len(A),positives=int(sum(base)),features=names[3:],threshold=threshold,train=score(prob>threshold,train),validation=best[2],test=score(prob>threshold,test))
print(json.dumps(out,indent=2),flush=True)
np.savez('output/coarse_lattice_20260924/whole_pillar_probe_predictions.npz',frame=f,cell=ids,probability=prob)
with open('output/coarse_lattice_20260924/whole_pillar_probe_summary.json','w') as h:json.dump(out,h,indent=2)
