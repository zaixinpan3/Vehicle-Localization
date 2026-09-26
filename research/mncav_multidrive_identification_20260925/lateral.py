#!/usr/bin/env python3
"""Compare physical and stable empirical models on fixed multi-recording splits."""
import json,sys
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from numba import njit
from prepare import ROOT,DEST,OUT
sys.path.insert(0,str(ROOT/'research/mncav_identification_20260925'))
import identifyMncav as base
L=base.LF+base.LR
TAUS=np.array([0.,.04,.1,.25,.5,1.,2.])

@njit(cache=True)
def lowpass(u,tau):
    y=u.copy()
    if tau>0:
        decay=np.exp(-.02/tau)
        for k in range(1,len(u)):y[k]=decay*y[k-1]+(1-decay)*u[k]
    return y

@njit(cache=True)
def empirical(p,v,sw,initial):
    ratio,understeer,tau,zero=p
    target=v*(sw-zero)/(ratio*(L+understeer*v*v));y=target.copy();y[0]=initial
    decay=np.exp(-.02/tau)
    for k in range(1,len(v)):y[k]=decay*y[k-1]+(1-decay)*(target[k]+target[k-1])/2
    return y

def segments():
    out=[]
    for row in pd.read_csv(DEST/'drive_summary.csv').itertuples():
        f=pd.read_csv(OUT/row.drive/'uniform.csv'); f['vy_ref']=np.nan
        if row.split.startswith('june'):
            import followup
            drive='12-11-24' if row.split=='june_calibration' else '12-09-31'
            ref=followup.load(drive)
            # Map native times through their shared ROS epoch (not row indices).
            cp=ROOT/'output/mncav_wheel_only_20260916/calibration_sensors/inspva.clock.json' if drive=='12-11-24' else ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.clock.json'
            clock=json.loads(cp.read_text());o=pd.read_csv(OUT/row.drive/'ouster.csv')
            ros=(f.time-row.clockOffset)/row.clockScale+o.stamp.iloc[0]
            rt=base.convert_time(clock,ros)
            f['vy_ref']=np.interp(rt,ref.time,ref.vy)
            f['valid']&=(np.interp(rt,ref.time,ref.ins_quality.astype(float))>.999)&(rt>.6)&(rt<ref.time.iloc[-1]-.6)
            parts=[('june_train',(rt>=1)&(rt<=39.5)),('june_holdout',rt>=40.5)] if drive=='12-11-24' else [('june_evaluation',np.ones(len(f),bool))]
        else:parts=[(row.split,np.ones(len(f),bool))]
        for split,mask in parts:
            ids=np.flatnonzero(f.valid&mask)
            for group in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
                for start in range(0,len(group),400):
                    ind=group[start:start+400]
                    if len(ind)<=75:continue
                    z=f.iloc[ind]
                    out.append(dict(drive=row.drive,split=split,t=z.time.to_numpy(),v=z.speed.to_numpy(),sw=z.sw.to_numpy(),
                       r=z.r.to_numpy(),vy=z.vy_ref.to_numpy(),ay=z.ay.to_numpy(),x=z.index.to_numpy()))
    return out

@njit(cache=True)
def midpoint(time,speed,delta,initial,parameters):
    """A-stable implicit midpoint integration of the time-varying bicycle."""
    qf,qr,j=parameters;lf=base.LF;lr=base.LR
    states=np.zeros((len(time),2));states[0]=initial
    for k in range(len(time)-1):
        h=(time[k+1]-time[k])/4;z1=states[k,0];z2=states[k,1]
        for step in range(4):
            fraction=(step+.5)/4
            v=speed[k]+fraction*(speed[k+1]-speed[k]);d=delta[k]+fraction*(delta[k+1]-delta[k])
            a=-(qf+qr)/v;b=(-lf*qf+lr*qr)/v-v
            c=(-lf*qf+lr*qr)/(j*v);e=-(lf*lf*qf+lr*lr*qr)/(j*v)
            f1=(1+h*a/2)*z1+h*b/2*z2+h*qf*d
            f2=h*c/2*z1+(1+h*e/2)*z2+h*lf*qf/j*d
            determinant=(1-h*a/2)*(1-h*e/2)-h*h*b*c/4
            z1=min(1e5,max(-1e5,((1-h*e/2)*f1+h*b/2*f2)/determinant))
            z2=min(1e5,max(-1e5,(h*c/2*f1+(1-h*a/2)*f2)/determinant))
        states[k+1,0]=z1;states[k+1,1]=z2
    return states

def physical(p,s):
    delta=lowpass((s['sw']-p[3])/p[4],p[5])
    return midpoint(s['t'],s['v'],delta,np.array([0.,s['r'][0]]),np.exp(p[:3]))

def features(s):
    u=s['v']*s['sw']/16.2
    signals=[u,u*s['v']**2/100,s['v']/10]
    return np.column_stack([lowpass(x,tau) for x in signals for tau in TAUS])

def score(name,predictions,data):
    records=[];full=[]
    for pred,s in zip(predictions,data):
        pred=np.asarray(pred);vy=pred[:,0] if pred.ndim==2 else np.full(len(pred),np.nan);r=pred[:,1] if pred.ndim==2 else pred
        for k in range(50,len(r)):
            full.append(dict(model=name,drive=s['drive'],split=s['split'],time=s['t'][k],referenceYaw=s['r'][k],predictedYaw=r[k],referenceVy=s['vy'][k],predictedVy=vy[k]))
    f=pd.DataFrame(full)
    for keys,g in f.groupby(['split','drive']):
        e=g.predictedYaw-g.referenceYaw;ev=g.predictedVy-g.referenceVy
        records.append(dict(model=name,split=keys[0],drive=keys[1],samples=len(g),yawRmseRadps=np.sqrt(np.mean(e**2)),yawBiasRadps=np.mean(e),vyRmseMps=np.sqrt(np.mean(ev**2)),vyBiasMps=np.mean(ev)))
    return records,f

def main():
    data=segments();train=[s for s in data if s['split'] in ['train','june_train']]
    rows=[];outputs=[];models={};starts=[]
    def evaluate(name,preds):
        r,f=score(name,preds,data);rows.extend(r);outputs.append(f)
    nominal=np.r_[base.NOMINAL[:3],base.INTERFACE['steeringWheelOffsetRad'],16.2,0.]
    prev=json.loads((ROOT/'research/mncav_identification_followup_20260925/summary.json').read_text())['candidateVectors']['yaw_alignment']
    previous=np.r_[prev[:3],prev[3]*16.2,16.2,0.]
    point=-2.3597988944583816
    def physical_outputs(p):
        arr=[physical(p,s) for s in data]
        return [np.c_[a[:,0]+point*a[:,1],a[:,1]] for a in arr]
    for name,p in [('nominal',nominal),('previous_yaw',previous)]:
        models[name]=p.tolist();evaluate(name,physical_outputs(p))
    rng=np.random.default_rng(20260925)
    for family in ['bicycle_fixed_ratio','bicycle_ratio_lag','bicycle_joint_vy']:
        active=list(range(6)) if family=='bicycle_ratio_lag' else [0,1,2,3]
        lo=np.array([np.log(5),np.log(5),np.log(.3),-.15,12.,.005])
        hi=np.array([np.log(400),np.log(400),np.log(8),.15,22.,.5])
        default=previous.copy();default[5]=.02 if len(active)==6 else 0.
        def residual(z):
            p=default.copy();p[active]=z
            errors=[]
            for s in train:
                prediction=physical(p,s);weight=np.sqrt(len(s['r'])-50)
                errors.extend((prediction[50:,1]-s['r'][50:])/.01/weight)
                if family=='bicycle_joint_vy' and s['split']=='june_train':
                    errors.extend((prediction[50:,0]+point*prediction[50:,1]-s['vy'][50:])/.05/weight)
            return np.array(errors)
        candidates=[]
        for k in range(5):
            x=default.copy()
            if k:x[:3]=rng.uniform(lo[:3]+.2,hi[:3]-.2)
            res=least_squares(residual,x[active],bounds=(lo[active],hi[active]),loss='soft_l1',max_nfev=200,x_scale='jac')
            p=default.copy();p[active]=res.x;candidates.append((np.mean(res.fun**2),p))
            starts.append(dict(family=family,start=k,cost=np.mean(res.fun**2),success=res.success,parameters=json.dumps(p.tolist())))
        p=min(candidates,key=lambda a:a[0])[1];models[family]=p.tolist();evaluate(family,physical_outputs(p));print(family,p,flush=True)
    def er(p):return np.concatenate([(empirical(p,s['v'],s['sw'],s['r'][0])[50:]-s['r'][50:])/.01/np.sqrt(len(s['r'])-50) for s in train])
    res=least_squares(er,[16.2,.002,.15,.026],bounds=([10,0,.01,-.15],[30,.1,2,.15]),loss='soft_l1',x_scale='jac',max_nfev=300)
    models['understeer_lag']=res.x.tolist();evaluate('understeer_lag',[empirical(res.x,s['v'],s['sw'],s['r'][0]) for s in data])
    x=np.vstack([features(s)[50:] for s in train]);y=np.concatenate([s['r'][50:] for s in train]);mean=x.mean(0);scale=x.std(0);scale[scale<1e-8]=1
    z=np.c_[np.ones(len(x)),(x-mean)/scale]
    # Equal total squared-error weight per prediction window.
    weights=np.concatenate([np.full(len(s['r'])-50,1/(len(s['r'])-50)) for s in train]);zw=z*np.sqrt(weights[:,None]);yw=y*np.sqrt(weights)
    for alpha in [.0001,.001,.01,.1,1.,10.,100.]:
        coef=np.linalg.solve(zw.T@zw+np.diag(np.r_[0.,np.full(x.shape[1],alpha)]),zw.T@yw)
        name='filter_bank_'+str(alpha);models[name]=dict(alpha=alpha,mean=mean.tolist(),scale=scale.tolist(),coefficients=coef.tolist(),taus=TAUS.tolist())
        evaluate(name,[np.c_[np.ones(len(s['r'])),(features(s)-mean)/scale]@coef for s in data])
    metrics=pd.DataFrame(rows);metrics.to_csv(DEST/'lateral_metrics.csv',index=False)
    selection=metrics[metrics.split=='validation'].groupby('model').yawRmseRadps.mean().sort_values()
    selected=next(name for name in selection.index if name not in ['nominal','previous_yaw'])
    selected_physical=next(name for name in selection.index if name.startswith('bicycle'))
    pd.DataFrame(starts).to_csv(DEST/'physical_multistart.csv',index=False)
    pd.concat(outputs,ignore_index=True).to_csv(OUT/'lateral_predictions.csv',index=False)
    (DEST/'lateral_models.json').write_text(json.dumps(dict(selected=selected,selectedPhysical=selected_physical,selectionMetric='May validation mean of per-drive yaw RMSE',selectionScores=selection.to_dict(),candidates=models,outputPointX=point,seed=20260925),indent=2)+'\n')
    print('Selected',selected,selection.to_dict(),flush=True)
if __name__=='__main__':main()
