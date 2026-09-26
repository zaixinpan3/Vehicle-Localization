#!/usr/bin/env python3
"""Freeze source-backed nominal parameters and identify the remaining dynamics."""
import json,sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.integrate import solve_ivp
from numba import njit
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent
OUT=ROOT/'output/mncav_public_constraints_20260925'
sys.path.insert(0,str(ROOT/'research/mncav_multidrive_identification_20260925'))
import lateral

@njit(cache=True)
def simulate(t,v,sw,p,lf,lr,ratio):
    qf,qr,j=np.exp(p[:3]);delta=(sw-p[3])/ratio
    states=np.zeros((len(t),2));states[0,1]=p[4]
    for k in range(len(t)-1):
        h=(t[k+1]-t[k])/4;z1,z2=states[k]
        for step in range(4):
            f=(step+.5)/4;speed=v[k]+f*(v[k+1]-v[k]);d=delta[k]+f*(delta[k+1]-delta[k])
            a=-(qf+qr)/speed;b=(-lf*qf+lr*qr)/speed-speed
            c=(-lf*qf+lr*qr)/(j*speed);e=-(lf*lf*qf+lr*lr*qr)/(j*speed)
            f1=(1+h*a/2)*z1+h*b/2*z2+h*qf*d;f2=h*c/2*z1+(1+h*e/2)*z2+h*lf*qf/j*d
            det=(1-h*a/2)*(1-h*e/2)-h*h*b*c/4
            z1=((1-h*e/2)*f1+h*b/2*f2)/det;z2=(h*c/2*f1+(1-h*a/2)*f2)/det
            z1=min(1e5,max(-1e5,z1));z2=min(1e5,max(-1e5,z2))
        states[k+1,0]=z1;states[k+1,1]=z2
    return states

def main():
    OUT.mkdir(exist_ok=True,parents=True)
    registry=json.loads((DEST/'parameter_registry.json').read_text());L=registry['fixed_public']['wheelbaseM']['value'];ratio=registry['fixed_public']['steeringRatio']['value']
    nominal_lf=L*(1-registry['fixed_nominal_public']['frontLoadFraction']['value']);mass=registry['fixed_nominal_public']['massKg']['value']
    x0=registry['retained_empirical_calibrations']['outputPointXAtNominalCgM']
    data=lateral.segments();train=[s for s in data if s['split'] in ['train','june_train']]
    start=np.array(json.loads((lateral.DEST/'lateral_models.json').read_text())['candidates']['bicycle_joint_vy'][:4])
    rng=np.random.default_rng(20260925);parameters=[];metrics=[];multistart=[];predictions=[];solutions={};jacobians={}
    for shift in registry['sensitivity_only']['cgShiftM']:
        lf=nominal_lf+shift;lr=L-lf;point=x0+shift
        def predict(p,s):
            result=simulate(s['t'],s['v'],s['sw'],np.r_[p,s['r'][0]],lf,lr,ratio)
            return np.c_[result[:,0]+point*result[:,1],result[:,1]]
        def residual(p):
            errors=[]
            for s in train:
                pred=predict(p,s);weight=np.sqrt(len(s['r'])-50)
                errors.extend((pred[50:,1]-s['r'][50:])/.01/weight)
                if s['split']=='june_train':errors.extend((pred[50:,0]-s['vy'][50:])/.05/weight)
            return np.array(errors)
        candidates=[]
        for k in range(3):
            initial=start.copy()
            if k:initial[:3]+=rng.normal(0,.2,3)
            result=least_squares(residual,initial,bounds=([np.log(5),np.log(5),np.log(.3),-.15],[np.log(400),np.log(400),np.log(8),.15]),x_scale='jac',loss='soft_l1',max_nfev=300,ftol=1e-10,xtol=1e-10,gtol=1e-10)
            candidates.append(result);multistart.append(dict(cgShiftM=shift,start=k,cost=float(np.mean(result.fun**2)),success=result.success,parameters=json.dumps(result.x.tolist())))
        result=min(candidates,key=lambda r:np.mean(r.fun**2));p=result.x;solutions[str(shift)]=p.tolist();jacobians[str(shift)]=result.jac
        qf,qr,j=np.exp(p[:3]);name=f'cg_{shift:+.2f}'
        records,full=lateral.score(name,[predict(p,s) for s in data],data);metrics.extend(records);predictions.append(full)
        # Scaling absolute mass, stiffness and inertia together changes no dynamics.
        for m in registry['sensitivity_only']['massKg']:
            parameters.append(dict(model=name,cgShiftM=shift,massKg=m,lfM=lf,lrM=lr,outputPointXM=point,CfNPerRad=qf*m,CrNPerRad=qr*m,IzKgM2=j*m,steeringRatio=ratio,steeringWheelZeroDeg=np.rad2deg(p[3]),fitCost=np.mean(result.fun**2)))
        print(name,'stock-mass Cf Cr Iz',qf*mass,qr*mass,j*mass,'zero',np.rad2deg(p[3]),flush=True)
    pd.DataFrame(parameters).to_csv(DEST/'conditional_parameters.csv',index=False)
    pd.DataFrame(metrics).to_csv(DEST/'prediction_metrics.csv',index=False)
    pd.DataFrame(multistart).to_csv(DEST/'multistart.csv',index=False)
    full=pd.concat(predictions,ignore_index=True);full.to_csv(OUT/'predictions.csv',index=False)
    selected=next(row for row in parameters if row['cgShiftM']==0 and row['massKg']==mass)
    (DEST/'constrained_model.json').write_text(json.dumps(dict(status='Fixed public nominal model; loaded mass/CG are not measured',fixed=selected,identifiedVector=solutions['0.0'],selection='Nominal published stock geometry fixed prospectively, not selected using evaluation errors',parameterOrder=['log(Cf/m)','log(Cr/m)','log(Iz/m)','steeringWheelZeroRad'],productionConfigurationChanged=False),indent=2)+'\n')
    # Independent score arithmetic and numerical comparison with previous solver.
    maxdiff=0.
    for row in metrics:
        g=full[(full.model==row['model'])&(full.drive==row['drive'])&(full.split==row['split'])]
        expected=np.sqrt(np.mean((g.predictedYaw-g.referenceYaw)**2));maxdiff=max(maxdiff,abs(expected-row['yawRmseRadps']))
    s=next(s for s in data if s['split']=='june_train');p=np.array(solutions['0.0'])
    a=simulate(s['t'],s['v'],s['sw'],np.r_[p,s['r'][0]],nominal_lf,L-nominal_lf,ratio)
    b=lateral.physical(np.r_[p,ratio,0.],s);solver_difference=float(np.max(abs(a-b)))
    assert solver_difference<1e-12 and maxdiff<1e-12
    qf,qr,j=np.exp(p[:3]);scale_checks=[]
    for m in registry['sensitivity_only']['massKg']:
        reconstructed=np.log(np.array([qf*m,qr*m,j*m])/m)
        other=simulate(s['t'],s['v'],s['sw'],np.r_[reconstructed,p[3],s['r'][0]],nominal_lf,L-nominal_lf,ratio)
        scale_checks.append(float(np.max(abs(other-a))))
    assert max(scale_checks)<1e-12
    assert np.max(abs(full.predictedYaw))<2 and np.max(abs(full.predictedVy))<10
    assert all(abs(row['lfM']+row['lrM']-L)<1e-12 and row['steeringRatio']==ratio for row in parameters)
    result=dict(metricRows=len(metrics),metricRecomputeMaxDifference=maxdiff,previousSolverMaxDifference=solver_difference,massScalePredictionMaxDifferences=scale_checks,fixedGeometryChecks='passed',multistartFits=len(multistart),allFitsConverged=all(row['success'] for row in multistart),noFinalTrajectoryGuardActivation=True)
    (DEST/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
if __name__=='__main__':main()
