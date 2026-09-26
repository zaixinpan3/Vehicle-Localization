#!/usr/bin/env python3
"""Constrained, held-out MnCAV bicycle identification; raw inputs are read-only.

uv run --with numpy --with scipy --with pandas --with matplotlib --with numba \
    python research/mncav_identification_20260925/identifyMncav.py

Parameters are Cf/m, Cr/m, Iz/m, road-steering zero, reference-point x,
small reference-heading correction, and steering time shift. Absolute mass
cannot be identified from this steering/kinematics-only model.
"""
from pathlib import Path
import hashlib
import json
import sys
from importlib.metadata import version

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
from numba import njit
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
OUT = ROOT / 'output/mncav_identification_20260925'
sys.path.insert(0, str(ROOT / 'scripts'))
from receiverClock import convert_time, native_seconds

VEHICLE = json.loads((ROOT / 'config/mncavVehicleParameters.json').read_text())
V = VEHICLE['vehicle']
MASS, LF, LR = V['mass'], V['lf'], V['lr']
RATIO = VEHICLE['steeringRatio']
INTERFACE = json.loads((ROOT / 'config/mncavReplayInterface.json').read_text())
NOMINAL = np.array([np.log(V['frontCorneringStiffness']/MASS),
                    np.log(V['rearCorneringStiffness']/MASS),
                    np.log(V['yawInertia']/MASS),
                    INTERFACE['steeringWheelOffsetRad']/RATIO, 0., 0., 0.])
LOW = np.array([np.log(5), np.log(5), np.log(.4), -.01, -3, -.05, -.15])
HIGH = np.array([np.log(150), np.log(150), np.log(6), .01, 3, .05, .15])
SCALES = np.array([1, 1, 1, .003, 1, .02, .05])
MODES = {'fixed_frame': [0, 1, 2], 'point_offset': [0, 1, 2, 4],
         'point_heading': [0, 1, 2, 4, 5], 'point_heading_steering': list(range(7))}


def load_drive(drive, smooth_seconds=.51):
    folder = 'calibration_sensors' if drive == '12-11-24' else 'sensors'
    base = ROOT / 'output/mncav_wheel_only_20260916' / folder
    clock_path = (base / 'inspva.clock.json' if drive == '12-11-24' else
                  ROOT / 'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.clock.json')
    clock = json.loads(clock_path.read_text())
    ins, imu, steering = [pd.read_csv(base / (name+'.csv')) for name in ['inspva', 'imu', 'steering']]
    time = native_seconds(ins.gps_week, ins.gps_seconds)
    dt = float(np.median(np.diff(time)))
    assert np.max(np.abs(np.diff(time)-dt)) < 1e-6
    yaw = np.unwrap(np.pi/2-np.deg2rad(ins.azimuth.to_numpy()))
    vx = ins.east_velocity.to_numpy()*np.cos(yaw)+ins.north_velocity.to_numpy()*np.sin(yaw)
    vy = -ins.east_velocity.to_numpy()*np.sin(yaw)+ins.north_velocity.to_numpy()*np.cos(yaw)
    window = int(round(smooth_seconds/dt)) | 1
    window = max(window, 5)
    smooth = lambda x, order=0: savgol_filter(x, window, 3, deriv=order, delta=dt)
    steer = np.interp(time, convert_time(clock, steering.stamp_sec), steering.steering_wheel_angle_rad)/RATIO
    raw_ay = np.interp(time, convert_time(clock, imu.stamp_sec), imu.acceleration_y_mps2)
    raw_r = np.interp(time, convert_time(clock, imu.stamp_sec), imu.angular_z_radps)
    frame = pd.DataFrame(dict(time=time, vx=smooth(vx), vy=smooth(vy),
        vx_dot=smooth(vx, 1), vy_dot=smooth(vy, 1), r=smooth(yaw, 1),
        r_dot=smooth(yaw, 2), steer=smooth(steer), raw_ay=raw_ay, raw_r=raw_r,
        roll=np.deg2rad(ins.roll.to_numpy())))
    frame['ay_ref'] = frame.vy_dot+frame.vx*frame.r
    audit = next(row for row in json.loads((DEST/'raw_bag_audit.json').read_text()) if row['drive']==drive)
    quality = np.ones(len(time), dtype=bool)
    for interval in audit['inspvaStatusIntervals']:
        if interval['status'] != 3:
            # Exclude the smoothing support around non-good INS intervals.
            collar = (window//2)*dt
            quality &= ((time<interval['startSeconds']-collar-1e-6) |
                        (time>interval['endSeconds']+collar+1e-6))
    frame['ins_quality'] = quality
    frame['valid'] = ((frame.time>=.6) & (frame.time<=time[-1]-.6) &
                      (frame.vx>=5) & (np.abs(frame.ay_ref)<=4) & quality)
    frame.attrs.update(drive=drive, dt=dt, smoothing=window*dt)
    return frame


def partition(frame, part):
    mask = frame.valid.to_numpy().copy()
    if part == 'fit': mask &= (frame.time.to_numpy()>=1) & (frame.time.to_numpy()<=39.5)
    if part == 'holdout': mask &= frame.time.to_numpy()>=40.5
    return mask


def derivative_residual(p, data):
    qf, qr, j = np.exp(p[:3])
    vx = data.vx.to_numpy(); r = data.r.to_numpy()
    vy = data.vy.to_numpy()+p[5]*vx-p[4]*r
    delta = np.interp(data.time.to_numpy()+p[6], data.time, data.steer)-p[3]
    safe_vx = np.maximum(vx, 1.)  # Only unused low-speed rows need protection.
    af = delta-(vy+LF*r)/safe_vx
    ar = -(vy-LR*r)/safe_vx
    ay = data.vy_dot.to_numpy()+p[5]*data.vx_dot.to_numpy()-p[4]*data.r_dot.to_numpy()+vx*r
    return np.c_[(qf*af+qr*ar-ay)/.15,
                 ((LF*qf*af-LR*qr*ar)/j-data.r_dot.to_numpy())/.03]


def fit_derivative(frame, mask, mode, start=NOMINAL, max_nfev=300):
    # Keep the full timeline during time shifting; subset residuals only.
    active = MODES[mode]
    def residual(z):
        p = start.copy(); p[active] = z
        return derivative_residual(p, frame)[mask].ravel()
    result = least_squares(residual, start[active], bounds=(LOW[active], HIGH[active]),
                           x_scale=SCALES[active], loss='soft_l1', max_nfev=max_nfev,
                           ftol=1e-9, xtol=1e-9, gtol=1e-9)
    p = start.copy(); p[active] = result.x
    return p, result


@njit(cache=True)
def flow(vy, r, speed, delta, qf, qr, inertia):
    af = delta-(vy+LF*r)/speed
    ar = -(vy-LR*r)/speed
    return qf*af+qr*ar-speed*r, (LF*qf*af-LR*qr*ar)/inertia


@njit(cache=True)
def simulate_array(time, speed, delta, initial, physical):
    qf, qr, j = physical
    states = np.zeros((len(time), 2)); states[0] = initial
    for k in range(len(time)-1):
        h = time[k+1]-time[k]; v, r = states[k]
        mid_speed = (speed[k]+speed[k+1])/2; mid_delta = (delta[k]+delta[k+1])/2
        a, b = flow(v,r,speed[k],delta[k],qf,qr,j)
        c, d = flow(v+h*a/2,r+h*b/2,mid_speed,mid_delta,qf,qr,j)
        e, f = flow(v+h*c/2,r+h*d/2,mid_speed,mid_delta,qf,qr,j)
        g, z = flow(v+h*e,r+h*f,speed[k+1],delta[k+1],qf,qr,j)
        states[k+1,0] = v+h*(a+2*c+2*e+g)/6
        states[k+1,1] = r+h*(b+2*d+2*f+z)/6
    return states


def prediction_segments(frame, mask, max_seconds=8):
    ids = np.flatnonzero(mask)
    groups = np.split(ids, np.flatnonzero(np.diff(ids)>1)+1)
    segments = []
    for group in groups:
        # Non-overlapping 8-second prediction windows, same for all models.
        for start in range(0,len(group),int(round(max_seconds/.02))):
            chosen = group[start:start+int(round(max_seconds/.02))]
            if len(chosen)>=51: segments.append(frame.iloc[chosen].copy())
    return segments


def predict(p, segment, full_frame):
    t = segment.time.to_numpy(); vx = segment.vx.to_numpy()
    delta = np.interp(t+p[6],full_frame.time,full_frame.steer)-p[3]
    initial = np.array([segment.vy.iloc[0]+p[5]*vx[0]-p[4]*segment.r.iloc[0],segment.r.iloc[0]])
    states = simulate_array(t,vx,delta,initial,np.exp(p[:3]))
    outputs = np.c_[states[:,0]+p[4]*states[:,1]-p[5]*vx, states[:,1]]
    return outputs


def forward_errors(p, segments, full_frame):
    return [predict(p,s,full_frame)[25:]-s[['vy','r']].to_numpy()[25:] for s in segments]


def fit_forward(frame, mask, mode, start):
    active = MODES[mode]; segments = prediction_segments(frame,mask)
    def residual(z):
        p = start.copy(); p[active] = z
        errors = np.vstack(forward_errors(p,segments,frame))
        return np.nan_to_num((errors/np.array([.1,.01])).ravel(),nan=1e8,posinf=1e8,neginf=-1e8)
    result = least_squares(residual,start[active],bounds=(LOW[active],HIGH[active]),
        x_scale=SCALES[active],loss='soft_l1',max_nfev=200,ftol=1e-8,xtol=1e-8,gtol=1e-8)
    p = start.copy(); p[active] = result.x
    return p,result


def parameter_row(name,p,result=None,mode=None):
    qf,qr,j=np.exp(p[:3]); active=MODES[mode] if mode else []
    at_bound=[int(k) for k in active if min(p[k]-LOW[k],HIGH[k]-p[k])<1e-4*(HIGH[k]-LOW[k])]
    condition=np.nan
    if result is not None:
        singular=np.linalg.svd(result.jac*SCALES[active],compute_uv=False)
        condition=float(singular[0]/singular[-1]) if singular[-1]>0 else np.inf
    return dict(model=name,CfOverMass=qf,CrOverMass=qr,IzOverMass=j,
        conditionalCfNPerRad=qf*MASS,conditionalCrNPerRad=qr*MASS,conditionalIzKgM2=j*MASS,
        roadSteeringOffsetRad=p[3],steeringWheelOffsetDeg=np.rad2deg(p[3]*RATIO),
        referenceXFromCgM=p[4],referenceHeadingCorrectionDeg=np.rad2deg(p[5]),
        steeringTimeShiftSeconds=p[6],boundIndices=str(at_bound),
        scaledJacobianCondition=condition,success=bool(result.success) if result is not None else True,
        evaluations=int(result.nfev) if result is not None else 0)


def score(name,p,frame,part):
    mask=partition(frame,part); segments=prediction_segments(frame,mask)
    errors=np.vstack(forward_errors(p,segments,frame))
    derivative=derivative_residual(p,frame)[mask]*[.15,.03]
    return dict(model=name,drive=frame.attrs['drive'],partition=part,
        fitEquationSamples=int(mask.sum()),predictionSamples=len(errors),predictionSegments=len(segments),
        vyPredictionRmseMps=float(np.sqrt(np.mean(errors[:,0]**2))),
        yawPredictionRmseRadps=float(np.sqrt(np.mean(errors[:,1]**2))),
        vyPredictionBiasMps=float(errors[:,0].mean()),
        lateralEquationRmseMps2=float(np.sqrt(np.mean(derivative[:,0]**2))),
        yawEquationRmseRadps2=float(np.sqrt(np.mean(derivative[:,1]**2))))


def sensor_consistency(calibration,evaluation):
    mask=partition(calibration,'fit')
    outputs=[]
    # This effective DBW-to-INS lever term is separate from the bicycle CG.
    for geometry in ['bias_only','bias_lever','bias_lever_roll']:
        def columns(d):
            cols=[np.ones(len(d))]
            if 'lever' in geometry: cols.append(d.r_dot.to_numpy())
            if 'roll' in geometry: cols.append(9.80665*np.sin(d.roll.to_numpy()))
            return np.column_stack(cols)
        X=columns(calibration); y=-calibration.raw_ay-calibration.ay_ref
        result=least_squares(lambda p:(X@p-y.to_numpy())[mask]/.15,np.zeros(X.shape[1]),loss='soft_l1')
        for data,part in [(calibration,'fit'),(calibration,'holdout'),(evaluation,'evaluation')]:
            use=partition(data,part);err=(-data.raw_ay-data.ay_ref).to_numpy()-columns(data)@result.x
            outputs.append(dict(model=geometry,drive=data.attrs['drive'],partition=part,
                rmseMps2=float(np.sqrt(np.mean(err[use]**2))),biasMps2=float(err[use].mean()),
                fittedCoefficients=json.dumps(result.x.tolist())))
    return outputs


def drive_diagnostics(calibration, evaluation):
    rows=[]
    fit=partition(calibration,'fit')
    gyro_offset=float(np.median((calibration.raw_r-calibration.r).to_numpy()[fit]))
    for data,part in [(calibration,'fit'),(calibration,'holdout'),(evaluation,'evaluation')]:
        mask=partition(data,part)
        straight=mask & (np.abs(data.r.to_numpy())<.01)
        error=(data.raw_r-data.r-gyro_offset).to_numpy()[mask]
        rows.append(dict(drive=data.attrs['drive'],partition=part,usedSamples=int(mask.sum()),
            goodInsAfterSmoothingCollar=int(data.ins_quality.sum()),
            speedMinMps=float(data.vx[mask].min()),speedMaxMps=float(data.vx[mask].max()),
            yawMinRadps=float(data.r[mask].min()),yawMaxRadps=float(data.r[mask].max()),
            fittedDbwGyroBiasRadps=gyro_offset,gyroRmseRadps=float(np.sqrt(np.mean(error**2))),
            straightSamples=int(straight.sum()),straightMeanVyMps=float(data.vy[straight].mean()),
            straightMeanCourseMinusHeadingDeg=float(np.rad2deg(np.arctan2(data.vy[straight],data.vx[straight])).mean())))
    return rows


def source_hashes():
    paths=[ROOT/'config'/name for name in ['mncavVehicleParameters.json',
           'mncavReplayInterface.json','mncavMotionOutputPoint.json']]
    paths += [ROOT/'scripts/receiverClock.py',DEST/'raw_bag_audit.json']
    for folder in ['calibration_sensors','sensors']:
        paths += [ROOT/'output/mncav_wheel_only_20260916'/folder/(name+'.csv')
                  for name in ['inspva','imu','steering']]
    paths += [ROOT/'output/mncav_wheel_only_20260916/calibration_sensors/inspva.clock.json',
              ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.clock.json']
    pd.DataFrame([dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,
        sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in paths]).to_csv(DEST/'source_hashes.csv',index=False)


def synthetic_recovery():
    t=np.arange(0,24,.02);vx=12+3*np.sin(.3*t)
    raw=NOMINAL[3]+.025*np.sin(.8*t)+.015*np.sin(1.7*t)
    truth=NOMINAL.copy();truth[:3]+=np.log([1.15,.85,1.1]);truth[4:6]=[-1.1,.006]
    state=simulate_array(t,vx,raw-truth[3],np.array([0.,0.]),np.exp(truth[:3]))
    f=pd.DataFrame(dict(time=t,vx=vx,steer=raw,vy=state[:,0]+truth[4]*state[:,1]-truth[5]*vx,r=state[:,1]))
    fitted,result=fit_forward(f,np.ones(len(t),dtype=bool),'point_heading',NOMINAL)
    assert np.max(np.abs(fitted[:3]-truth[:3]))<1e-5
    assert np.max(np.abs(fitted[4:6]-truth[4:6]))<1e-5
    flexible_truth=truth.copy();flexible_truth[3]=.0025;flexible_truth[6]=.035
    shifted=np.interp(t+flexible_truth[6],t,raw)-flexible_truth[3]
    state=simulate_array(t,vx,shifted,np.array([0.,0.]),np.exp(flexible_truth[:3]))
    f['vy']=state[:,0]+flexible_truth[4]*state[:,1]-flexible_truth[5]*vx
    f['r']=state[:,1]
    flexible_fit,flexible_result=fit_forward(f,np.ones(len(t),dtype=bool),'point_heading_steering',NOMINAL)
    assert np.max(np.abs((flexible_fit-flexible_truth)/SCALES))<1e-5
    # Re-scaling every extensive physical parameter leaves normalized flow unchanged.
    scale=1.25
    physical=np.array([V['frontCorneringStiffness'],V['rearCorneringStiffness'],V['yawInertia']])
    assert np.allclose(physical/MASS,(scale*physical)/(scale*MASS),rtol=1e-15)
    return dict(recoveredLogRatioMaxError=float(np.max(np.abs(fitted[:3]-truth[:3]))),
        recoveredOffsetMaxError=float(np.max(np.abs(fitted[4:6]-truth[4:6]))),massScaleInvariant=True,
        sevenParameterRecoveryMaxScaledError=float(np.max(np.abs((flexible_fit-flexible_truth)/SCALES))),
        sevenParameterOptimizerSuccess=bool(flexible_result.success),
        optimizerSuccess=bool(result.success))


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(20260925)
    recovery=synthetic_recovery();print('Synthetic recovery:',recovery,flush=True)
    calibration=load_drive('12-11-24');fitmask=partition(calibration,'fit')
    parameters=[parameter_row('nominal',NOMINAL)];models={'nominal':NOMINAL.copy()}
    for mode in MODES:
        p,result=fit_derivative(calibration,fitmask,mode)
        name='derivative_'+mode;models[name]=p;parameters.append(parameter_row(name,p,result,mode))
        print(name,parameters[-1],flush=True)
        p,result=fit_forward(calibration,fitmask,mode,p)
        name='forward_'+mode;models[name]=p;parameters.append(parameter_row(name,p,result,mode))
        print(name,parameters[-1],flush=True)
    # Select the structure using calibration holdout ONLY; evaluation not yet loaded.
    scores=[]
    for name,p in models.items():
        for part in ['fit','holdout']:scores.append(score(name,p,calibration,part))
    holdout=[r for r in scores if r['partition']=='holdout']
    selected=min(holdout,key=lambda r:(r['vyPredictionRmseMps']/.1)**2+(r['yawPredictionRmseRadps']/.01)**2)['model']
    print('Selected before evaluation:',selected,flush=True)
    # Additional previously calibrated output-point baseline, not a new fit.
    # Added diagnostically after the initial unfiltered run; excluded from selection.
    baseline=NOMINAL.copy()
    baseline[4]=-json.loads((ROOT/'config/mncavMotionOutputPoint.json').read_text())['forwardOffsetM']
    models['nominal_empirical_point']=baseline
    parameters.append(parameter_row('nominal_empirical_point',baseline))
    for part in ['fit','holdout']:scores.append(score('nominal_empirical_point',baseline,calibration,part))
    evaluation=load_drive('12-09-31')
    for name,p in models.items():scores.append(score(name,p,evaluation,'evaluation'))
    pd.DataFrame(parameters).to_csv(DEST/'candidate_parameters.csv',index=False)
    pd.DataFrame(scores).to_csv(DEST/'prediction_metrics.csv',index=False)
    pd.DataFrame(sensor_consistency(calibration,evaluation)).to_csv(DEST/'sensor_consistency.csv',index=False)
    pd.DataFrame(drive_diagnostics(calibration,evaluation)).to_csv(DEST/'drive_diagnostics.csv',index=False)
    # Smoothing and contiguous-block sensitivity are diagnostics, not IID CIs.
    sensitivity=[]
    for seconds in [.21,.51,1.01]:
        data=load_drive('12-11-24',seconds);mask=partition(data,'fit')
        p,result=fit_derivative(data,mask,'point_heading')
        row=parameter_row('smoothing_'+str(seconds),p,result,'point_heading');sensitivity.append(row)
    blocks=np.floor(calibration.time.to_numpy()/4).astype(int)
    for mode in ['point_offset','point_heading']:
        for block in np.unique(blocks[fitmask]):
            mask=fitmask & (blocks!=block)
            p,result=fit_forward(calibration,mask,mode,models['forward_'+mode])
            row=parameter_row(mode+'_omit_block_'+str(block),p,result,mode);sensitivity.append(row)
    pd.DataFrame(sensitivity).to_csv(DEST/'parameter_sensitivity.csv',index=False)
    # Multiple optimizer starts diagnose local minima in the most flexible fit.
    multistart=[]
    for k in range(8):
        start=NOMINAL.copy();start[:3]+=rng.uniform(-.4,.4,3)
        start[4:6]=[rng.uniform(-2,2),rng.uniform(-.015,.015)]
        p,result=fit_forward(calibration,fitmask,'point_heading_steering',start)
        row=parameter_row('start_'+str(k),p,result,'point_heading_steering');row['cost']=result.cost
        multistart.append(row)
    pd.DataFrame(multistart).to_csv(DEST/'multistart.csv',index=False)
    summary=dict(selectedOnCalibrationHoldout=selected,productionParametersChanged=False,
        fixedMassKg=MASS,absoluteMassIdentifiable=False,parameterNames=['log_Cf_over_m','log_Cr_over_m',
        'log_Iz_over_m','road_steering_offset_rad','reference_x_from_cg_m','heading_correction_rad','steering_shift_s'],
        candidateVectors={name:p.tolist() for name,p in models.items()},syntheticRecovery=recovery,
        bounds={'lower':LOW.tolist(),'upper':HIGH.tolist()},
        fixedGeometry={'lfM':LF,'lrM':LR,'steeringRatio':RATIO},randomSeed=20260925,
        preprocessing='INS status 3 only, non-good smoothing collars excluded, speed >=5 m/s, abs reference ay <=4 m/s2',
        software={name:version(name) for name in ['numpy','scipy','pandas','numba','matplotlib']})
    (DEST/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    source_hashes()
    calibration.to_csv(OUT/'calibration_signals.csv',index=False);evaluation.to_csv(OUT/'evaluation_signals.csv',index=False)
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex='col')
    for col,(frame,part) in enumerate([(calibration,'holdout'),(evaluation,'evaluation')]):
        segments=prediction_segments(frame,partition(frame,part))
        for channel in range(2):
            ax=axes[channel,col]
            for k,s in enumerate(segments):
                ax.plot(s.time,s[['vy','r']].to_numpy()[:,channel],color='black',lw=1,label='Reference' if k==0 else None)
                for name,color in [('nominal_empirical_point','tab:orange'),(selected,'tab:blue')]:
                    ax.plot(s.time,predict(models[name],s,frame)[:,channel],color=color,lw=1,label=name if k==0 else None)
            ax.grid(True,alpha=.3);ax.set_ylabel(['Lateral velocity (m/s)','Yaw rate (rad/s)'][channel]);ax.legend(fontsize=7)
        axes[0,col].set_title(frame.attrs['drive']+' '+part);axes[1,col].set_xlabel('Receiver time (s)')
    fig.tight_layout();fig.savefig(DEST/'prediction.png',dpi=160);plt.close(fig)
    print(pd.DataFrame(scores).to_string(index=False),flush=True)
    print('Complete; no production parameter adoption.',flush=True)


if __name__=='__main__':main()
