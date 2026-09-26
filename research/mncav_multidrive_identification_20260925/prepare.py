#!/usr/bin/env python3
"""Prepare uniform native-time motion tables; May split is fixed before fitting."""
import json,sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
ROOT=Path(__file__).resolve().parents[2]; DEST=Path(__file__).resolve().parent
OUT=ROOT/'output/mncav_multidrive_identification_20260925'
sys.path.insert(0,str(ROOT/'research/mncav_identification_followup_20260925'))
import followup
GYRO=followup.GYRO_BIAS

def main():
    reports=[]; index=0
    for report in json.loads((DEST/'extraction_manifest.json').read_text()):
        folder=OUT/Path(report['bag']).stem
        o,im,s,w,c=[pd.read_csv(folder/(k+'.csv')) for k in ['ouster','imu','steer','wheel','can']]
        origin=o.stamp.iloc[0];x=o.stamp.to_numpy()-origin
        y=(o.gyro_ns.to_numpy()-o.gyro_ns.iloc[0])*1e-9
        good=np.ones(len(x),bool)
        for _ in range(4):
            a,b=np.polyfit(x[good],y[good],1);err=y-(a*x+b);good=abs(err-np.median(err))<max(.002,6*np.median(abs(err-np.median(err))))
        assert np.all(np.diff(y)>0)
        p95=np.quantile(abs(err[good]),.95)
        assert p95<.002 and np.mean(good)>.9
        # Origin is bag first Ouster packet; all header stamps share ROS clock.
        start=max(f.stamp.min() for f in [im,s,w,c,o]);end=min(f.stamp.max() for f in [im,s,w,c,o])
        t=np.arange(np.ceil((a*(start-origin)+b)*50)/50,a*(end-origin)+b,.02)
        def interp(f,key):return np.interp(t,a*(f.stamp.to_numpy()-origin)+b,f[key])
        # Frozen radius calibration from June 11-24, no lag correction here.
        radii=json.loads((ROOT/'config/mncavWheelSpeedCalibration.json').read_text())['effectiveRadiusM']
        v=(interp(w,'rl')*radii[2]+interp(w,'rr')*radii[3])/2
        f=pd.DataFrame(dict(time=t,speed=v,sw=interp(s,'sw'),r=interp(im,'r')-GYRO,
          ay=-interp(im,'ay'),ax=interp(im,'ax'),ouster_r=np.deg2rad(interp(o,'gz_dps')),
          axle=interp(c[c.id==117],'request_or_axle_nm'),brake=interp(c[c.id==116],'actual_brake_nm')))
        if len(f)>31:
            for k in ['speed','sw','r','ay','ax','ouster_r','axle','brake']:f[k]=savgol_filter(f[k],25,3)
            f['acceleration']=savgol_filter(v,25,3,deriv=1,delta=.02)
        else:f['acceleration']=np.nan
        may='2024-05' in folder.name
        split=('validation' if index%3==0 else 'train') if may else ('june_calibration' if '12-11-24' in folder.name else 'june_evaluation')
        if may:index+=1
        f['valid']=(f.time>.6)&(f.time<f.time.iloc[-1]-.6)&(f.speed>=5)&(abs(f.ay)<=4)&(abs(f.r)<.6)
        f.to_csv(folder/'uniform.csv',index=False)
        diag=dict(drive=folder.name,split=split,samples=len(f),eligibleSamples=int(f.valid.sum()),
          clockScale=a,clockOffset=b,clockP95Seconds=p95,clockInlierFraction=float(np.mean(good)),
          speedMin=float(v.min()),speedMax=float(v.max()),yawMin=float(f.r.min()),yawMax=float(f.r.max()))
        if not may:
            previous=followup.load('12-11-24' if '12-11-24' in folder.name else '12-09-31')
            # Independent Ouster-vs-NovAtel clock-rate check; no transfer bias fit.
            cp=(ROOT/'output/mncav_wheel_only_20260916/calibration_sensors/inspva.clock.json') if '12-11-24' in folder.name else ROOT/'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.clock.json'
            clock=json.loads(cp.read_text());diag['novatelScale']=clock['scale']
        if f.valid.sum()>20:
            z=f.loc[f.valid];coef=np.polyfit(z.ouster_r,z.r,1)
            diag.update(ousterYawGain=coef[0],ousterYawOffset=coef[1],ousterYawResidualRmse=float(np.sqrt(np.mean((z.r-np.polyval(coef,z.ouster_r))**2))))
        reports.append(diag);print(folder.name,split,diag['eligibleSamples'],'clock',a,flush=True)
    pd.DataFrame(reports).to_csv(DEST/'drive_summary.csv',index=False)
if __name__=='__main__':main()
