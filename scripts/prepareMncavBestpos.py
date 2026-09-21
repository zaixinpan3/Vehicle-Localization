#!/usr/bin/env python3
"""Export recorded BESTPOS XY/information on the receiver clock.

uv run --offline --with numpy --with pandas --with pyproj python scripts/prepareMncavBestpos.py
BESTPOS in this drive is INS aided, not a GNSS-only reference-independent fix.
No ODOM, INSPVA position/velocity or fitted spatial offset is used.
"""
from pathlib import Path
import argparse
import hashlib,json
import numpy as np
import pandas as pd
from pyproj import Transformer,Geod
from receiverClock import ensure_clock, convert_time, native_seconds

ROOT=Path(__file__).resolve().parents[1]


def main():
    folder=ROOT/'data/raw/Missisipi/gnss';stem='raw_data_2024-06-07-12-09-31_0'
    paths=[folder/(stem+s) for s in ['_bestpos.csv','_inspva.csv','_front_lidar_points.csv']]
    best,ins,frames=[pd.read_csv(p) for p in paths]
    clock=ensure_clock(paths[1])
    start=convert_time(clock,frames.stamp_sec.iloc[0])
    time=native_seconds(best.gps_week,best.gps_seconds,
        clock['receiverOriginGpsWeek'],clock['receiverOriginGpsSeconds'])-start
    assert np.all(np.diff(time)>0)
    lon=best.longitude_deg.to_numpy();lat=best.latitude_deg.to_numpy()
    transform=Transformer.from_crs(4326,32615,always_xy=True);east,north=transform.transform(lon,lat)
    geod=Geod(ellps='WGS84');columns=[]
    for bearing in [90,0]:
        plus=geod.fwd(lon,lat,np.full(len(lon),bearing),np.full(len(lon),.5))
        minus=geod.fwd(lon,lat,np.full(len(lon),bearing+180),np.full(len(lon),.5))
        xp,yp=transform.transform(plus[0],plus[1]);xm,ym=transform.transform(minus[0],minus[1])
        columns.append(np.column_stack((xp-xm,yp-ym)))
    jacobian=np.stack(columns,axis=2)
    sigmas=np.column_stack((best.longitude_stdev_m,best.latitude_stdev_m))
    valid=(best.solution_status.to_numpy()==0)&np.isfinite(sigmas).all(axis=1)&(sigmas>0).all(axis=1)&np.isfinite(east)&np.isfinite(north)
    information=np.full((len(time),2,2),np.nan)
    for i in np.flatnonzero(valid):
        covariance=jacobian[i]@np.diag(sigmas[i]**2)@jacobian[i].T
        information[i]=np.linalg.inv(covariance)
    result=pd.DataFrame(dict(time=time,x=east,y=north,valid=valid.astype(int),informationXX=information[:,0,0],
        informationXY=information[:,0,1],informationYY=information[:,1,1],positionType=best.position_type,
        solutionStatus=best.solution_status,sourceStamp=best.stamp_sec))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/receiver_synchronized_inputs')
    out=parser.parse_args().output;out.mkdir(parents=True,exist_ok=True)
    result.to_csv(out/'bestpos.csv',index=False)
    metadata=dict(source='/novatel/oem7/bestpos',samples=len(time),medianRateHz=float(1/np.median(np.diff(time))),
        positionTypes={str(int(k)):int(v) for k,v in best.position_type.value_counts().items()},
        interpretation='Recorded INS_RTKFIXED (56), INS_RTKFLOAT (55), INS_PSRDIFF (54); not pure GNSS',
        coordinates='EPSG:32615, same map projection; no spatial fit or lever-arm compensation',
        information='Reported east/north standard deviations projected by a numerical UTM Jacobian; unreported EN covariance assumed zero',
        clock=clock,
        runtimeReferencePoseUsed=False,odomUsed=False,validSamples=int(valid.sum()),
        sourceDocumentation='https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm',
        inputs=[dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in paths])
    (out/'bestpos_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
