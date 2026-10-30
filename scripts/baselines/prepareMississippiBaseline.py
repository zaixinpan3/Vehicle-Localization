"""Prepare temporary raw-bag inputs for the two GNSS-equipped Mississippi drives.

Run with uv run --with rosbags --with numpy --with scipy --with pyproj.
Generated frame arrays and sensor exports belong in a temporary directory.
"""
from pathlib import Path
import sys, argparse, json
import numpy as np
from scipy.io import savemat
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from extractGnssFromBag import bestpos_row, inspva_row, lidar_row, write_csv
from receiverClock import ensure_clock, convert_time, native_seconds
from prepareInspvaMappingPoses import prepare
from pyproj import Transformer, Geod

def main():
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=True);rows={'bestpos':[],'inspva':[],'frames':[]}
 topics={'/novatel/oem7/bestpos':'bestpos','/novatel/oem7/inspva':'inspva','/vehicle/lidar/front_ouster/points':'frames'}
 mount=json.loads(Path('config/mississippiLidarMount.json').read_text());rot=np.array(mount['additionalRotation'])@np.array(mount['baseRotation'])
 with Reader(a.bag) as r:
  cs=[c for c in r.connections if c.topic in topics];store=get_typestore(Stores.EMPTY)
  for c in cs:store.register(get_types_from_msg(c.msgdef.data,c.msgtype))
  for c,ts,raw in r.messages(connections=cs):
   m=store.deserialize_ros1(raw,c.msgtype);kind=topics[c.topic];i=len(rows[kind])+1
   if kind!='frames':rows[kind].append((bestpos_row if kind=='bestpos' else inspva_row)(m,ts,i));continue
   stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
   rows[kind].append(dict(frame_index=i,stamp_sec=stamp));h,w=int(m.height),int(m.width)
   types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
   dtype=np.dtype(dict(names=[f.name for f in m.fields],formats=[('<' if not m.is_bigendian else '>')+types[f.datatype] for f in m.fields],offsets=[f.offset for f in m.fields],itemsize=m.point_step))
   cloud=np.ndarray((h,w),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
   xyz=np.stack([cloud[k] for k in ['x','y','z']],axis=-1)@rot.T
   frame={k:xyz[:,:,j].astype('float32') for j,k in enumerate(['x','y','z'])};frame['timestamp']=stamp
   for k in ['intensity','reflectivity','ambient','range']:
    frame[k]=cloud[k].astype('float32') if k in cloud.dtype.names else np.zeros((h,w),np.float32)
    if k=='range' and k in cloud.dtype.names and cloud[k].dtype.kind in 'ui':frame[k]*=.001
   savemat(a.output/f'frame_{i:04d}.mat',dict(frame=frame),do_compression=False)
   if i%100==0:print('EXTRACT',i,flush=True)
 for k,v in rows.items():write_csv(a.output/f'{k}.csv',v)
 prepare(a.output/'frames.csv',a.output/'inspva.csv',a.output/'poses.csv')
 clock=ensure_clock(a.output/'inspva.csv');start=convert_time(clock,[rows['frames'][0]['stamp_sec']])[0]
 b=rows['bestpos'];time=native_seconds([x['gps_week'] for x in b],[x['gps_seconds'] for x in b],clock['receiverOriginGpsWeek'],clock['receiverOriginGpsSeconds'])-start
 xy=Transformer.from_crs(4326,32615,always_xy=True).transform([x['longitude_deg'] for x in b],[x['latitude_deg'] for x in b])
 transform=Transformer.from_crs(4326,32615,always_xy=True);geod=Geod(ellps='WGS84')
 lon=np.array([x['longitude_deg'] for x in b]);lat=np.array([x['latitude_deg'] for x in b]);columns=[]
 for bearing in [90,0]:
  plus=geod.fwd(lon,lat,np.full(len(b),bearing),np.full(len(b),.5));minus=geod.fwd(lon,lat,np.full(len(b),bearing+180),np.full(len(b),.5))
  xp,yp=transform.transform(plus[0],plus[1]);xm,ym=transform.transform(minus[0],minus[1]);columns.append(np.column_stack((xp-xm,yp-ym)))
 jacobian=np.stack(columns,axis=2)
 out=[]
 for i,x in enumerate(b):
  sx=x['longitude_stdev_m'];sy=x['latitude_stdev_m'];valid=x['solution_status']==0 and sx>0 and sy>0
  I=np.linalg.inv(jacobian[i]@np.diag([max(sx,1e-6)**2,max(sy,1e-6)**2])@jacobian[i].T)
  out.append(dict(time=time[i],x=xy[0][i],y=xy[1][i],valid=int(valid),informationXX=I[0,0],informationXY=I[0,1],informationYY=I[1,1]))
 write_csv(a.output/'gnss.csv',out)
 print('PREPARED',len(rows['frames']),flush=True)
if __name__=='__main__':main()
