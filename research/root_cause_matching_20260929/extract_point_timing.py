"""Recover verified point-relative Ouster timing without inferring a GPS epoch.

The documented stored-axis rotation and point order are checked for every frame before admitting its timing.
Output is a local data sidecar, not a replacement for the recorded point cloud.
"""
from pathlib import Path
import json
import h5py
import numpy as np
from scipy.io import savemat
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/root_cause_matching_20260929'

def main():
    store=get_typestore(Stores.EMPTY)
    bag=ROOT/'data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag'
    source=ROOT/'data/raw/MissisipiPointClouds.mat'
    times=np.zeros((64,1024,1170),dtype=np.float32);seen=np.zeros(1170,dtype=bool)
    maxima=np.zeros(1170);header_error=np.zeros(1170);xyz_error=np.zeros(1170)
    with h5py.File(source) as f, Reader(bag) as reader:
        p=f['pointClouds'];stamps=np.array([f[r[0]][0,0] for r in p['timestamp']])
        conns=[c for c in reader.connections if c.topic=='/vehicle/lidar/front_ouster/points']
        assert len(conns)==1
        for c in conns:store.register(get_types_from_msg(c.msgdef.data,c.msgtype))
        for c,_,raw in reader.messages(connections=conns):
            m=store.deserialize_ros1(raw,c.msgtype);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            k=int(np.argmin(abs(stamps-stamp)))
            if abs(stamps[k]-stamp)>1e-6:continue
            assert not seen[k], f'duplicate frame {k+1}'
            assert not m.is_bigendian and m.height==64 and m.width==1024
            offsets={q.name:q.offset for q in m.fields}
            dtype=np.dtype(dict(names=['x','y','z','t'],formats=['<f4','<f4','<f4','<u4'],offsets=[offsets[n] for n in ['x','y','z','t']],itemsize=m.point_step))
            assert m.row_step==m.point_step*m.width
            points=np.frombuffer(m.data,dtype=dtype).reshape(m.height,m.width)
            base=np.array([[.925216,-.367871,.093195],[.368647,.929524,.009468],[-.090110,.025595,.995625]])
            additional=np.array([[.931395,.364011,0],[-.364011,.931395,0],[0,0,1]])
            rotation=(additional@base).astype(np.float32)
            original=np.stack([points[n] for n in ['x','y','z']],axis=-1)
            transformed=original@rotation.T
            for j,n in enumerate(['x','y','z']):
                recorded=f[p[n][k,0]][:].T
                error=float(np.nanmax(abs(recorded-transformed[:,:,j])))
                xyz_error[k]=max(xyz_error[k],error)
                assert error<1e-4,f'XYZ transform/order mismatch {k+1} {n}: {error}'
            relative=points['t'].astype(np.float64)*1e-9
            assert relative.min()>=0 and .09<relative.max()<.11
            times[:,:,k]=relative;seen[k]=True;maxima[k]=relative.max();header_error[k]=abs(stamps[k]-stamp)
            if (k+1)%100==0:print(f'Validated timing {k+1}/1170',flush=True)
    assert seen.all(),np.flatnonzero(~seen)+1
    savemat(OUT/'point_timing.mat',dict(pointTimes=times,frameHeaderSeconds=stamps),do_compression=True)
    report=dict(frames=int(seen.sum()),pointsPerFrame=65536,xyzEquality='Stored-axis rotation and organized point order verified against every frame',maximumXyzDifferenceMeters=float(xyz_error.max()),relativeTimeUnit='seconds from PointCloud2 t nanoseconds',relativeSpanRangeSeconds=[float(maxima.min()),float(maxima.max())],maximumHeaderDifferenceSeconds=float(header_error.max()),absoluteHeaderToScanOffsetCalibrated=False)
    (ROOT/'research/root_cause_matching_20260929/point_timing_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
