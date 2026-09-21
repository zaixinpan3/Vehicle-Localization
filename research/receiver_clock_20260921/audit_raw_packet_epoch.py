"""Audit one cloud against raw Ouster columns without assuming hardware sync.

uv run --offline --with numpy --with rosbags python research/receiver_clock_20260921/audit_raw_packet_epoch.py
This recorded packet layout is verified by range/ring matches. It is not a
firmware-independent packet decoder or an absolute GPS calibration.
"""
from pathlib import Path
import json
import struct
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT=Path(__file__).resolve().parents[2]


def main():
    path=ROOT/'data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag'
    topic='/vehicle/lidar/front_ouster/'
    store=get_typestore(Stores.EMPTY)
    packets=[];cloud=None;clock_status=None
    with Reader(path) as reader:
        conns=[c for c in reader.connections if c.topic in [topic+'lidar_packets',topic+'points','/novatel/oem7/time']]
        for c in conns:
            store.register(get_types_from_msg(c.msgdef.data,c.msgtype))
        for c,t,raw in reader.messages(connections=conns,start=int(1717776599.4*1e9),stop=int(1717776599.9*1e9)):
            message=store.deserialize_ros1(raw,c.msgtype)
            if c.topic.endswith('lidar_packets'):
                buf=bytes(message.buf)
                assert len(buf)==12609
                for j in range(16):
                    ns,measurement,frame,encoder=struct.unpack_from('<QHHI',buf,j*788)
                    if measurement!=100:continue
                    ranges=np.frombuffer(buf[j*788+16:j*788+784],dtype='<u4')[::3]&0xfffff
                    packets.append((frame,ns,t,ranges))
            elif c.topic.endswith('points'):
                stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                if abs(stamp-1717776599.665203488)<1e-6:cloud=message
        # Native receiver TIME is available, but supplies no Ouster-to-GPS
        # relation by itself. Preserve its actual status/offset as evidence.
        time_conns=[c for c in conns if c.topic=='/novatel/oem7/time']
        for c,t,raw in reader.messages(connections=time_conns):
            m=store.deserialize_ros1(raw,c.msgtype)
            clock_status=dict(clockStatus=int(m.clock_status),utcOffsetSeconds=float(m.utc_offset),
                              receiverOffsetSeconds=float(m.offset))
            break
    assert cloud is not None
    offsets={f.name:f.offset for f in cloud.fields}
    dtype=np.dtype(dict(names=['range','ring','t'],formats=['<u4','u1','<u4'],
                        offsets=[offsets[k] for k in ['range','ring','t']],itemsize=cloud.point_step))
    points=np.frombuffer(cloud.data,dtype=dtype)
    pairs=set(zip(points['range'].tolist(),points['ring'].tolist()))
    rows=[]
    for frame,ns,t,ranges in packets:
        matches=sum((int(r),k) in pairs for k,r in enumerate(ranges) if r>0)
        rows.append(dict(hardwareFrameId=frame,column=100,rangeRingMatches=matches,
                         columnSensorSeconds=ns*1e-9,packetBagSeconds=t*1e-9))
    best=max(rows,key=lambda r:r['rangeRingMatches'])
    assert best['hardwareFrameId']==10397 and best['rangeRingMatches']>=60
    report=dict(cloudFrame=307,cloudHeaderSeconds=1717776599.665203488,
                packetCandidates=rows,matchedHardwareFrame=best['hardwareFrameId'],
                pointRelativeTimeMinimumSeconds=int(points['t'].min())*1e-9,
                pointRelativeTimeMaximumSeconds=int(points['t'].max())*1e-9,
                nativeTimeStatus=clock_status,
                conclusion='Column range/ring data identify hardware scan 10397. Sensor timestamps are approximately 4882 s, not GPS/Unix epoch. No recoverable absolute clock mode/configuration is assumed. Point t spans one rotation; header-to-scan epoch and deskew remain uncalibrated.')
    (Path(__file__).parent/'raw_packet_epoch.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
