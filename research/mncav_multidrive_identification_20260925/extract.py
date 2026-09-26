#!/usr/bin/env python3
"""Extract motion, legacy Ouster IMU clocks, and documented torque CAN frames."""
import csv, hashlib, json, struct
from collections import Counter
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
ROOT=Path(__file__).resolve().parents[2]; DEST=Path(__file__).resolve().parent
OUT=ROOT/'output/mncav_multidrive_identification_20260925'
TOPICS={'/vehicle/imu/data_raw':'imu','/vehicle/steering_report':'steer',
 '/vehicle/wheel_speed_report':'wheel','/vehicle/lidar/front_ouster/imu_packets':'ouster',
 '/vehicle/can_bus_dbw/can_rx':'can'}

def decode_torque(identifier, data):
    """Dataspeed FCA release a8015bc: sentinel-aware, little-endian decoding."""
    value=int.from_bytes(data,'little')
    if identifier==0x74:
        request=value&4095;actual=(value>>12)&4095
        return [np.nan if request==4095 else request*3.,np.nan if actual==4095 else actual*3.]
    value=value&32767; value=value-32768 if value&16384 else value
    return [np.nan if value==-16384 else value*1.5625]

def main():
    OUT.mkdir(exist_ok=True,parents=True);reports=[]
    bags=list(csv.DictReader((ROOT/'research/mncav_identification_followup_20260925/bag_inventory.csv').open()))
    for item in bags:
        path=ROOT/item['bag'];folder=OUT/path.stem;folder.mkdir(exist_ok=True)
        cached=folder/'manifest.json'
        if cached.exists():
            reports.append(json.loads(cached.read_text()));continue
        streams={k:[] for k in TOPICS.values()};hashes={};ids=Counter();counts=Counter();frames={}
        with Reader(path) as reader:
            selected=[c for c in reader.connections if c.topic in TOPICS]
            store=get_typestore(Stores.EMPTY);types={}
            for c in selected:types.update(get_types_from_msg(c.msgdef.data,c.msgtype))
            store.register(types)
            for c,t,raw in reader.messages(connections=selected):
                k=TOPICS[c.topic];m=store.deserialize_ros1(raw,c.msgtype)
                hashes.setdefault(k,hashlib.sha256()).update(t.to_bytes(8,'little')+len(raw).to_bytes(8,'little')+raw)
                counts[k]+=1;stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9 if hasattr(m,'header') else t*1e-9
                if hasattr(m,'header'):frames[k]=m.header.frame_id
                if k=='imu':row=[stamp,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y]
                elif k=='steer':row=[stamp,m.steering_wheel_angle,m.speed]
                elif k=='wheel':row=[stamp,m.front_left,m.front_right,m.rear_left,m.rear_right]
                elif k=='ouster':
                    assert len(m.buf)==48 or (len(m.buf)==49 and m.buf[-1]==0), 'Unexpected IMU packet length/padding'
                    row=[t*1e-9,*struct.unpack('<QQQffffff',bytes(m.buf[:48]))]
                else:
                    ids[int(m.id)]+=1
                    if m.id not in [0x74,0x75] or m.dlc<8 or m.is_error or m.is_rtr or m.is_extended:continue
                    row=[stamp,int(m.id),*decode_torque(m.id,bytes(m.data))]
                    if m.id==0x75:row.append(np.nan)
                streams[k].append(row)
        fields={'imu':['stamp','r','ax','ay'],'steer':['stamp','sw','speed'],
         'wheel':['stamp','fl','fr','rl','rr'],'ouster':['stamp','diagnostic_ns','accel_ns','gyro_ns','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps'],
         'can':['stamp','id','request_or_axle_nm','actual_brake_nm']}
        export_hashes={}
        for k,rows in streams.items():
            out=folder/(k+'.csv')
            with out.open('w') as f:
                w=csv.writer(f,lineterminator='\n');w.writerow(fields[k]);w.writerows(rows)
            export_hashes[k]=hashlib.sha256(out.read_bytes()).hexdigest()
        report=dict(bag=item['bag'],bagBytes=path.stat().st_size,counts=dict(counts),frames=frames,
          canIds={hex(k):v for k,v in sorted(ids.items())},topicStreamSha256={k:v.hexdigest() for k,v in hashes.items()},exportSha256=export_hashes)
        cached.write_text(json.dumps(report,indent=2)+'\n');reports.append(report)
        print(path.stem,dict(counts),'torque',ids[0x74],ids[0x75],flush=True)
    (DEST/'extraction_manifest.json').write_text(json.dumps(reports,indent=2)+'\n')
if __name__=='__main__':main()
