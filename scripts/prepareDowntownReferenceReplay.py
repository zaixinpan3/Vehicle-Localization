#!/usr/bin/env python3
"""Prepare native-clock Downtown measurements and separate offline map poses.

Clouds retain native front-Ouster axes and original raw frame slots. Wheel,
DBW IMU and steering clocks are associated using raw Ouster packet timing;
no navigation or trajectory position/attitude fits the motion clocks. Pose
CSV files belong only to mapping/scoring and are never localization inputs.
Dependencies: rosbags, numpy, scipy, json5.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.io import savemat
from rosbags.rosbag1 import Reader as Reader1
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
from prepareOfflineReferenceBag import point_view
from extractOusterImuFromBag import decode_packet
from extractVehicleReplaySensors import sensor_row
from runOfflineReference import sha256
from validateOfflineReference import robust_line

TOPICS={'/vehicle/wheel_speed_report':'wheel_speed_report','/vehicle/imu/data_raw':'imu','/vehicle/steering_report':'steering'}

def prepare(prepared,reference,output):
    preparation=json.loads((prepared/'preparation.json').read_text())
    bag=Path(preparation['source_bag'])
    output.mkdir(parents=True,exist_ok=False);(output/'frames').mkdir();(output/'sensors').mkdir()
    frames=list(csv.DictReader((prepared/'frames.csv').open()))
    native_to_frame={int(r['native_start_ns']):int(r['frame_index']) for r in frames}
    store=get_typestore(Stores.ROS2_JAZZY)
    with Reader2(prepared/'rosbag2') as reader:
        for c,_,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/reference/points']):
            cloud=store.deserialize_cdr(raw,c.msgtype);stamp=cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec
            index=native_to_frame[stamp]
            data={f:np.asarray(point_view(cloud,f),dtype=np.float32) for f in ['x','y','z','intensity']}
            data.update(pointTimeSeconds=np.asarray(point_view(cloud,'t'),dtype=np.float64)*1e-9,timestamp=stamp*1e-9)
            savemat(output/'frames'/f'{index:06d}.mat',data,do_compression=True)
    rows={v:[] for v in TOPICS.values()};anchors=[]
    with Reader1(bag) as reader:
        cs=[c for c in reader.connections if c.topic in TOPICS or c.topic=='/vehicle/lidar/front_ouster/imu_packets']
        if set(TOPICS)-{c.topic for c in cs}:raise ValueError('Required measured vehicle motion topic missing')
        ts=get_typestore(Stores.EMPTY);types={}
        for c in cs:types.update(get_types_from_msg(c.msgdef.data,c.msgtype))
        ts.register(types)
        for c,arrival,raw in reader.messages(connections=cs):
            msg=ts.deserialize_ros1(raw,c.msgtype)
            if c.topic in TOPICS:rows[TOPICS[c.topic]].append(sensor_row(msg,arrival,TOPICS[c.topic]))
            else:
                _,a,g,*_=decode_packet(bytes(msg.buf));anchors.append([arrival*1e-9,(a+g)*.5e-9])
    anchors=np.asarray(anchors);coef,x0,y0,residual=robust_line(anchors[:,0],anchors[:,1])
    clock=dict(schemaVersion=1,method='raw_ouster_packet_arrival_affine',sourceOriginSeconds=float(x0),scale=float(coef[0]),offsetSeconds=float(coef[1]+y0),sourceSpanSeconds=float(np.ptp(anchors[:,0])),maximumExtrapolationSeconds=.15,referencePoseUsed=False,gnssUsed=False,receiptAssociationRmsSeconds=float(np.sqrt(np.mean(residual**2))))
    for kind,records in rows.items():
        for r in records:r['native_time_sec']=(r['stamp_sec']-x0)*coef[0]+coef[1]+y0
        with (output/'sensors'/f'{kind}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    (output/'clock.json').write_text(json.dumps(clock,indent=2)+'\n')
    with (output/'frames.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['frame_index','native_time_sec','available']);w.writeheader()
        for i in range(1,preparation['frames_seen']+1):
            source=next((r for r in frames if int(r['frame_index'])==i),None)
            w.writerow(dict(frame_index=i,native_time_sec=int(source['native_start_ns'])*1e-9 if source else '',available=int(source is not None)))
    # Canonical mapping poses are a separate artifact, not in sensor exports.
    refs=list(csv.DictReader((reference/'reference_poses.csv').open()))
    by_index={int(r['frame_index']):r for r in frames}
    canonical=[]
    for r in refs:
        if int(r['estimated']):
            index=int(r['frame_index'])
            if index not in by_index or abs(float(r['native_time_sec'])-int(by_index[index]['native_start_ns'])*1e-9)>1e-7:
                raise ValueError('Reference pose does not match this recording and original frame clock')
            canonical.append(dict(frame_index=index,lidar_stamp_sec=float(r['native_time_sec']),pose_x_m=float(r['x_m']),pose_y_m=float(r['y_m']),pose_z_m=float(r['z_m']),pose_qx=float(r['qx']),pose_qy=float(r['qy']),pose_qz=float(r['qz']),pose_qw=float(r['qw']),pose_source='independent_offline_pseudo_ground_truth'))
    with (output/'mapping_poses.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(canonical[0]));w.writeheader();w.writerows(canonical)
    metadata=dict(schemaVersion=1,measurementOnly=True,sourceBag=str(bag),sourceBagSha256=preparation['source_bag_sha256'],sourceFrames=preparation['frames_seen'],preparedFrames=len(frames),referenceTrajectorySha256=sha256(reference/'reference_lidar.tum'),referenceKind='pseudo_ground_truth',poseFrame='front_ouster',storedPointAxes='Native front Ouster XYZ; no historical empirical mounting transform',sensorTopics=list(TOPICS),gnssExported=False,clock=clock,hashes={str(p.relative_to(output)):sha256(p) for p in [output/'frames.csv',output/'mapping_poses.csv',output/'clock.json',*list((output/'sensors').glob('*.csv'))]})
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(bag.name,len(frames),'clouds;',len(canonical),'reference poses',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepared-root',type=Path,required=True);p.add_argument('--reference-root',type=Path,required=True);p.add_argument('--output-root',type=Path,required=True)
    args=p.parse_args()
    for prep in sorted((args.prepared_root/'Downtown').glob('*/preparation.json')):
        out=args.output_root/prep.parent.name
        if (out/'metadata.json').exists():continue
        prepare(prep.parent,args.reference_root/'Downtown'/prep.parent.name,out)
