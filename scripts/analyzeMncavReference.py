#!/usr/bin/env python3
"""Audit recorded reference provenance without replacing original inputs.

Requires numpy, pyproj, and rosbags. UTM-projected INSPVA is an alternate
same-receiver reference, not independent ground truth or a corrected map.
"""
from pathlib import Path
import csv
import importlib.util
import json
import numpy as np
from pyproj import Transformer
from rosbags.rosbag1 import Reader

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/mncav_error_diagnosis_20260914"
BASE = ROOT / "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0"


def read(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    pva = read(str(BASE) + "_inspva.csv")
    odom = read(str(BASE) + "_odom.csv")
    best = read(str(BASE) + "_bestpos.csv")
    calls = read(ROOT / "output/mncav_full_localization_20260914/matching/calls.csv")
    trajectory = read(ROOT / "output/mncav_full_localization_20260914/trajectory.csv")
    project = Transformer.from_crs(4326, 32615, always_xy=True)
    pva_xy = np.column_stack(project.transform(pva["longitude_deg"], pva["latitude_deg"]))
    best_xy = np.column_stack(project.transform(best["longitude_deg"], best["latitude_deg"]))
    odom_xy = np.column_stack([odom["x_m"], odom["y_m"]])
    start = np.interp(calls["rosStamp"][0], pva["stamp_sec"], pva["gps_seconds"])
    pva_time = pva["gps_seconds"] - start
    odom_time = np.interp(odom["stamp_sec"], pva["stamp_sec"], pva["gps_seconds"]) - start
    displacement = np.linalg.norm(np.diff(odom_xy, axis=0), axis=1)
    implied_speed = displacement / np.diff(odom_time)
    closest = {}
    for name, source, xy in [("pva", pva, pva_xy), ("bestpos", best, best_xy)]:
        index = np.searchsorted(source["stamp_sec"], odom["stamp_sec"])
        closest[name] = np.array([
            np.min(np.linalg.norm(xy[max(0, j-10):min(len(xy), j+2)]-odom_xy[k], axis=1))
            for k, j in enumerate(index)
        ])
    best_match = closest["bestpos"] < .001
    pose_table = read(str(BASE) + "_front_lidar_pose_match_1_1170.csv")
    best_indices = set(odom["index"][best_match])
    mixed_map_frames = [int(row["frame_index"]) for row in pose_table
                        if row["nearest_odom_index"] in best_indices]
    starts = np.flatnonzero(best_match & np.r_[True, ~best_match[:-1]])
    ends = np.flatnonzero(best_match & np.r_[~best_match[1:], True])
    intervals = [[float(odom_time[a]), float(odom_time[b])] for a, b in zip(starts, ends)]
    t = trajectory["time"]
    alternate = np.column_stack([np.interp(t, pva_time, pva_xy[:,j]) for j in range(2)])
    old = np.column_stack([trajectory["referenceX"], trajectory["referenceY"]])
    global_xy = np.column_stack([trajectory["x"], trajectory["y"]])
    scan_alt = np.column_stack([np.interp(calls["timeSeconds"], pva_time, pva_xy[:,j]) for j in range(2)])
    scan_xy = np.column_stack([calls["x"], calls["y"]])
    error = np.linalg.norm(global_xy-old, axis=1)
    mse = error**2
    # Fixed descriptive partitions retain every sample; no trimmed headline score.
    partitions = []
    for a, b in [(0,5),(5,20),(20,80),(80,90),(90,105),(105,115),(115,117)]:
        m = (t>=a) & (t<b)
        partitions.append(dict(start=a,end=b,samples=int(m.sum()),rmse=float(np.sqrt(mse[m].mean())),mseShare=float(mse[m].sum()/mse.sum())))
    source_difference = np.linalg.norm(old-alternate, axis=1)
    k = int(np.argmax(implied_speed))
    audit = dict(
        projection="EPSG:4326 to EPSG:32615, no fitted alignment or reference smoothing",
        rawOdomSamples=len(odom), rawPvaSamples=len(pva),
        nativeOdomMaxStepM=float(displacement.max()), maximumImpliedOdomSpeedMps=float(implied_speed[k]),
        worstStepTime=float(odom_time[k]), worstStepDurationSeconds=float(odom_time[k+1]-odom_time[k]),
        contemporaneousReportedSpeedMps=float(np.hypot(odom["linear_x_mps"][k],odom["linear_y_mps"][k])),
        nativeOdomStepsOver04M=int((displacement>=.4).sum()),
        nativePvaMaxStepM=float(np.linalg.norm(np.diff(pva_xy,axis=0),axis=1).max()),
        odomPointsMatchingPvaWithin1mm=int((closest["pva"]<.001).sum()),
        odomPointsMatchingBestposWithin1mm=int(best_match.sum()), bestposMatchingIntervals=intervals,
        mappingFramesUsingBestposPosition=mixed_map_frames,
        pointsOver01MFromNearbyPva=int((closest["pva"]>.1).sum()),
        thosePointsMatchingBestposWithin1mm=int(((closest["pva"]>.1)&best_match).sum()),
        maximumOldVsPvaReferenceDifferenceM=float(source_difference.max()),
        observerRmseAgainstPva=float(np.sqrt(np.mean(np.sum((global_xy-alternate)**2,axis=1)))),
        matcherRmseAgainstPva=float(np.sqrt(np.mean(np.sum((scan_xy-scan_alt)**2,axis=1)))),
        top1PercentMseShare=float(np.sort(mse)[-int(np.ceil(.01*len(mse))):].sum()/mse.sum()),
        top5PercentMseShare=float(np.sort(mse)[-int(np.ceil(.05*len(mse))):].sum()/mse.sum()),
        partitions=partitions,
        interpretation="Observed raw position source substitutions; published driver mechanism is consistent, deployed version/config not recovered. Alternate PVA is not independent truth."
    )
    # Verify the three largest suspect native ODOM transitions against bag bytes.
    spec = importlib.util.spec_from_file_location("extract_gnss", ROOT / "scripts/extractGnssFromBag.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    targets = np.unique(np.r_[np.argsort(implied_speed)[-3:], np.argsort(implied_speed)[-3:]+1])
    compared = []
    with Reader(ROOT / "data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag") as reader:
        topic = "/novatel/oem7/odom"
        store = module.build_typestore(reader, {topic})
        conns = [c for c in reader.connections if c.topic==topic]
        for j in targets:
            time_ns = round(float(odom["bag_time_sec"][j])*1e9)
            found = False
            for conn, stamp, raw in reader.messages(connections=conns,start=time_ns-2000,stop=time_ns+2001):
                message = store.deserialize_ros1(raw,conn.msgtype)
                row = module.odom_row(message,stamp,int(odom["index"][j]))
                delta = max(abs(row[name]-float(odom[name][j])) for name in ["x_m","y_m","stamp_sec"])
                if delta<1e-9:
                    compared.append(dict(index=int(odom["index"][j]),maximumExportDifference=delta));found=True;break
            assert found, f"Original bag verification did not resolve row {j}"
    audit["originalBagChecks"] = compared
    with (OUT/"reference_audit.json").open("w") as stream: json.dump(audit,stream,indent=2)
    np.savetxt(OUT/"pva_reference.csv",np.column_stack([pva_time,pva_xy]),delimiter=",",header="time,x,y",comments="")
    np.savetxt(OUT/"reference_comparison.csv",np.column_stack([t,old,alternate,global_xy]),delimiter=",",header="time,odomX,odomY,pvaX,pvaY,observerX,observerY",comments="")
    seeds=read(OUT/"matching_seeds.csv")
    with (OUT/"matching_seed_reference_audit.csv").open("w",newline="") as stream:
        writer=csv.writer(stream,lineterminator="\n");writer.writerow(["frame","referenceSeed","odomDiscrepancyM","pvaDiscrepancyM"])
        for row in seeds:
            j=int(row["frame"])-1
            writer.writerow([int(row["frame"]),int(row["referenceSeed"]),row["positionDiscrepancyM"],float(np.linalg.norm([row["x"]-scan_alt[j,0],row["y"]-scan_alt[j,1]]))])
    print(json.dumps(audit,indent=2))


if __name__ == "__main__":
    main()
