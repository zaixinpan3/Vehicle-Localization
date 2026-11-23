"""Read native dual-antenna HEADING2 as map-frame vehicle yaw for the Mississippi replay."""
import json
from pathlib import Path

import numpy as np
from pyproj import Proj
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = Path(__file__).resolve().parents[1]
TOPIC = "/novatel/oem7/heading2"


def read_heading2(bag):
    rows = []
    with Reader(bag) as reader:
        connections = [c for c in reader.connections if c.topic == TOPIC]
        if not connections:
            return np.empty((0, 6))
        store = get_typestore(Stores.EMPTY)
        for connection in connections:
            store.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        for connection, _, raw in reader.messages(connections=connections):
            msg = store.deserialize_ros1(raw, connection.msgtype)
            h = msg.nov_header
            rows.append((h.gps_week_number, h.gps_week_milliseconds * 1e-3,
                         msg.heading, msg.heading_stdev, msg.sol_status.status, msg.pos_type.type))
    return np.asarray(rows).reshape(-1, 6)


def pose_heading():
    stem = "raw_data_2024-06-07-12-09-31_0"
    folder = ROOT / "data/raw/Missisipi"
    prepared = ROOT / "output/receiver_synchronized_inputs"
    meta = json.loads((prepared / "bestpos_metadata.json").read_text())
    fix = np.genfromtxt(prepared / "bestpos.csv", names=True, delimiter=",")
    native = np.genfromtxt(folder / "gnss" / (stem + "_bestpos.csv"), names=True, delimiter=",")
    assert np.allclose(native["stamp_sec"], fix["sourceStamp"], rtol=0, atol=1e-6)
    return heading_packet(folder / (stem + ".bag"), native["gps_week"][0],
                          native["gps_seconds"][0], fix["time"], native["longitude_deg"],
                          native["latitude_deg"], meta["clock"]["modelId"])


def heading_packet(bag, first_week, first_seconds, fix_time, longitude, latitude, clock_id):
    calibration = json.loads((ROOT / "config/mncavGnssHeading.json").read_text())
    week, seconds, baseline, sigma, status, typ = read_heading2(bag).T
    time = (week - first_week) * 604800 + (seconds - first_seconds) + fix_time[0]
    valid = (status==0) & (typ!=0) & np.isfinite(baseline) & np.isfinite(sigma)
    valid &= (time>=fix_time[0]) & (time<=fix_time[-1])
    if not np.all(np.diff(time)>0):
        raise ValueError("HEADING2 receiver timestamps must increase")
    heading = np.zeros_like(time)
    if valid.any():
        lon = np.interp(time[valid], fix_time, longitude)
        lat = np.interp(time[valid], fix_time, latitude)
        convergence = Proj("EPSG:32615").get_factors(lon, lat).meridian_convergence
        # HEADING2 is clockwise from true north along the base-to-rover baseline.
        body = baseline[valid] - calibration["clockwiseBodyToReportedBaselineDeg"]
        yaw = np.deg2rad(90-body+convergence)
        heading[valid] = np.arctan2(np.sin(yaw), np.cos(yaw))
    packet = dict(time=time.tolist(), heading=heading.tolist(),
                  valid=valid.tolist(), delay=0, source=TOPIC, clockModelId=clock_id,
                  computedSamples=int((status==0).sum()), validSamples=int(valid.sum()))
    if len(time) > 1:
        packet["maximumBracketSeconds"] = 1.5*float(np.median(np.diff(time)))
    if valid.any():
        packet["medianValidReportedStdDeg"] = float(np.median(sigma[valid]))
    return packet


if __name__ == "__main__":
    print(json.dumps(pose_heading(),allow_nan=False))
