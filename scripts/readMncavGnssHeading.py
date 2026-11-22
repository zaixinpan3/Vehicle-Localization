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
        connection, = [c for c in reader.connections if c.topic == TOPIC]
        store = get_typestore(Stores.EMPTY)
        store.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        for _, _, raw in reader.messages(connections=[connection]):
            msg = store.deserialize_ros1(raw, connection.msgtype)
            h = msg.nov_header
            rows.append((h.gps_week_number, h.gps_week_milliseconds * 1e-3,
                         msg.heading, msg.heading_stdev, msg.sol_status.status, msg.pos_type.type))
    return np.array(rows)


def pose_heading():
    stem = "raw_data_2024-06-07-12-09-31_0"
    folder = ROOT / "data/raw/Missisipi"
    prepared = ROOT / "output/receiver_synchronized_inputs"
    meta = json.loads((prepared / "bestpos_metadata.json").read_text())
    fix = np.genfromtxt(prepared / "bestpos.csv", names=True, delimiter=",")
    native = np.genfromtxt(folder / "gnss" / (stem + "_bestpos.csv"), names=True, delimiter=",")
    calibration = json.loads((ROOT / "config/mncavGnssHeading.json").read_text())
    assert np.array_equal(native["stamp_sec"], fix["sourceStamp"]) or np.allclose(
        native["stamp_sec"], fix["sourceStamp"], rtol=0, atol=1e-6)
    week, seconds, baseline, sigma, status, typ = read_heading2(folder / (stem + ".bag")).T
    # Express each receiver timestamp relative to the same BESTPOS epoch as the map clock.
    time = (week-native["gps_week"][0])*604800 + (seconds-native["gps_seconds"][0]) + fix["time"][0]
    valid = (status==0) & (typ!=0) & np.isfinite(baseline) & np.isfinite(sigma)
    valid &= (time>=fix["time"][0]) & (time<=fix["time"][-1])
    if not np.all(np.diff(time)>0):
        raise ValueError("HEADING2 receiver timestamps must increase")
    lon = np.interp(time, fix["time"], native["longitude_deg"])
    lat = np.interp(time, fix["time"], native["latitude_deg"])
    convergence = Proj("EPSG:32615").get_factors(lon, lat).meridian_convergence
    # HEADING2 is clockwise from true north along the base-to-rover baseline.
    body = baseline - calibration["clockwiseBodyToReportedBaselineDeg"]
    yaw = np.deg2rad(90-body+convergence)
    return dict(time=time.tolist(), heading=np.where(valid,np.arctan2(np.sin(yaw),np.cos(yaw)),0).tolist(),
                valid=valid.tolist(), delay=0, maximumBracketSeconds=1.5*float(np.median(np.diff(time))),
                source=TOPIC, clockModelId=meta["clock"]["modelId"],
                computedSamples=int((status==0).sum()), validSamples=int(valid.sum()),
                medianValidReportedStdDeg=float(np.median(sigma[valid])))


if __name__ == "__main__":
    print(json.dumps(pose_heading(),allow_nan=False))
