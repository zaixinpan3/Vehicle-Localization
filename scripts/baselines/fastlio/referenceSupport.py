"""Reference boundary for offline map creation and scoring only."""

import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pyproj import Proj, Transformer
from receiverClock import convert_time, ensure_clock, native_seconds

ROOT = Path(__file__).resolve().parents[3]


def table(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def reference(drive, temporary):
    folder = Path(temporary) / f"drive{drive}"
    if drive == 1:
        gnss = ROOT / "data/raw/Missisipi/gnss"
        stem = "raw_data_2024-06-07-12-09-31_0"
        pvaPath = gnss / (stem + "_inspva.csv")
        pva = table(pvaPath)
        poses = table(ROOT / "output/receiver_reference/corrected_poses.csv")
        native = table(
            ROOT / "output/receiver_reference/corrected_native_reference.csv"
        )
        times = native["time"]
        xyz = np.column_stack([native[k] for k in ["x", "y", "z"]])
        yaw = native["psi"]
    else:
        gnss = folder / "gnss"
        stem = "raw_data_2024-06-07-12-11-24_0"
        pvaPath = gnss / (stem + "_inspva.csv")
        pva = table(pvaPath)
        poses = table(folder / "reference_poses.csv")
        clock = ensure_clock(pvaPath)
        start = convert_time(clock, [poses["lidar_stamp_sec"][0]])[0]
        times = native_seconds(pva["gps_week"], pva["gps_seconds"]) - start
        lon = pva["longitude_deg"]
        lat = pva["latitude_deg"]
        x, y = Transformer.from_crs(4326, 32615, always_xy=True).transform(lon, lat)
        xyz = np.column_stack([x, y, pva["height_m"]])
        gamma = Proj("EPSG:32615").get_factors(lon, lat).meridian_convergence
        yaw = np.unwrap(np.deg2rad(90 - pva["azimuth_deg"] + gamma))
    R = Rotation.from_euler(
        "xyz",
        np.column_stack(
            [np.deg2rad(pva["roll_deg"]), -np.deg2rad(pva["pitch_deg"]), yaw]
        ),
    )
    clock = ensure_clock(pvaPath)
    frames = table(folder / "frames.csv")
    frameTimes = convert_time(clock, frames["rosHeader"])
    frameTimes -= frameTimes[0]
    mount = json.loads((ROOT / "config/mississippiLidarMount.json").read_text())
    calib = json.loads(
        (ROOT / "config/mississippiLidarFrameCalibration.json").read_text()
    )["calibration"]
    rawToRef = (
        np.array(calib["rotation"])
        @ np.array(mount["additionalRotation"])
        @ np.array(mount["baseRotation"])
    )
    rawToRef = Rotation.from_matrix(rawToRef).as_matrix()
    lever = np.array(calib["translation"])
    slerp = Slerp(times, R)

    def at(query):
        q = np.asarray(query)
        clipped = np.clip(q, times[0], times[-1])
        p = np.column_stack([np.interp(q, times, xyz[:, j]) for j in range(3)])
        for mask, index, other in [(q < times[0], 0, 1), (q > times[-1], -1, -2)]:
            if np.any(mask):
                p[mask] = xyz[index] + (q[mask] - times[index])[:, None] * (
                    xyz[index] - xyz[other]
                ) / (times[index] - times[other])
        return p, slerp(clipped)

    anchor = 4 if drive == 1 else int(np.flatnonzero(frameTimes >= 40)[0] + 1)
    p0, r0 = at([frameTimes[anchor - 1]])
    RL0 = r0.as_matrix()[0] @ rawToRef
    pL0 = p0[0] + r0.apply(lever)[0]
    return {
        "at": at,
        "times": times,
        "frameTimes": frameTimes,
        "frames": frames,
        "rawToRef": rawToRef,
        "lever": lever,
        "anchorFrame": anchor,
        "anchorRotation": RL0,
        "anchorPosition": pL0,
        "referenceScope": "Corrected same-drive receiver"
        if drive == 1
        else "Original INSPVA",
        "commonEndFrame": len(frames) - 1,
    }
