"""Compare INSPVA course over ground with INSPVA azimuth on straight driving.

Run with python3 (NumPy only) from the repository root. Uses native INSPVA and
INSPVAX messages only; no wheel, IMU, LiDAR or localization output enters.
The output is evaluation data and must not enter a production observer.
"""
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
DRIVES = [
    ("12-09-31 evaluation drive", "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv",
     ("azimuth_deg", "north_velocity_mps", "east_velocity_mps")),
    ("12-11-24 calibration drive", "output/mncav_wheel_only_20260916/calibration_sensors/inspva.csv",
     ("azimuth", "north_velocity", "east_velocity")),
]
INSPVAX = "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspvax.csv"
TYPES = {53: "INS_PSRSP", 54: "INS_PSRDIFF", 55: "INS_RTKFLOAT", 56: "INS_RTKFIXED"}


def course_offset(path, columns):
    data = np.genfromtxt(ROOT / path, delimiter=",", names=True)
    azimuth, north, east = (data[name] for name in columns)
    gps = data["gps_seconds"]
    time = gps - gps[0]
    speed = np.hypot(east, north)
    # Azimuth is clockwise from north. Positive: course lies left of azimuth.
    offset = (azimuth - np.rad2deg(np.arctan2(east, north)) + 180) % 360 - 180
    rate = np.rad2deg(np.gradient(np.unwrap(np.deg2rad(azimuth)), time))
    straight = (speed >= 5) & (np.abs(rate) <= 1)
    bins = []
    for begin in range(0, int(time[-1]) + 1, 10):
        mask = straight & (time >= begin) & (time < begin + 10)
        if mask.sum() >= 25:
            bins.append(dict(beginSeconds=begin, endSeconds=begin + 10, samples=int(mask.sum()),
                             medianSpeedMps=float(np.median(speed[mask])),
                             medianCourseMinusAzimuthDeg=float(np.median(offset[mask]))))
    quartiles = np.percentile(offset[straight], [25, 50, 75])
    return dict(source=path, firstGpsSeconds=float(gps[0]), lastGpsSeconds=float(gps[-1]),
                samples=int(len(time)), straightSamples=int(straight.sum()),
                straightDefinition="speed >= 5 m/s and |azimuth rate| <= 1 deg/s",
                courseMinusAzimuthDeg=dict(q25=float(quartiles[0]), median=float(quartiles[1]),
                                           q75=float(quartiles[2]), mean=float(offset[straight].mean())),
                bins=bins)


def solution_quality(origin):
    data = np.genfromtxt(ROOT / INSPVAX, delimiter=",", names=True)
    time = data["gps_seconds"] - origin
    transitions, previous = [], None
    for k, kind in enumerate(data["position_type"].astype(int)):
        if kind != previous:
            transitions.append(dict(timeSeconds=float(time[k]), positionType=int(kind), name=TYPES.get(int(kind), "other")))
            previous = kind
    rows = [dict(timeSeconds=float(time[k]), positionType=int(data["position_type"][k]),
                 latitudeStdM=float(data["latitude_stdev_m"][k]), longitudeStdM=float(data["longitude_stdev_m"][k]),
                 azimuthStdDeg=float(data["azimuth_stdev_deg"][k]))
            for k in range(len(time)) if k % 10 == 0 or 104 <= time[k] <= 112]
    return dict(source=INSPVAX, timeOrigin="first INSPVA message of the evaluation drive",
                transitions=transitions, samples=rows)


def main():
    drives = {name: course_offset(path, columns) for name, path, columns in DRIVES}
    evaluation = drives[DRIVES[0][0]]
    report = dict(drives=drives, evaluationSolution=solution_quality(evaluation["firstGpsSeconds"]),
                  gapBetweenDrivesSeconds=drives[DRIVES[1][0]]["firstGpsSeconds"] - evaluation["lastGpsSeconds"],
                  scope="Native receiver messages only; does not determine whether azimuth or course is physically correct")
    (DEST / "reference_course_audit.json").write_text(json.dumps(report, indent=2) + "\n")
    for name, drive in drives.items():
        print(name, drive["straightSamples"], drive["courseMinusAzimuthDeg"])
    print("gap between drives", report["gapBetweenDrivesSeconds"])
    print(report["evaluationSolution"]["transitions"])


if __name__ == "__main__":
    main()
