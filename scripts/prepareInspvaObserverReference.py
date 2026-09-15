#!/usr/bin/env python3
"""Export native-rate INSPVA for evaluation of the new map's observer replay.

Run with uv run --offline --with numpy --with pyproj python
scripts/prepareInspvaObserverReference.py. This file is evaluation data only.
"""
from pathlib import Path
import csv
import hashlib
import json

import numpy as np
from pyproj import Proj, Transformer


def main():
    root = Path(__file__).resolve().parents[1]
    folder = root / "data/raw/Missisipi/gnss"
    stem = "raw_data_2024-06-07-12-09-31_0"
    source = folder / (stem + "_inspva.csv")
    frames = folder / (stem + "_front_lidar_points.csv")
    ins = np.genfromtxt(source, delimiter=",", names=True)
    lidar = np.genfromtxt(frames, delimiter=",", names=True)
    ros = ins["stamp_sec"] - ins["stamp_sec"][0]
    receiver = ins["gps_seconds"] - ins["gps_seconds"][0]
    start = np.interp(lidar["stamp_sec"][0] - ins["stamp_sec"][0], ros, receiver)
    lon, lat = ins["longitude_deg"], ins["latitude_deg"]
    east, north = Transformer.from_crs(4326, 32615, always_xy=True).transform(lon, lat)
    gamma = np.asarray(Proj("EPSG:32615").get_factors(lon, lat).meridian_convergence)
    yaw = np.unwrap(np.deg2rad(90 - ins["azimuth_deg"] + gamma))
    output = root / "output/mncav_inspva_observer_20260915"
    output.mkdir(parents=True, exist_ok=True)
    values = np.column_stack((receiver - start, east, north, ins["height_m"], yaw, ins["ins_status"]))
    assert np.all(np.isfinite(values)) and np.all(np.diff(values[:, 0]) > 0)
    with (output / "native_reference.csv").open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["time", "x", "y", "z", "psi", "ins_status"])
        writer.writerows(values)
    metadata = {"samples": len(values), "role": "Evaluation only; never an observer measurement or reset",
                "coordinates": "EPSG:32615, ellipsoidal height, unwrapped grid yaw",
                "inputs": [{"path": str(p.relative_to(root)), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                           for p in (source, frames)],
                "sha256": hashlib.sha256((output / "native_reference.csv").read_bytes()).hexdigest()}
    (output / "reference_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
