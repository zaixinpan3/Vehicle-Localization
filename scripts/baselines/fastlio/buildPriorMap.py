"""Construct a temporary dense prior from reference poses, never online states.

Every scan contributes all finite returns beyond the official blind radius.
Native per-point offsets deskew the OFFLINE map with reference poses; this
is the declared reference-map design. Runtime consumes only the frozen map.
"""

import argparse
import json
import struct
from pathlib import Path

import numpy as np
from referenceSupport import reference


def reduce(keys, sums, counts):
    u, inv = np.unique(keys, axis=0, return_inverse=True)
    total = np.zeros((len(u), 3))
    np.add.at(total, inv, sums)
    cnt = np.bincount(inv, weights=counts)
    return u, total, cnt


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--drive", type=int, required=True)
    p.add_argument("--temporary", type=Path, required=True)
    p.add_argument("--voxel", type=float, default=0.5)
    a = p.parse_args()
    ref = reference(a.drive, a.temporary)
    folder = a.temporary / f"drive{a.drive}"
    keys = np.empty((0, 3), np.int32)
    sums = np.empty((0, 3))
    counts = np.empty(0)
    batch = []
    inputPoints = 0
    edgePoints = 0
    with (folder / "scans.bin").open("rb") as f:
        while header := f.read(24):
            frame, begin, _ros, count = struct.unpack("<IddI", header)
            raw = np.frombuffer(f.read(count * 20), dtype="<f4").reshape(-1, 5)
            xyz = raw[:, :3].astype(float)
            keep = np.isfinite(xyz).all(axis=1) & (np.sum(xyz**2, axis=1) > 16)
            xyz = xyz[keep]
            offset = raw[keep, 4].astype(float) / 1000
            physical = (
                ref["frameTimes"][frame - 1]
                + begin
                + offset
                - ref["frames"]["nativeEnd"][frame - 1]
            )
            edgePoints += int(
                np.count_nonzero(
                    (physical < ref["times"][0]) | (physical > ref["times"][-1])
                )
            )
            unique, inv = np.unique(physical, return_inverse=True)
            pos, R = ref["at"](unique)
            rotation = R.as_matrix()
            pointRef = xyz @ ref["rawToRef"].T + ref["lever"]
            world = np.einsum("nij,nj->ni", rotation[inv], pointRef) + pos[inv]
            local = (world - ref["anchorPosition"]) @ ref["anchorRotation"]
            batch.append(local)
            inputPoints += len(local)
            if frame % 40 == 0 or frame == len(ref["frameTimes"]):
                new = np.concatenate(batch)
                batch = []
                k = np.floor(new / a.voxel).astype(np.int32)
                k, ss, cc = reduce(k, new, np.ones(len(new)))
                keys, sums, counts = reduce(
                    np.vstack([keys, k]), np.vstack([sums, ss]), np.r_[counts, cc]
                )
                print("PRIOR MAP", frame, "voxels", len(keys), flush=True)
    points = (sums / counts[:, None]).astype("<f4")
    with (folder / "prior.bin").open("wb") as f:
        f.write(struct.pack("<Q", len(points)))
        f.write(points.tobytes())
    info = {
        "drive": a.drive,
        "frames": len(ref["frameTimes"]),
        "inputPoints": inputPoints,
        "mapPoints": len(points),
        "voxelM": a.voxel,
        "anchorFrame": ref["anchorFrame"],
        "referenceOnlyForMap": True,
        "onlineReferenceRead": False,
        "edgeTimePoints": edgePoints,
        "edgePolicy": "Linear position extrapolation and held orientation only beyond reference endpoints, within one scan; offline map only",
        "mapCoordinateFrame": "Raw LiDAR axes at the declared initialization reference pose",
    }
    (folder / "prior_summary.json").write_text(json.dumps(info, indent=2) + "\n")
    print(info)


if __name__ == "__main__":
    main()
