"""Run pinned FAST-LIO2 native and prior-map controls; keep arrays temporary."""

import argparse
import json
import os
import subprocess
from pathlib import Path

from referenceSupport import reference
from scoreTrajectory import score


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--temporary", type=Path, required=True)
    p.add_argument("--executable", type=Path, required=True)
    p.add_argument("--deps", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--drive", type=int, required=True)
    p.add_argument("--mode", choices=["native", "prior"], required=True)
    p.add_argument("--voxel", type=float, default=0.5)
    p.add_argument("--label", default="")
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    folder = a.temporary / f"drive{a.drive}"
    ref = reference(a.drive, a.temporary)
    name = f"drive{a.drive}_{a.mode}_v{a.voxel:g}" + a.label
    trajectory = folder / (name + ".csv")
    command = [
        str(a.executable),
        str(folder / "scans.bin"),
        str(folder / "imu.bin"),
        str(trajectory),
        str(a.voxel),
        "2",
        str(folder / "prior.bin") if a.mode == "prior" else "-",
        str(ref["anchorFrame"]),
    ]
    environment = dict(os.environ)
    environment["LD_LIBRARY_PATH"] = str(a.deps / "lib")
    environment["OMP_NUM_THREADS"] = "2"
    with (a.output / (name + ".log")).open("w") as log:
        result = subprocess.run(
            command, env=environment, stdout=log, stderr=subprocess.STDOUT, check=False
        )
    if result.returncode:
        raise RuntimeError(
            f"FAST-LIO trial failed with status {result.returncode}: {name}"
        )
    metrics, _ = score(a.drive, a.temporary, trajectory)
    metrics.update(mode=a.mode, voxelM=a.voxel, command=command)
    (a.output / (name + ".json")).write_text(json.dumps(metrics, indent=2) + "\n")
    print(name, metrics, flush=True)


if __name__ == "__main__":
    main()
