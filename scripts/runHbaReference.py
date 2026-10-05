#!/usr/bin/env python3
"""Run isolated official HBA on prepared scans and export audited references.

The image must contain the standalone build at /hba/build/hba and its PCL/GTSAM
dependencies. Input pose.json is preserved as initial_pose.txt before HBA writes
its solution. A refinement is never accepted on optimizer exit status alone.
"""
import argparse
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

from runOfflineReference import export_reference, sha256


def run(args):
    if args.threads < 1:
        raise ValueError("Thread count must be positive")
    docker = (["sudo", "-n"] if args.sudo else []) + [args.docker]
    if args.docker_host:
        docker += ["-H", args.docker_host]
    image_id = subprocess.check_output(docker + ["image", "inspect", args.image, "--format", "{{.Id}}"], text=True).strip()
    work = args.scans.resolve()
    initial = np.loadtxt(work / "initial_lidar.tum", ndmin=2)
    # Require enough scans at the highest layer (GAP=5, WIN_SIZE=10).
    if len(initial) // (5 ** (args.layers - 1)) < 10:
        raise ValueError("Hierarchy too deep for scan count")
    if (work / "hba_run.json").exists():
        raise ValueError("Preserve prior run; prepare a fresh scans directory")
    shutil.copyfile(work / "pose.json", work / "initial_pose.txt")
    command = docker + ["run", "--rm", "--network", "none", "--entrypoint", "/hba/build/hba",
                        "--user", f"{os.getuid()}:{os.getgid()}", "-v", f"{work}:/data", image_id,
                        "/data/", str(args.layers), str(args.threads)]
    record = {"image_id": image_id, "command": command, "layers": args.layers, "threads": args.threads,
              "initial_sha256": sha256(work / "initial_lidar.tum"), "navigation_pose_used": False}
    start = time.monotonic()
    with (work / "hba.log").open("w") as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
    record.update(exit_code=result.returncode, elapsed_seconds=time.monotonic() - start, status="failed")
    try:
        if result.returncode:
            raise RuntimeError(f"HBA exited {result.returncode}")
        poses = np.loadtxt(work / "pose.json", ndmin=2)
        if poses.shape != (len(initial), 7) or not np.isfinite(poses).all():
            raise ValueError("HBA output missing, nonfinite, or wrong pose count")
        trajectory = np.column_stack([initial[:, 0], poses[:, :3], poses[:, 4:], poses[:, 3]])
        (work / "dump").mkdir()
        np.savetxt(work / "dump/traj_lidar.txt", trajectory, fmt="%.12f")
        quality = export_reference(args.prepared, work, "GLIM initialization + HBA LiDAR bundle adjustment", "hba.log")
        record.update(status="completed", quality=quality)
    except Exception as error:
        record["error"] = str(error)
    (work / "hba_run.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))
    if record["status"] == "failed":
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--scans", type=Path, required=True)
    parser.add_argument("--image", default="pdvl/hba-standalone:v2-20261005")
    parser.add_argument("--layers", type=int, choices=[2, 3], default=3)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--docker", default="docker")
    parser.add_argument("--docker-host")
    parser.add_argument("--sudo", action="store_true")
    run(parser.parse_args())
