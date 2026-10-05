#!/usr/bin/env python3
"""Refine each eligible independent GLIM recording with isolated HBA.

The hierarchy is chosen from scan count; 10--49 covered scans use one global
bundle, with deeper hierarchies on longer recordings. Existing successful or
failed runs are preserved for audit.
Dependencies: rosbags, numpy, scipy, json5; external standalone HBA image.
"""
import argparse
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from prepareHbaReference import prepare
from runHbaReference import run


def batch(args):
    args.output_root.mkdir(parents=True, exist_ok=True)
    statuses = []
    for trajectory in sorted(args.initial_root.glob("*/*/reference_lidar.tum")):
        if args.only and args.only not in str(trajectory):
            continue
        relative = trajectory.parent.relative_to(args.initial_root)
        prepared = args.prepared_root / relative
        output = args.output_root / relative
        count = len(np.loadtxt(trajectory, ndmin=2))
        status = {"sequence": str(relative), "initial_scan_count": count}
        record = output / "hba_run.json"
        if record.exists():
            status.update(status="preserved_existing_run", result=json.loads(record.read_text())["status"])
        elif count < 10:
            status.update(status="insufficient_scans", reason="At least ten fully covered scans required for one layer")
        elif output.exists():
            status.update(status="preserved_incomplete_run", reason="Use a new output root to retry")
        else:
            print(f"Refining {relative}", flush=True)
            start = time.monotonic()
            try:
                prepare(prepared, trajectory, output, args.voxel)
                covered = len(np.loadtxt(output / "initial_lidar.tum", ndmin=2))
                layers = 3 if covered >= 250 else (2 if covered >= 50 else 1)
                run(SimpleNamespace(prepared=prepared, scans=output, layers=layers, threads=args.threads,
                                    docker=args.docker, docker_host=args.docker_host, sudo=args.sudo, image=args.image))
                status.update(status="completed", layers=layers, covered_scans=covered)
            except (Exception, SystemExit) as error:
                status.update(status="failed", error=str(error))
            status["elapsed_seconds"] = time.monotonic() - start
        statuses.append(status)
        (args.output_root / "batch_status.json").write_text(json.dumps(statuses, indent=2) + "\n")
        print(f"{relative}: {status['status']}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepared-root", type=Path, required=True)
    parser.add_argument("--initial-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--image", default="pdvl/hba-standalone:v3-20261005")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--voxel", type=float, default=0.2)
    parser.add_argument("--only", help="Process only sequence paths containing this substring")
    parser.add_argument("--docker", default="docker")
    parser.add_argument("--docker-host")
    parser.add_argument("--sudo", action="store_true")
    batch(parser.parse_args())
