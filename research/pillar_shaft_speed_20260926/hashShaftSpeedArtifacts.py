"""Hash independent study artifacts; exclude weekly reports and generated binaries."""

import csv
import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FOLDER = Path(__file__).resolve().parent
OUTPUT = FOLDER / "artifact_hashes.csv"
PRODUCTION = [
    "config/pillarShaftConfig.m",
    "perception/native/perceptionKernelsMex.cpp",
    "perception/native/pillarShaftKernel.hpp",
    "perception/offGroundFeatures/findPillarShaftModes.m",
    "perception/offGroundFeatures/assignPillarShaftSupport.m",
    "perception/perceptionNativeAvailable.m",
    "scripts/buildPerceptionKernels.m",
    "tests/pillarShaftExecutionTest.m",
]
INPUTS = [
    "data/raw/MissisipiPointClouds.mat",
    "data/raw/downTownPointClouds.mat",
    "output/pole_miss_analysis_20260925/cases.mat",
    "output/pillar_shaft_20260926/full_pipeline.mat",
    "output/pillar_shaft_20260926/downtown_pipeline.mat",
    "research/pillar_shaft_20260926/uniform_controls.csv",
]


def main():
    paths = {ROOT / name: "implementation/test source" for name in PRODUCTION}
    paths.update({ROOT / name: "local input/reference" for name in INPUTS})
    local = ROOT / "output/pillar_shaft_speed_20260926"
    paths.update({p: "local replay/profile/validation output" for p in local.glob("*.mat")})
    paths.update({p: "frozen baseline source/identity" for p in
                  (local / "baseline_rebuilt").iterdir()
                  if p.suffix in {".m", ".cpp", ".hpp", ".json"}})
    paths.update({p: "study source/result" for p in FOLDER.iterdir()
                  if p.is_file() and p != OUTPUT})
    rows = []
    for path, role in sorted(paths.items()):
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        rows.append({"path": str(path.relative_to(ROOT)), "role": role,
                     "bytes": path.stat().st_size, "sha256": digest})
    with OUTPUT.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["path", "role", "bytes", "sha256"],
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"Identified {len(rows)} independent technical artifacts.")


if __name__ == "__main__":
    main()
