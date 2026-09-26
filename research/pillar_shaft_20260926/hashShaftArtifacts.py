"""Identify independent technical artifacts; no weekly/archive report hashes."""

import csv
import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FOLDER = Path(__file__).resolve().parent
OUTPUT = FOLDER / "artifact_hashes.csv"
PRODUCTION = [
    "config/pillarShaftConfig.m",
    "config/structuralPillarConfig.m",
    "perception/offGroundFeatures/findPillarShaftModes.m",
    "perception/offGroundFeatures/assignPillarShaftSupport.m",
    "perception/offGroundFeatures/analyzeStructuralPillars.m",
    "perception/offGroundFeatures/detectPolePillars.m",
    "perception/native/pillarShaftKernel.hpp",
    "perception/native/poleSubsetKernel.hpp",
    "perception/native/perceptionKernelsMex.cpp",
    "perception/perceptionNativeAvailable.m",
    "scripts/buildPerceptionKernels.m",
    "tests/pillarShaftModesTest.m",
    "tests/wholePillarPerceptionTest.m",
]
INPUTS = [
    "data/raw/MissisipiPointClouds.mat",
    "data/raw/downTownPointClouds.mat",
    "output/coarse_lattice_20260924/distributionEvidenceFull_sequence.mat",
    "output/coarse_lattice_20260924/baseline_sequence.mat",
    "output/coarse_lattice_20260924/fine_pole_pillars.mat",
    "output/pole_distribution_20260925/point_distributions.mat",
    "output/pole_miss_analysis_20260925/cases.mat",
]


def main():
    paths = {ROOT / name: "implementation/test source" for name in PRODUCTION}
    paths.update({ROOT / name: "local input/reference" for name in INPUTS})
    paths.update({p: "local replay/trial output" for p in
                  (ROOT / "output/pillar_shaft_20260926").glob("*.mat")})
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
