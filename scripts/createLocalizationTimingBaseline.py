#!/usr/bin/env python3
"""Extract baseline functions under distinct names for interleaved timing.

Only function identifiers are renamed. Private helpers come from the same Git
commit; unchanged public helpers and legacy native commands are shared with
the current implementation. Generated snapshots belong under ignored output.
"""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

BASELINE = "883c2e4cb58ea3e90ffbe4e67a15a062b8b43560"
SOURCES = [
    "localization/localizeLidarFrame.m",
    "localization/registerSemanticProbabilityCloud.m",
    "perception/perceiveCoarseProbabilityCloud.m",
    "perception/perceiveFrame.m",
    "perception/aggregatePlanarCellMoments.m",
    "perception/groundFeatures/analyzeGroundPillars.m",
    "perception/groundFeatures/buildCurbEnergyMaps.m",
    "perception/offGroundFeatures/analyzeStructuralPillars.m",
]
PRIVATE_FOLDERS = [
    "perception/private",
    "perception/groundFeatures/private",
    "perception/offGroundFeatures/private",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    destination = args.output.resolve()
    names = {Path(p).stem: Path(p).stem + "LatencyBaseline" for p in SOURCES}
    pattern = re.compile(r"\b(" + "|".join(names) + r")\b")
    provenance = []

    def read_git(path):
        return subprocess.check_output(
            ["git", "show", f"{BASELINE}:{path}"], cwd=root
        )

    for source in SOURCES:
        original = read_git(source)
        text = pattern.sub(lambda match: names[match.group()], original.decode())
        target = destination / Path(source).parent / (names[Path(source).stem] + ".m")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)
        provenance.append({"path": source, "originalSha256": hashlib.sha256(original).hexdigest()})

    private_paths = subprocess.check_output(
        ["git", "ls-tree", "-r", "--name-only", BASELINE, "--", *PRIVATE_FOLDERS],
        cwd=root, text=True,
    ).splitlines()
    for source in private_paths:
        target = destination / source
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(read_git(source))
    manifest = {
        "baseline": BASELINE,
        "renamedIdentifiers": names,
        "sources": provenance,
        "privateHelpers": private_paths,
        "method": "Rename function identifiers only; share unchanged public helpers and legacy native commands.",
    }
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
