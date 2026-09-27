"""Audit frozen original-frame fine labels without executing perception."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import subprocess

import h5py
import numpy as np
from scipy.io import loadmat


ROOT = Path(__file__).resolve().parents[2]
FOLDER = Path(__file__).resolve().parent
FINE_REVISION = "48d043b083ccc2fb44c19726655223b1d5054484"
LATTICE_REVISION = "796c737344e6c29c3e7aa8869aab2630dc0ebbb1"


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main() -> None:
    manifest = json.loads((ROOT / "research/fine_matching_20260919/artifact_hashes.json").read_text())
    expected_hashes = {item["path"]: item["sha256"] for item in manifest["artifacts"]}
    assert manifest["runtimeRevision"] == FINE_REVISION
    sparse_path = ROOT / "output/coarse_lattice_20260924/fine_pole_pillars.mat"
    sparse = loadmat(sparse_path, simplify_cells=True)["fine"]
    sparse_points = {
        int(row["frame"]): np.asarray(row["polePoints"], dtype=np.int64).reshape(-1)
        for row in sparse
    }
    assert sorted(sparse_points) == list(range(1, 1171, 10))
    references: dict[int, np.ndarray] = {}
    partitions = []
    mismatches = []
    for worker in range(1, 5):
        relative = f"output/fine_matching_20260919/inputs_{worker}.mat"
        path = ROOT / relative
        digest = sha256(path)
        assert digest == expected_hashes[relative], f"Changed frozen partition: {relative}"
        with h5py.File(path, "r") as source:
            frames = source["frames"][...].reshape(-1).astype(int)
            assert int(source["processed"][0, 0]) == len(frames)
            assert source["selectedIndices"].shape == (3, len(frames))
            point_count = 0
            # MATLAB cfg.featureNames is [curb,pole,trafficSign]; the exact
            # sparse-cache overlap below independently checks this column.
            for index, frame in enumerate(frames):
                values = source[source["selectedIndices"][1, index]]
                points = (
                    np.empty(0, dtype=np.int64)
                    if "MATLAB_empty" in values.attrs
                    else values[...].reshape(-1).astype(np.int64)
                )
                assert points.size == source["counts"][1, index]
                assert np.all(points >= 1) and len(np.unique(points)) == len(points)
                assert int(frame) not in references
                references[int(frame)] = points
                point_count += len(points)
                if frame in sparse_points and not np.array_equal(points, sparse_points[frame]):
                    mismatches.append(int(frame))
        partitions.append({
            "path": relative, "sha256": digest, "matchesOriginalManifest": True,
            "firstFrame": int(frames[0]), "lastFrame": int(frames[-1]),
            "frames": len(frames), "finePolePoints": point_count,
        })
    assert sorted(references) == list(range(1, 1171))
    assert not mismatches, f"Fine index cache mismatch in frames: {mismatches}"
    changed = subprocess.check_output([
        "git", "diff", "--name-only", FINE_REVISION, LATTICE_REVISION, "--",
        "perception", "config/finePerceptionConfig.m", "config/structuralPillarConfig.m",
        "config/groundSegmentationConfig.m", "config/pillarGridConfig.m",
    ], cwd=ROOT, text=True).splitlines()
    report = {
        "referenceKind": "Frozen original-frame pole indices from offline 0.3 m fine perception",
        "fineRuntimeRevision": FINE_REVISION,
        "preMigrationRevision": LATTICE_REVISION,
        "changedPerceptionOrFineStructuralSourcesBetweenRevisions": changed,
        "allPartitionsMatchOriginalHashes": True,
        "frames": len(references),
        "finePolePointCountBeforeRoi": sum(len(points) for points in references.values()),
        "development": {"firstFrame": 1, "lastFrame": 780, "frames": 780,
                        "finePolePointCountBeforeRoi": sum(len(p) for f, p in references.items() if f <= 780)},
        "temporalEvaluation": {"firstFrame": 781, "lastFrame": 1170, "frames": 390,
                               "finePolePointCountBeforeRoi": sum(len(p) for f, p in references.items() if f > 780)},
        "sparseReference": {"path": str(sparse_path.relative_to(ROOT)), "sha256": sha256(sparse_path),
                            "frames": len(sparse_points), "pointIndexMismatchFrames": mismatches,
                            "comparison": "Exact ordered original-frame point-index equality"},
        "partitions": partitions,
        "roi": "No ROI filtering is applied by this provenance audit; metrics project original coordinates separately",
        "limitations": [
            "Fine labels are the designated algorithmic reference, not independent manual truth",
            "Temporal evaluation frames have appeared in earlier studies; they are frozen against new tuning only",
            "Downtown exact point references require an independently frozen point-mask capture",
        ],
    }
    (FOLDER / "fine_reference_provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    with (FOLDER / "fine_reference_frame_counts.csv").open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["frame", "split", "finePointCountBeforeRoi", "sparseCacheOverlap"])
        for frame, points in sorted(references.items()):
            writer.writerow([frame, "development" if frame <= 780 else "temporalEvaluation",
                             len(points), int(frame in sparse_points)])
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
