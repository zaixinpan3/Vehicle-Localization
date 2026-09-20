"""Extract the exact historical runtime into ignored experiment output."""
from pathlib import Path
import hashlib
import io
import json
import subprocess
import tarfile

REVISION = "d6502080b392e6d368696eeb0b7fd755f556752b"


def main():
    repo = Path(__file__).resolve().parents[2]
    output = repo / "output/reference_free_785_20260920"
    snapshot = output / "historical_runtime"
    # Refuse to overwrite an existing experimental runtime.
    snapshot.mkdir(parents=True, exist_ok=False)
    archive = subprocess.check_output([
        "git", "archive", REVISION, "setupVehicleLocalization.m", "config",
        "localization", "mapping", "perception", "scripts", "tests",
    ], cwd=repo)
    with tarfile.open(fileobj=io.BytesIO(archive)) as source:
        source.extractall(snapshot, filter="data")
    for name in ("data", "output"):
        (snapshot / name).symlink_to(repo / name, target_is_directory=True)
    metadata = {
        "revision": REVISION,
        "snapshot": str(snapshot.relative_to(repo)),
        "scope": "Unmodified historical runtime extracted from Git for this experiment only; not an active alternative implementation.",
        "archiveSha256": hashlib.sha256(archive).hexdigest(),
    }
    (output / "runtime_snapshot.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
