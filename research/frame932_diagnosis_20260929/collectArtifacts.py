"""Collect diagnostic metrics, technical hashes, and only this study's windows."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
STUDY = Path(__file__).resolve().parent
OUTPUT = ROOT / "output/frame932_diagnosis_20260929"


def rows(name):
    with (STUDY / name).open(newline="") as stream:
        return list(csv.DictReader(stream))


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write(name, value):
    (STUDY / name).write_text(json.dumps(value, indent=2) + "\n")


if __name__ == "__main__":
    controls = {r["variant"]: r for r in rows("controls.csv")}
    transport = {r["variant"]: r for r in rows("transport_controls.csv")}
    pole = rows("pole_detection_control.csv")[0]
    validation = json.loads((STUDY / "validation.json").read_text())
    assert validation["curb711OtherWeightsAndResidualsPreserved"]
    assert not validation["productionCodeChanged"]
    base = float(controls["production"]["errorM"])
    assert abs(base - 0.159861269865301) < 1e-10
    assert float(transport["translation_transport_oracle"]["errorM"]) < 0.065
    assert float(transport["yaw_transport_oracle"]["errorM"]) > 0.15
    assert float(pole["errorM"]) > base
    source = OUTPUT / "viewer/features_0932.json"
    shutil.copyfile(source, STUDY / "feature_comparison.json")
    viewer = json.loads(source.read_text())
    assert viewer["frame"] == 932 and viewer["markerSize"] == 4
    assert not viewer["featureOverlay"]
    assert viewer["features"]["pole"]["selectedPillarCount"] == 0
    assert viewer["features"]["curb"]["referencePointCount"] == 67
    windows = json.loads(subprocess.check_output(["niri", "msg", "--json", "windows"]))
    matching = [w for w in windows if "Mississippi 932" in w["title"]]
    assert len(matching) == 2, "Both requested study windows must be mapped."
    write("viewer_windows.json", [{k: w[k] for k in
          ["id", "title", "app_id", "workspace_id", "is_focused"]} for w in matching])
    findings = {
        "frame": 932,
        "productionErrorM": base,
        "productionYawErrorDeg": float(controls["production"]["yawErrorDeg"]),
        "productionLateralErrorM": float(controls["production"]["lateralM"]),
        "referenceMotionPoolErrorM": float(controls["reference_motion_pool"]["errorM"]),
        "referenceTranslationPoolErrorM": float(transport["translation_transport_oracle"]["errorM"]),
        "referenceYawPoolErrorM": float(transport["yaw_transport_oracle"]["errorM"]),
        "dropCurb711ErrorM": float(controls["drop_target_711"]["errorM"]),
        "restoredCurrentPoleErrorM": float(pole["errorM"]),
        "productionChanged": False,
        "interpretation": "History translation disagreement and conflicting curb geometry bias the objective; current near-pole recovery alone does not improve this frame.",
        "scope": "Offline single-frame controls on overlapping mapping/query recording; reference interventions cannot run online.",
        "validation": validation,
    }
    write("findings.json", findings)
    inputs = [
        "output/source_shape_matching_20260929/final_raw.mat",
        "output/root_cause_matching_20260929/finalSurface_sources.mat",
        "output/line_direction_matching_20260928/sources.mat",
        "output/line_direction_matching_20260928/production/report.mat",
        "output/mississippi_mapping_calibrated/view_conditioned_cloud.mat",
        "output/mississippi_mapping_calibrated/feature_observations.mat",
        "config/mississippiPolePillarDistributionModel.json",
        "config/mississippiLidarFrameCalibration.json",
        "config/localizationSourceWindowConfig.m",
        "config/pillarPoleDistributionConfig.m",
        "config/distributionRegistrationConfig.m",
        viewer["referenceSource"],
        "output/pole_precision_20260927/mississippi.mat",
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_front_lidar_synchronized_pose_1_1170.csv",
    ]
    write("input_hashes.json", {p: digest(ROOT / p) for p in inputs})
    artifacts = {str(p.relative_to(ROOT)): digest(p) for p in sorted(STUDY.iterdir())
                 if p.is_file() and p.name != "artifact_hashes.json"}
    write("artifact_hashes.json", artifacts)
    print(json.dumps({"findings": findings, "technicalArtifacts": len(artifacts),
                      "studyWindowsMapped": len(matching)}, indent=2))
