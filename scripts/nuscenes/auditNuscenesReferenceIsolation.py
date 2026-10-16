#!/usr/bin/env python3
"""Poison later query reference poses while preserving map/initialization poses.

Compare two MATLAB replays using one freshly solved in-memory gain design.
The source dataset is read-only. Temporary metadata and output arrays are not
retained; the mutation recipe, small comparison report and run log are kept.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def read_json(path):
    return json.loads(path.read_text())


def select_poses(dataset, scene_names):
    metadata = dataset / "v1.0-mini"
    scenes = read_json(metadata / "scene.json")
    samples = read_json(metadata / "sample.json")
    frames = read_json(metadata / "sample_data.json")
    sample_by_token = {row["token"]: row for row in samples}
    protected, targets = set(), set()
    selections = []
    for name in scene_names:
        scene = next(row for row in scenes if row["name"] == name)
        sample_tokens = {row["token"] for row in samples if row["scene_token"] == scene["token"]}
        first = sample_by_token[scene["first_sample_token"]]["timestamp"]
        last = sample_by_token[scene["last_sample_token"]]["timestamp"]
        lidar = sorted((row for row in frames if row["sample_token"] in sample_tokens
                        and "LIDAR_TOP" in row["filename"] and first <= row["timestamp"] <= last),
                       key=lambda row: row["timestamp"])
        can = dataset / "can_bus"
        imu = read_json(can / f"{name}_ms_imu.json")
        wheel = read_json(can / f"{name}_zoe_veh_info.json")
        steer = read_json(can / f"{name}_steeranglefeedback.json")
        lo, hi = max(wheel[0]["utime"], steer[0]["utime"]), min(wheel[-1]["utime"], steer[-1]["utime"])
        native = [row["utime"] for row in imu if lo <= row["utime"] <= hi]
        mapping = lidar[::2]
        query = [row for i, row in enumerate(lidar) if i % 2 == 1
                 and native[0] <= row["timestamp"] <= native[-1]]
        assert query, f"No queries in {name}"
        protected.update(row["ego_pose_token"] for row in mapping)
        protected.add(query[0]["ego_pose_token"])
        targets.update(row["ego_pose_token"] for row in query[1:])
        selections.append({"scene": name, "mappingFrames": len(mapping), "queryFrames": len(query),
                           "poisonedQueries": len(query) - 1})
    assert not protected.intersection(targets), "Cannot poison a pose shared with mapping/initialization"
    return protected, targets, selections


def matlab_quote(path):
    return "'" + str(path).replace("'", "''") + "'"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--matlab", default="matlab")
    parser.add_argument("--baseline-output", type=Path,
                        help="Retain normal trial summaries/figures here; otherwise remove them with scratch outputs")
    args = parser.parse_args()
    dataset, output = args.dataset.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[2]
    scenes = ["scene-0061", "scene-0103"]
    protected, targets, selections = select_poses(dataset, scenes)
    original_path = dataset / "v1.0-mini/ego_pose.json"
    original = original_path.read_bytes()
    poses = json.loads(original)
    shift = [1000, -700, 100]
    quaternion = [0.5, 0.5, 0.5, 0.5]
    changed = 0
    for pose in poses:
        if pose["token"] in targets:
            pose["translation"] = [a + b for a, b in zip(pose["translation"], shift)]
            pose["rotation"] = quaternion.copy()
            changed += 1
    assert changed == len(targets)
    recipe = {"scenes": selections, "protectedPoseRecords": len(protected), "poisonedPoseRecords": changed,
              "translationShiftM": shift, "replacementQuaternionWxyz": quaternion,
              "sourceEgoPoseSha256": hashlib.sha256(original).hexdigest(),
              "selection": "One-based odd frames map; even frames with CAN coverage query; protect all map poses and each first query pose",
              "runtimeBoundary": "Later query x/y/z, yaw, roll and pitch are poisoned; raw LiDAR/CAN/calibration/timestamps remain identical"}
    (output / "mutation_recipe.json").write_text(json.dumps(recipe, indent=2) + "\n")
    with tempfile.TemporaryDirectory(prefix="pdvl_nuscenes_reference_audit_") as scratch:
        scratch = Path(scratch)
        poisoned = scratch / "dataset"
        poisoned.mkdir()
        for item in dataset.iterdir():
            if item.name != "v1.0-mini":
                (poisoned / item.name).symlink_to(item, target_is_directory=item.is_dir())
        metadata = poisoned / "v1.0-mini"
        metadata.mkdir()
        for item in (dataset / "v1.0-mini").iterdir():
            if item.name != "ego_pose.json":
                (metadata / item.name).symlink_to(item, target_is_directory=item.is_dir())
        (metadata / "ego_pose.json").write_text(json.dumps(poses))
        baseline_output = args.baseline_output.resolve() if args.baseline_output else scratch / "baseline"
        script = scratch / "run_audit.m"
        script.write_text(f"""cd({matlab_quote(root)});setupVehicleLocalization;
[baseReports,baseline,paired]=runNuscenesOverlapTrial({matlab_quote(dataset)},{matlab_quote(baseline_output)}, ...
 ReferenceAuditDatasetFolder={matlab_quote(poisoned)},ReferenceAuditOutputFolder={matlab_quote(scratch / 'poisoned')});
badReports=paired.reports;poisoned=paired.estimates;
comparisons=cell(numel(baseline),1);
for k=1:numel(baseline)
 a=baseline{{k}};b=poisoned{{k}};
 comparisons{{k}}=struct('scene',baseReports{{k}}.scene,'frames',numel(a.time), ...
  'maximumEightStateDifference',max(abs(a.internalState-b.internalState),[],'all'), ...
  'eightStatesExactlyEqual',isequaln(a.internalState,b.internalState), ...
  'poseExactlyEqual',isequaln(a.pose,b.pose),'lateralOutputsExactlyEqual',isequaln(a.lateral,b.lateral), ...
  'intrinsicPredictionExactlyEqual',isequaln(a.diagnostics.intrinsicPrediction,b.diagnostics.intrinsicPrediction), ...
  'overlapGradientExactlyEqual',isequaln(a.diagnostics.lidarOverlap.gradient,b.diagnostics.lidarOverlap.gradient), ...
  'overlapSimilarityExactlyEqual',isequaln(a.diagnostics.lidarOverlap.similarity,b.diagnostics.lidarOverlap.similarity), ...
  'baselineMetrics',baseReports{{k}}.metrics,'poisonedScoringMetrics',badReports{{k}}.metrics);
end
result=struct('comparisons',{{comparisons}},'randomSeed',20261014,'freshGainSolvePerPairedExperiment',true, ...
 'gainControl',"Share this invocation's freshly solved gains in memory; no persisted gains; reset scene RNG for paired replay", ...
 'scope',"Tests estimator dependence on later query reference fields with map and initialization held fixed; not statistical independence of vendor sensor/reference products");
f=fopen({matlab_quote(output / 'audit.json')},'w');fprintf(f,'%s\\n',jsonencode(result,PrettyPrint=true));fclose(f);
for k=1:numel(comparisons)
 c=comparisons{{k}};disp(c);
 assert(c.eightStatesExactlyEqual && c.poseExactlyEqual && c.lateralOutputsExactlyEqual ...
  && c.intrinsicPredictionExactlyEqual && c.overlapGradientExactlyEqual && c.overlapSimilarityExactlyEqual, ...
  'VehicleLocalization:NuscenesReferenceDependence','Reference poisoning changed an estimator output.');
 assert(c.poisonedScoringMetrics(2).position_rmse_m>100, ...
  'VehicleLocalization:InactiveReferencePoison','Poison did not reach scoring.');
end
disp('NUSCENES_REFERENCE_ISOLATION_AUDIT_PASSED');
""")
        with (output / "audit.log").open("w") as log:
            completed = subprocess.run([args.matlab, "-batch", f"run({matlab_quote(script)})"],
                                       cwd=root, stdout=log, stderr=subprocess.STDOUT, check=False)
        assert original_path.read_bytes() == original, "Source reference data changed"
        completed.check_returncode()
    print(f"Reference isolation passed: {changed} poisoned later reference records; see {output / 'audit.json'}")


if __name__ == "__main__":
    main()
