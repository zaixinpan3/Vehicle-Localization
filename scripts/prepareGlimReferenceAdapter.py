#!/usr/bin/env python3
"""Prepare a narrowly scoped external GLIM end-of-recording adapter.

The CT frontend inherits an empty get_remaining_frames implementation, losing
its active smoothing window from global mapping at EOF. Return actual final
estimates using the same callback pattern as the upstream IMU frontend. No
measurement, objective, optimization or initialization equation is changed.
The output must remain outside the repository's committed source tree.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def prepare(source, output):
    header = source / "include/glim/odometry/odometry_estimation_ct.hpp"
    cpp = source / "src/glim/odometry/odometry_estimation_ct.cpp"
    original_header, original_cpp = header.read_text(), cpp.read_text()
    declaration = "  virtual bool requires_imu() const override { return false; }"
    if original_header.count(declaration) != 1 or "OdometryEstimationCT::get_remaining_frames" in original_cpp:
        raise ValueError("Unsupported or already adapted upstream CT frontend")
    patched_header = original_header.replace(declaration, declaration + "\n\n  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;")
    marker = "OdometryEstimationCT::~OdometryEstimationCT() {}"
    if original_cpp.count(marker) != 1:
        raise ValueError("Unsupported upstream CT destructor marker")
    function = """

std::vector<EstimationFrame::ConstPtr> OdometryEstimationCT::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> remaining;
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    if (frames[i]) {
      remaining.push_back(frames[i]);
    }
  }
  marginalized_cursor = frames.size();
  Callbacks::on_marginalized_frames(remaining);
  return remaining;
}
"""
    patched_cpp = original_cpp.replace(marker, marker + function)
    output.mkdir(parents=True, exist_ok=False)
    for relative, content in [(header.relative_to(source), patched_header), (cpp.relative_to(source), patched_cpp)]:
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    dockerfile = """FROM koide3/glim_ros2@sha256:a6f9ba006358b87867ada08881566d2f8ae9fde53c823eabe4f41266087cf57a
SHELL ["/bin/bash", "-c"]
COPY include/glim/odometry/odometry_estimation_ct.hpp /root/ros2_ws/src/glim/include/glim/odometry/odometry_estimation_ct.hpp
COPY src/glim/odometry/odometry_estimation_ct.cpp /root/ros2_ws/src/glim/src/glim/odometry/odometry_estimation_ct.cpp
RUN source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && cmake --build /root/ros2_ws/build/glim --target odometry_estimation_ct -j2 && cp /root/ros2_ws/build/glim/libglim.so /root/ros2_ws/install/glim/lib/libglim.so && cp /root/ros2_ws/build/glim/libodometry_estimation_ct.so /root/ros2_ws/install/glim/lib/libodometry_estimation_ct.so
COPY adapter_manifest.json /opt/pdvl-glim-adapter.json
"""
    (output / "Dockerfile").write_text(dockerfile)
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    manifest = dict(upstream_commit=revision, change="Return estimated CT active-window frames at EOF; no algorithmic equations changed",
                    hashes={str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest() for p in [header, cpp]},
                    adapted_hashes={str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest() for p in output.rglob("*") if p.is_file()})
    (output / "adapter_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    prepare(args.source, args.output)
