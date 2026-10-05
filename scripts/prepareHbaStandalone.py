#!/usr/bin/env python3
"""Adapt an external official HBA checkout for a standalone, headless build.

No third-party source is vendored. The adapter replaces ROS timing/parameters,
removes unused visualization includes, and fixes EOF parsing and quaternion
serialization of pose files.
It preserves the published optimization algorithm and records source hashes.
Build with PCL, Eigen3 and GTSAM development packages in an isolated container.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path


def adapt(source, output):
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source / "include", output / "include")
    (output / "source").mkdir()
    text = (source / "source/hba.cpp").read_text()
    start = text.index('\tros::init(argc, argv, "hba");')
    end = text.index("  HBA hba(", start)
    text = text[:start] + '''  if (argc != 4) { std::cerr << "usage: hba DATA_PATH LAYERS THREADS\\n"; return 2; }
  string data_path = argv[1];
  if (data_path.back() != '/') data_path += '/';
  int total_layer_num = std::stoi(argv[2]), thread_num = std::stoi(argv[3]);
  pcd_name_fill_num = 6;
''' + text[end:]
    text = '#include "standalone_time.hpp"\n' + text
    (output / "source/hba.cpp").write_text(text)
    for path in [output / "source/hba.cpp", *list((output / "include").glob("*.hpp"))]:
        text = path.read_text()
        text = "\n".join(line for line in text.splitlines()
                         if not any(line.startswith("#include <" + prefix) for prefix in
                                    ["ros/", "sensor_msgs/", "geometry_msgs/", "tf/", "pcl_conversions/", "visualization_msgs/"])) + "\n"
        text = text.replace("ros::Time::now().toSec()", "pdvl_now_seconds()")
        # Extraction must be the loop condition; trailing whitespace must not
        # silently append a duplicate pose, nor a partial final row.
        text = text.replace("while(!file.eof())\n    {\n      file >> tx >> ty >> tz >> w >> x >> y >> z;",
                            "while(file >> tx >> ty >> tz >> w >> x >> y >> z)\n    {")
        # Upstream assigns quaternion components one at a time while repeatedly
        # multiplying the already-mutated quaternion. Evaluate the product once.
        first = "      pose_vec[i].q.w() = (q0.inverse()*pose_vec[i].q).w();"
        if first in text:
            begin = text.index(first)
            end = text.index("      file <<", begin)
            text = text[:begin] + "      pose_vec[i].q = (q0.inverse()*pose_vec[i].q).normalized();\n" + text[end:]
        path.write_text(text)
    (output / "include/standalone_time.hpp").write_text('''#pragma once
#include <chrono>
inline double pdvl_now_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
''')
    (output / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.16)
project(pdvl_hba_standalone LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
add_executable(hba source/hba.cpp)
target_include_directories(hba PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(hba PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen gtsam pthread)
target_compile_options(hba PRIVATE -O3)
''')
    files = sorted([source / "source/hba.cpp", *list((source / "include").glob("*.hpp"))])
    manifest = {"upstream_commit": subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip(),
                "source_sha256": {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                "adaptations": ["ROS parameters replaced by command arguments", "steady clock timing", "unused ROS visualization includes removed", "EOF-safe pose parsing", "atomic quaternion serialization fixes repeated in-place multiplication"],
                "algorithm_modified": False}
    (output / "adapter_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    adapt(args.source, args.output)
