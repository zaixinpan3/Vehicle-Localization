"""Build pinned official FAST-LIO2 math with a minimal offline message adapter.

The upstream functions are copied verbatim into a temporary build directory;
only ROS includes/types and executable transport are replaced. No upstream
source or dependencies are vendored in this repository.
"""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--deps", type=Path, required=True)
    p.add_argument("--build", type=Path, required=True)
    a = p.parse_args()
    a.build.mkdir(parents=True, exist_ok=True)
    source = a.source.resolve()
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    assert revision == "7cc4175de6f8ba2edf34bab02a42195b141027e9", (
        "Use the pinned upstream revision"
    )
    if subprocess.check_output(
        ["git", "-C", str(source), "diff", "HEAD", "--"], text=True
    ):
        raise RuntimeError("Tracked upstream changes are not allowed")
    tree_revision = subprocess.check_output(
        ["git", "-C", str(source / "include/ikd-Tree"), "rev-parse", "HEAD"], text=True
    ).strip()
    assert tree_revision == "e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4"
    build = a.build.resolve()
    deps = a.deps.resolve()
    stub = build / "compat"

    def write(name, text):
        path = stub / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    write(
        "ros/ros.h",
        """#pragma once
#include <cassert>
#include <cstdio>
namespace ros { struct Time { double seconds=0; Time()=default; explicit Time(double s):seconds(s){} double toSec()const{return seconds;} }; }
#define ROS_ASSERT(x) assert(x)
#define ROS_INFO(...) do{fprintf(stderr,__VA_ARGS__);fprintf(stderr,"\\n");}while(0)
#define ROS_WARN(...) ROS_INFO(__VA_ARGS__)
""",
    )
    write(
        "sensor_msgs/Imu.h",
        """#pragma once
#include <memory>
#include <ros/ros.h>
namespace sensor_msgs { struct Imu { using Ptr=std::shared_ptr<Imu>;using ConstPtr=std::shared_ptr<const Imu>;struct {ros::Time stamp;}header;struct V{double x=0,y=0,z=0;};V angular_velocity,linear_acceleration;};using ImuConstPtr=Imu::ConstPtr; }
""",
    )
    write(
        "fast_lio/Pose6D.h",
        """#pragma once
namespace fast_lio { struct Pose6D {double offset_time=0;double acc[3]={},gyr[3]={},vel[3]={},pos[3]={},rot[9]={};};}
""",
    )
    for name in [
        "nav_msgs/Odometry.h",
        "tf/transform_broadcaster.h",
        "eigen_conversions/eigen_msg.h",
        "pcl_conversions/pcl_conversions.h",
        "sensor_msgs/PointCloud2.h",
        "geometry_msgs/Vector3.h",
    ]:
        write(name, "#pragma once\n")
    imu = (
        (source / "src/IMU_Processing.hpp")
        .read_text()
        .replace(
            '#include "preprocess.h"',
            "enum LID_TYPE {AVIA=1, VELO16=2, OUST64=3, MARSIM=4};",
        )
    )
    (build / "official_imu.hpp").write_text(imu)
    s = (source / "src/laserMapping.cpp").read_text()
    parts = []
    globals = s[s.index("#define INIT_TIME") : s.index("void SigHandle")]
    globals = "\n".join(
        line
        for line in globals.splitlines()
        if not any(
            x in line
            for x in ["nav_msgs::", "geometry_msgs::", "shared_ptr<Preprocess>"]
        )
    )
    parts.append(globals)
    for start, end in [
        ("void pointBodyToWorld_ikfom", "void standard_pcl_cbk"),
        ("void map_incremental()", "PointCloudXYZI::Ptr pcl_wait_pub"),
        ("void h_share_model(", "int main("),
    ]:
        parts.append(s[s.index(start) : s.index(end)])
    (build / "official_core.hpp").write_text("\n".join(parts))
    adapter = Path(__file__).with_name("fastLioOffline.cpp").resolve()
    pcl = next((deps / "include").glob("pcl-*"))
    cpp = [
        "g++",
        "-std=c++17",
        "-O2",
        "-DNDEBUG",
        "-DMP_EN",
        "-DMP_PROC_NUM=2",
        f'-DROOT_DIR="{build}/"',
        "-fopenmp",
        "-pthread",
        f"-I{stub}",
        f"-I{build}",
        f"-I{source}/include",
        f"-I{source}/include/ikd-Tree",
        f"-I{deps}/include",
        f"-I{deps}/include/eigen3",
        f"-I{pcl}",
        str(adapter),
        str(source / "include/ikd-Tree/ikd_Tree.cpp"),
        f"-L{deps}/lib",
        f"-Wl,-rpath,{deps}/lib",
        "-lpcl_filters",
        "-lpcl_kdtree",
        "-lpcl_search",
        "-lpcl_common",
        "-llz4",
        "-lgomp",
        "-o",
        str(build / "fastlio_offline"),
    ]
    print("BUILD", cpp, flush=True)
    subprocess.run(cpp, check=True)
    commit = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    manifest = {
        "upstream": commit,
        "ikdTree": tree_revision,
        "adapterSha256": hashlib.sha256(adapter.read_bytes()).hexdigest(),
        "coreSha256": hashlib.sha256(
            (build / "official_core.hpp").read_bytes()
        ).hexdigest(),
        "imuSha256": hashlib.sha256(
            (build / "official_imu.hpp").read_bytes()
        ).hexdigest(),
        "implementation": "Unchanged upstream computation functions and IMU/IKFOM/ikd-Tree; offline transport adapter",
    }
    (build / "build_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
