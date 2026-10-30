// SPDX-License-Identifier: GPL-2.0-only
// Offline transport around pinned upstream FAST-LIO2 computational functions.
// The generated official_core.hpp keeps upstream functions byte-for-byte.
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ikd-Tree/ikd_Tree.h>
#include <iomanip>
#include <new>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>

#include "official_imu.hpp"
#include "official_core.hpp"

template <typename T> bool readValue(std::istream &in, T &value) {
  return bool(in.read(reinterpret_cast<char *>(&value), sizeof(T)));
}
struct ImuRecord {
  double t, ax, ay, az, gx, gy, gz;
};
struct RawPoint {
  float x, y, z, intensity, ms;
};
static_assert(sizeof(RawPoint) == 20, "Unexpected packed point layout");

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "usage: fastlio_offline scans.bin imu.bin poses.csv "
                 "[voxel=.5] [filter=2] [prior.bin] [anchorFrame=0]\n";
    return 2;
  }
  try {
    double voxel = argc > 4 ? std::stod(argv[4]) : .5;
    int thinning = argc > 5 ? std::stoi(argv[5]) : 2;
    const bool fixedMap = argc > 6 && std::string(argv[6]) != "-";
    const uint32_t anchor = argc > 7 ? std::stoul(argv[7]) : 0;
    std::ifstream scans(argv[1], std::ios::binary),
        imus(argv[2], std::ios::binary);
    if (!scans || !imus)
      throw std::runtime_error("Missing sensor streams");
    std::vector<ImuRecord> imuData;
    ImuRecord im;
    while (readValue(imus, im))
      imuData.push_back(im);
    if (imuData.empty())
      throw std::runtime_error("Empty IMU");
    std::ofstream out(argv[3]);
    out << std::setprecision(17)
        << "frame,ros_header,native_end,px,py,pz,qx,qy,qz,qw,lx,ly,lz,lqx,lqy,"
           "lqz,lqw,vx,vy,vz,points,matched,map_points,processing_s,preprocess_"
           "s,imu_s,voxel_s,update_s,map_s,prior_active\n";
    NUM_MAX_ITERATIONS = 4;
    filter_size_surf_min = voxel;
    filter_size_map_min = voxel;
    cube_len = 10000;
    DET_RANGE = 150;
    extrinsic_est_en = false;
    omp_set_num_threads(2);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(voxel, voxel, voxel);
    downSizeFilterMap.setLeafSize(voxel, voxel, voxel);
    p_imu->set_extrinsic(V3D::Zero(), M3D::Identity());
    p_imu->set_gyr_cov(V3D::Constant(.1));
    p_imu->set_acc_cov(V3D::Constant(.1));
    p_imu->set_gyr_bias_cov(V3D::Constant(.0001));
    p_imu->set_acc_bias_cov(V3D::Constant(.0001));
    p_imu->lidar_type = OUST64;
    double epsi[23];
    std::fill(epsi, epsi + 23, .001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS,
                      epsi);
    size_t imuIndex = 0;
    uint32_t frame, count;
    double begin, rosHeader;
    bool priorActive = false;
    bool anchored = false;
    while (readValue(scans, frame)) {
      if (!readValue(scans, begin) || !readValue(scans, rosHeader) ||
          !readValue(scans, count))
        throw std::runtime_error("Truncated scan header");
      std::vector<RawPoint> raw(count);
      if (!scans.read(reinterpret_cast<char *>(raw.data()),
                      count * sizeof(RawPoint)))
        throw std::runtime_error("Truncated scan");
      double started = omp_get_wtime();
      auto cloud = PointCloudXYZI::Ptr(new PointCloudXYZI);
      cloud->reserve(count / thinning + 1);
      for (size_t j = 0; j < count; j++) {
        const auto &p = raw[j];
        if (j % thinning || !std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.z) || p.x * p.x + p.y * p.y + p.z * p.z <= 16)
          continue;
        PointType q{};
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
        q.curvature = p.ms;
        cloud->push_back(q);
      }
      if (cloud->empty())
        continue;
      // The organized cloud is not ordered globally by firing time.
      // Keep the full scan epoch even if its final ring has no valid return.
      double end = begin;
      for (const auto &point : raw)
        end = std::max(end, begin + double(point.ms) / 1000.);
      Measures.lidar_beg_time = begin;
      Measures.lidar_end_time = end;
      Measures.lidar = cloud;
      Measures.imu.clear();
      while (imuIndex < imuData.size() && imuData[imuIndex].t <= end) {
        const auto &r = imuData[imuIndex++];
        auto msg = std::make_shared<sensor_msgs::Imu>();
        msg->header.stamp = ros::Time(r.t);
        msg->linear_acceleration.x = r.ax;
        msg->linear_acceleration.y = r.ay;
        msg->linear_acceleration.z = r.az;
        msg->angular_velocity.x = r.gx;
        msg->angular_velocity.y = r.gy;
        msg->angular_velocity.z = r.gz;
        Measures.imu.push_back(msg);
      }
      if (flg_first_scan) {
        first_lidar_time = begin;
        p_imu->first_lidar_time = begin;
        flg_first_scan = false;
        continue;
      }
      if (Measures.imu.empty())
        continue;
      double prepEnd = omp_get_wtime();
      p_imu->Process(Measures, kf, feats_undistort);
      double imuEnd = omp_get_wtime();
      state_point = kf.get_x();
      pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
      if (feats_undistort->empty())
        continue;
      flg_EKF_inited = (begin - first_lidar_time) >= INIT_TIME;
      if (!anchored && frame >= anchor) {
        M3D R = (state_point.rot * state_point.offset_R_L_I).toRotationMatrix();
        Eigen::Quaterniond q(R);
        V3D p = pos_lid;
        std::ofstream anchorOut(std::string(argv[3]) + ".anchor.json");
        anchorOut << std::setprecision(17) << "{\"frame\":" << frame
                  << ",\"nativeEnd\":" << end << ",\"position\":[" << p.x()
                  << "," << p.y() << "," << p.z() << "],\"quaternion\":["
                  << q.x() << "," << q.y() << "," << q.z() << "," << q.w()
                  << "]}\n";
        anchored = true;
      }
      if (fixedMap && !priorActive && frame >= anchor) {
        std::ifstream prior(argv[6], std::ios::binary);
        uint64_t n = 0;
        if (!readValue(prior, n) || n < 5)
          throw std::runtime_error("Invalid prior map");
        PointVector points;
        points.reserve(n);
        M3D R = (state_point.rot * state_point.offset_R_L_I).toRotationMatrix();
        V3D p = pos_lid;
        for (uint64_t j = 0; j < n; j++) {
          float xyz[3];
          if (!prior.read(reinterpret_cast<char *>(xyz), sizeof(xyz)))
            throw std::runtime_error("Truncated prior map");
          V3D v = R * V3D(xyz[0], xyz[1], xyz[2]) + p;
          PointType q{};
          q.x = v.x();
          q.y = v.y();
          q.z = v.z();
          points.push_back(q);
        }
        ikdtree.~KD_TREE();
        new (&ikdtree) KD_TREE<PointType>();
        ikdtree.set_downsample_param(voxel);
        ikdtree.Build(points);
        priorActive = true;
        std::cerr << "PRIOR installed frame " << frame << " count " << n
                  << "\n";
      }
      lasermap_fov_segment();
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      feats_down_size = feats_down_body->size();
      if (ikdtree.Root_Node == nullptr) {
        if (feats_down_size > 5) {
          ikdtree.set_downsample_param(voxel);
          feats_down_world->resize(feats_down_size);
          for (int j = 0; j < feats_down_size; j++)
            pointBodyToWorld(&feats_down_body->points[j],
                             &feats_down_world->points[j]);
          ikdtree.Build(feats_down_world->points);
        }
        continue;
      }
      if (feats_down_size < 5)
        continue;
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);
      pointSearchInd_surf.resize(feats_down_size);
      Nearest_Points.resize(feats_down_size);
      double voxelEnd = omp_get_wtime();
      double hTime = 0;
      kf.update_iterated_dyn_share_modified(LASER_POINT_COV, hTime);
      state_point = kf.get_x();
      pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
      double updateEnd = omp_get_wtime();
      if (!fixedMap || !priorActive)
        map_incremental();
      double seconds = omp_get_wtime() - started;
      auto R = state_point.rot;
      auto L = state_point.rot * state_point.offset_R_L_I;
      if (!state_point.pos.allFinite() || state_point.vel.norm() > 100)
        throw std::runtime_error("Nonfinite state or speed guard");
      out << frame << ',' << rosHeader << ',' << end << ','
          << state_point.pos.x() << ',' << state_point.pos.y() << ','
          << state_point.pos.z() << ',' << R.coeffs()[0] << ',' << R.coeffs()[1]
          << ',' << R.coeffs()[2] << ',' << R.coeffs()[3] << ',' << pos_lid.x()
          << ',' << pos_lid.y() << ',' << pos_lid.z() << ',' << L.coeffs()[0]
          << ',' << L.coeffs()[1] << ',' << L.coeffs()[2] << ','
          << L.coeffs()[3] << ',' << state_point.vel.x() << ','
          << state_point.vel.y() << ',' << state_point.vel.z() << ','
          << feats_down_size << ',' << effct_feat_num << ','
          << ikdtree.validnum() << ',' << seconds << ',' << prepEnd - started
          << ',' << imuEnd - prepEnd << ',' << voxelEnd - imuEnd << ','
          << updateEnd - voxelEnd << ',' << (started + seconds) - updateEnd
          << ',' << priorActive << '\n';
      if (frame % 50 == 0)
        std::cerr << "FRAME " << frame << " matches " << effct_feat_num
                  << " map " << ikdtree.validnum() << " seconds " << seconds
                  << "\n";
    }
    std::cerr << "FASTLIO_COMPLETE\n";
  } catch (const std::exception &e) {
    std::cerr << "FASTLIO_ERROR " << e.what() << "\n";
    return 1;
  }
}
