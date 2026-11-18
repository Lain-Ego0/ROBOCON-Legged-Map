#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <glim/mapping/async_global_mapping.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/global_mapping_base.hpp>
#include <glim/mapping/sub_mapping_base.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/raw_points.hpp>

namespace {

template <typename T>
bool read_value(T& value) {
  return static_cast<bool>(std::cin.read(reinterpret_cast<char*>(&value), sizeof(T)));
}

bool read_record_type(uint8_t& type) {
  std::cin.read(reinterpret_cast<char*>(&type), sizeof(type));
  return static_cast<bool>(std::cin) || std::cin.gcount() == sizeof(type);
}

void emit_invalid(double stamp) {
  std::cout << "{\"stamp\":" << stamp << ",\"valid\":false}\n" << std::flush;
}

void emit_pose(const glim::EstimationFrame::ConstPtr& frame) {
  if (!frame) {
    return;
  }
  const Eigen::Isometry3d& pose = frame->T_world_lidar;
  const Eigen::Quaterniond q(pose.linear());
  std::cout.precision(17);
  std::cout << "{\"stamp\":" << frame->stamp << ",\"valid\":true,\"pose\":["
            << pose.translation().x() << ',' << pose.translation().y() << ',' << pose.translation().z() << ','
            << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << "]}\n"
            << std::flush;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: mujoco_glim_bridge CONFIG_DIR MODULE_DIR\n";
    return 2;
  }

  try {
    auto logger = spdlog::stderr_color_mt("robocon-glim");
    logger->set_level(spdlog::level::warn);
    glim::set_default_logger(logger);
    glim::GlobalConfig::instance(argv[1], true);

    const std::string module_dir = argv[2];
    auto odometry = glim::OdometryEstimationBase::load_module(module_dir + "/libodometry_estimation_gpu.so");
    auto sub_mapping = glim::SubMappingBase::load_module(module_dir + "/libsub_mapping.so");
    auto global_mapping = glim::GlobalMappingBase::load_module(module_dir + "/libglobal_mapping_pose_graph.so");
    if (!odometry || !sub_mapping || !global_mapping) {
      throw std::runtime_error("failed to load one or more GLIM modules");
    }

    glim::CloudPreprocessor preprocessor;
    glim::AsyncSubMapping async_sub_mapping(sub_mapping);
    glim::AsyncGlobalMapping async_global_mapping(global_mapping, 2);
    uint8_t type = 0;
    int inserted_submaps = 0;

    while (read_record_type(type)) {
      if (type == static_cast<uint8_t>('I')) {
        double stamp = 0.0;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        if (!read_value(stamp) || !read_value(accel.x()) || !read_value(accel.y()) || !read_value(accel.z()) ||
            !read_value(gyro.x()) || !read_value(gyro.y()) || !read_value(gyro.z())) {
          throw std::runtime_error("truncated IMU record");
        }
        odometry->insert_imu(stamp, accel, gyro);
        async_sub_mapping.insert_imu(stamp, accel, gyro);
        async_global_mapping.insert_imu(stamp, accel, gyro);
        continue;
      }

      if (type != static_cast<uint8_t>('L')) {
        throw std::runtime_error("unknown sensor record type");
      }

      double stamp = 0.0;
      uint32_t count = 0;
      if (!read_value(stamp) || !read_value(count) || count > 100000) {
        throw std::runtime_error("invalid LiDAR record header");
      }
      auto raw = std::make_shared<glim::RawPoints>();
      raw->stamp = stamp;
      raw->points.reserve(count);
      raw->times.reserve(count);
      for (uint32_t i = 0; i < count; ++i) {
        double x = 0.0, y = 0.0, z = 0.0, relative_time = 0.0;
        if (!read_value(x) || !read_value(y) || !read_value(z) || !read_value(relative_time)) {
          throw std::runtime_error("truncated LiDAR point record");
        }
        raw->points.emplace_back(x, y, z, 1.0);
        raw->times.push_back(relative_time);
      }

      auto frame = preprocessor.preprocess(raw);
      std::vector<glim::EstimationFrame::ConstPtr> marginalized;
      auto result = odometry->insert_frame(frame, marginalized);
      for (const auto& old_frame : marginalized) {
        async_sub_mapping.insert_frame(old_frame);
      }
      if (result) {
        emit_pose(result);
      } else {
        emit_invalid(stamp);
      }

      for (const auto& submap : async_sub_mapping.get_results()) {
        async_global_mapping.insert_submap(submap);
        ++inserted_submaps;
        if (inserted_submaps % 4 == 0) {
          glim::GlobalMappingCallbacks::request_to_find_overlapping_submaps(0.12);
          glim::GlobalMappingCallbacks::request_to_optimize();
        }
      }
    }

    for (const auto& remaining : odometry->get_remaining_frames()) {
      async_sub_mapping.insert_frame(remaining);
    }
    async_sub_mapping.join();
    for (const auto& submap : sub_mapping->submit_end_of_sequence()) {
      async_global_mapping.insert_submap(submap);
    }
    for (const auto& submap : async_sub_mapping.get_results()) {
      async_global_mapping.insert_submap(submap);
    }
    async_global_mapping.join();
  } catch (const std::exception& error) {
    std::cerr << "GLIM bridge fatal: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
