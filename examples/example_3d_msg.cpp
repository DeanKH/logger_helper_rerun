#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <logger_helper_rerun/logger_3d/log_data.hpp>
#include <logger_helper_rerun/logger_basic/log_data.hpp>
#include <rerun.hpp>
#include <thread>

struct BoundingBox3D {
  Eigen::Vector3d center; // 中心点
  Eigen::Vector3d size;   // 各軸方向のサイズ (幅, 高さ, 奥行き)
  Eigen::Quaterniond orientation; // 回転 (クォータニオン)
};

int main() {
  auto rec = std::make_shared<rerun::RecordingStream>("rerun_example_log_msg");
  rec->spawn().exit_on_failure();

  // set current time.
  auto current = std::chrono::system_clock::now();
  rec->set_time_timestamp("time", current);
  logData(rec, "log", "This is a 3D log message",
          logger_basic::LogLevel::Debug);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translate(Eigen::Vector3d(1.0, 0, 0));

  logData(rec, tf, "world", "camera1_frame");
  // logData(rec, "camera1_frame");
  rec->log("camera1_frame/points",
           rerun::Points3D({{0.0f, 0.0f, 0.0f},
                            {0.1f, 0.0f, 0.0f},
                            {0.0f, 0.1f, 0.0f},
                            {0.0f, 0.0f, 0.1f}})
               .with_colors({rerun::Rgba32{255, 255, 255, 255},
                             rerun::Rgba32{255, 0, 0, 255},
                             rerun::Rgba32{0, 255, 0, 255},
                             rerun::Rgba32{0, 0, 255, 255}})
               .with_radii({0.02f}));

  std::vector<BoundingBox3D> bboxes;
  BoundingBox3D bbox;
  bbox.center = Eigen::Vector3d(1.0, 0.0, 0.0);
  bbox.size = Eigen::Vector3d(0.5, 0.5, 0.5);
  bbox.orientation = Eigen::Quaterniond::Identity();
  bboxes.push_back(bbox);

  logData(rec, "camera1_frame/boxes", bboxes);
  return 0;
}