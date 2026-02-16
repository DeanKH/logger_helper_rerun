#include <logger_helper_rerun/logger_basic/log_data.hpp>
#include <rerun/components/text_log_level.hpp>
namespace {
rerun::components::TextLogLevel convertLogLevel(logger_basic::LogLevel level) {
  switch (level) {
  case logger_basic::LogLevel::Debug:
    return rerun::components::TextLogLevel::Debug;
  case logger_basic::LogLevel::Info:
    return rerun::components::TextLogLevel::Info;
  case logger_basic::LogLevel::Warning:
    return rerun::components::TextLogLevel::Warning;
  case logger_basic::LogLevel::Error:
    return rerun::components::TextLogLevel::Error;
  case logger_basic::LogLevel::Fatal:
    return rerun::components::TextLogLevel::Critical;
  default:
    return rerun::components::TextLogLevel::Info; // Default to Info if unknown
  }
}
} // namespace

namespace rerun {
void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const Eigen::Isometry3d &transform,
             const std::string &parent_frame, const std::string &child_frame) {
  if (!rec) {
    return;
  }
  if (parent_frame.empty() || child_frame.empty()) {
    throw std::invalid_argument("Parent frame and child frame cannot be empty");
  }

  const std::string tf_prefix = "tf#";
  // parent_frame
  std::string parent_frame_full = parent_frame.front() != '/'
                                      ? tf_prefix + "/" + parent_frame
                                      : tf_prefix + parent_frame;
  // child_frame
  std::string child_frame_full = child_frame.front() != '/'
                                     ? tf_prefix + "/" + child_frame
                                     : tf_prefix + child_frame;
  auto translation = rerun::Vec3D{transform.translation().cast<float>().x(),
                                  transform.translation().cast<float>().y(),
                                  transform.translation().cast<float>().z()};

  Eigen::Quaterniond q;
  q = transform.rotation();
  auto orientation = rerun::Quaternion::from_wxyz(q.w(), q.x(), q.y(), q.z());
  auto rotation = rerun::Rotation3D(orientation);
  auto tf = rerun::Transform3D::from_translation_rotation(translation, rotation)
                .with_parent_frame(parent_frame_full)
                .with_child_frame(child_frame_full);
  rec->log(parent_frame, rerun::CoordinateFrame(parent_frame_full), tf);
}

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity, const Eigen::Isometry3d &transform,
             const float axis_length) {
  if (!rec) {
    return;
  }
  auto translation = rerun::Vec3D{transform.translation().cast<float>().x(),
                                  transform.translation().cast<float>().y(),
                                  transform.translation().cast<float>().z()};
  Eigen::Quaterniond q;
  q = transform.rotation();
  auto orientation = rerun::Quaternion::from_wxyz(q.w(), q.x(), q.y(), q.z());
  auto rotation = rerun::Rotation3D(orientation);
  auto tf =
      rerun::Transform3D::from_translation_rotation(translation, rotation);

  rec->log(entity, tf, rerun::TransformAxes3D(axis_length));
}

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity_path, const std::string &msg,
             logger_basic::LogLevel level) {
  if (!rec) {
    return;
  }

  rec->log(entity_path, rerun::TextLog(msg).with_level(convertLogLevel(level)));
}

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity_path,
             const std::vector<double> &values) {
  if (!rec) {
    return;
  }

  rec->log(entity_path, rerun::Scalars(values));
}
void clearView(const std::shared_ptr<rerun::RecordingStream> &rec,
               const std::string &entity_path) {
  if (!rec) {
    return;
  }

  rec->log(entity_path, rerun::Clear::FLAT);
}
} // namespace rerun