#include <logger_helper_rerun/logger_3d/log_data.hpp>

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
             const std::string &entity,
             const std::vector<logger_helper_rerun::logger_3d::Arrow> &arrows,
             const std::vector<std::array<uint8_t, 4>> &colors,
             const float radius) {
  if (!rec) {
    return;
  }

  std::vector<rerun::Vec3D> origins;
  std::vector<rerun::Vec3D> vectors;
  for (const auto &arrow : arrows) {
    origins.emplace_back(arrow.origin.x(), arrow.origin.y(), arrow.origin.z());
    vectors.emplace_back(arrow.direction.x(), arrow.direction.y(),
                         arrow.direction.z());
  }

  std::vector<rerun::Rgba32> rgba_colors;
  if (colors.empty()) {
    rgba_colors.resize(arrows.size(), rerun::Rgba32{255, 0, 0, 255});
  } else if (colors.size() == 1) {
    rgba_colors.resize(
        arrows.size(),
        rerun::Rgba32{colors[0][0], colors[0][1], colors[0][2], colors[0][3]});
  } else if (colors.size() != arrows.size()) {
    throw std::invalid_argument(
        "Colors size must be either 0, 1 or equal to the number of arrows");
  } else {
    for (const auto &color : colors) {
      rgba_colors.emplace_back(
          rerun::Rgba32{color[0], color[1], color[2], color[3]});
    }
  }

  rec->log(entity, rerun::Arrows3D::from_vectors(vectors)
                       .with_origins(origins)
                       .with_colors(rgba_colors)
                       .with_radii({radius}));
}

} // namespace rerun