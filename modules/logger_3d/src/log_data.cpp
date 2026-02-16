#include <logger_helper_rerun/logger_3d/log_data.hpp>

namespace rerun {

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