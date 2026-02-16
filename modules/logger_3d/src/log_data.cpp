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

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
             const std::vector<pcl::Vertices> &polygons) {
  if (!rec) {
    return;
  }

  std::vector<rerun::Position3D> vertex_positions;
  std::vector<rerun::Rgba32> vertex_colors;
  vertex_positions.reserve(points->size());
  vertex_colors.reserve(points->size());
  for (const auto &p : points->points) {
    vertex_positions.emplace_back(p.x, p.y, p.z);
    vertex_colors.emplace_back(p.r, p.g, p.b, 255);
  }
  std::vector<rerun::TriangleIndices> tis;
  tis.reserve(polygons.size());
  std::cout << "Number of polygons: " << polygons.size() << std::endl;
  for (const auto &poly : polygons) {
    if (poly.vertices.size() != 3) {
      std::cerr << "Warning: polygon with vertex size != 3 found: "
                << poly.vertices.size() << std::endl;
      continue;
    }
    tis.emplace_back(static_cast<uint32_t>(poly.vertices[0]),
                     static_cast<uint32_t>(poly.vertices[1]),
                     static_cast<uint32_t>(poly.vertices[2]));
  }

  auto mesh = rerun::Mesh3D()
                  .with_vertex_positions(vertex_positions)
                  .with_vertex_colors(vertex_colors)
                  .with_triangle_indices(tis);

  rec->log(entity, mesh);
}
} // namespace rerun