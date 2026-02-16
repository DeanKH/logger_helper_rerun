#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/Vertices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rerun.hpp>

namespace logger_helper_rerun::logger_3d {
struct Arrow {
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
};
struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a = 255;
};

/**
 * Maps an input h from a value between 0.0 and 1.0 into a rainbow.
 * Copied from VoxBlox.
 */
inline Color rainbowColorMap(double h) {
  Color color;
  color.a = 255;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
  case 6:
  case 0:
    color.r = 255 * v;
    color.g = 255 * n;
    color.b = 255 * m;
    break;
  case 1:
    color.r = 255 * n;
    color.g = 255 * v;
    color.b = 255 * m;
    break;
  case 2:
    color.r = 255 * m;
    color.g = 255 * v;
    color.b = 255 * n;
    break;
  case 3:
    color.r = 255 * m;
    color.g = 255 * n;
    color.b = 255 * v;
    break;
  case 4:
    color.r = 255 * n;
    color.g = 255 * m;
    color.b = 255 * v;
    break;
  case 5:
    color.r = 255 * v;
    color.g = 255 * m;
    color.b = 255 * n;
    break;
  default:
    color.r = 255;
    color.g = 127;
    color.b = 127;
    break;
  }

  return color;
}

template <typename PointT>
std::vector<rerun::Rgba32>
constructColorsFromPointCloud(typename pcl::PointCloud<PointT>::Ptr &cloud) {
  std::vector<rerun::Rgba32> colors;
  colors.reserve(cloud->size());
  for (const auto &p : cloud->points) {
    if constexpr (std::is_same<PointT, pcl::PointXYZRGB>::value) {
      colors.emplace_back(p.r, p.g, p.b, 255);
    } else {
      // If the point type does not have color information,
      // zの距離に応じたグラデーションカラーを生成する
      auto color = rainbowColorMap(p.z / 3.0); // 3.0は適当な距離のスケール.
      colors.emplace_back(color.r, color.g, color.b, color.a);
    }
  }
  return colors;
}
} // namespace logger_helper_rerun::logger_3d

namespace rerun {

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity,
             const std::vector<logger_helper_rerun::logger_3d::Arrow> &arrows,
             const std::vector<std::array<uint8_t, 4>> &colors = {},
             const float radius = 0.01f);

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
             const std::vector<pcl::Vertices> &polygons);

template <class T>
concept BB3dConcept = requires(const T &bb) {
  bb.center;
  bb.size;
  bb.orientation;
};

template <BB3dConcept T>
void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity, const std::vector<T> &bboxes,
             const std::array<uint8_t, 4> color = {0, 255, 0, 255},
             const float line_radius = 0.005f,
             rerun::components::FillMode fill_mode =
                 rerun::components::FillMode::MajorWireframe) {
  if (!rec) {
    return;
  }
  std::vector<rerun::Vec3D> centers;
  std::vector<rerun::Vec3D> sizes;
  std::vector<rerun::Quaternion> orientations;
  std::vector<rerun::Rgba32> colors;
  centers.reserve(bboxes.size());
  sizes.reserve(bboxes.size());
  orientations.reserve(bboxes.size());
  colors.reserve(bboxes.size());
  for (const auto &bbox : bboxes) {
    centers.emplace_back(bbox.center.template cast<float>().x(),
                         bbox.center.template cast<float>().y(),
                         bbox.center.template cast<float>().z());
    sizes.emplace_back(bbox.size.template cast<float>().x(),
                       bbox.size.template cast<float>().y(),
                       bbox.size.template cast<float>().z());
    Eigen::Quaternionf q = bbox.orientation.template cast<float>();
    orientations.emplace_back(
        rerun::Quaternion::from_wxyz(q.w(), q.x(), q.y(), q.z()));
    colors.emplace_back(rerun::Rgba32{color[0], color[1], color[2], color[3]});
  }

  auto boxes = rerun::Boxes3D::from_centers_and_sizes(centers, sizes)
                   .with_quaternions(orientations)
                   .with_colors(colors)
                   .with_radii({line_radius})
                   .with_fill_mode(fill_mode);
  rec->log(entity, boxes);
}

template <typename PointT>
void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity,
             typename pcl::PointCloud<PointT>::Ptr &cloud,
             const float radius = 0.005f) {
  static_assert(std::is_same<PointT, pcl::PointXYZ>::value,
                "Only pcl::PointXYZ is supported");

  if (!rec) {
    return;
  }

  std::vector<rerun::Vec3D> pts;
  std::vector<rerun::Rgba32> colors;
  pts.reserve(cloud->size());
  colors.reserve(cloud->size());
  for (const auto &p : cloud->points) {
    pts.emplace_back(p.x, p.y, p.z);
  }

  colors = constructColorsFromPointCloud(cloud);
  rec->log(entity,
           rerun::Points3D(pts).with_colors(colors).with_radii({radius}));
}

template <typename PointT>
void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity,
             const typename pcl::PointCloud<PointT>::Ptr &points,
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
  }
  vertex_colors = constructColorsFromPointCloud(points);

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