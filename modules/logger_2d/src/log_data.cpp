#include <logger_helper_rerun/logger_2d/log_data.hpp>
#include <opencv2/imgproc.hpp>

namespace rerun {
void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity, const cv::Mat &image,
             const std::string &color_format, const float opacity,
             int64_t z_index) {
  if (!rec) {
    return;
  }

  // TODO: support rgba input.

  assert(image.depth() == CV_8U);
  assert(image.channels() == 3 || image.channels() == 1);

  rerun::Image img;
  if (color_format == "bgr") {
    cv::Mat image_rgb;
    cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB);
    std::vector<uint8_t> data(image_rgb.total() * image_rgb.channels());
    std::memcpy(data.data(), image_rgb.data,
                image_rgb.total() * image_rgb.channels() * sizeof(uint8_t));
    img =
        rerun::Image::from_rgb24(data, {static_cast<uint32_t>(image_rgb.cols),
                                        static_cast<uint32_t>(image_rgb.rows)})
            .with_opacity(opacity)
            .with_draw_order(z_index);
  } else if (color_format == "rgb") {
    std::vector<uint8_t> data(image.total() * image.channels());
    std::memcpy(data.data(), image.data,
                image.total() * image.channels() * sizeof(uint8_t));
    img = rerun::Image::from_rgb24(data, {static_cast<uint32_t>(image.cols),
                                          static_cast<uint32_t>(image.rows)})
              .with_opacity(opacity)
              .with_draw_order(z_index);
  } else if (color_format == "gray") {
    std::vector<uint8_t> data(image.total() * image.channels());
    std::memcpy(data.data(), image.data,
                image.total() * image.channels() * sizeof(uint8_t));
    img =
        rerun::Image::from_grayscale8(data, {static_cast<uint32_t>(image.cols),
                                             static_cast<uint32_t>(image.rows)})
            .with_opacity(opacity)
            .with_draw_order(z_index);
  } else {
    throw std::invalid_argument("Unsupported color format: " + color_format);
  }
  rec->log(entity, img);
}

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity, const cv::Mat &image,
             const float depth_factor, int64_t z_index) {
  if (!rec) {
    return;
  }
  assert(image.depth() == CV_16U);
  assert(image.channels() == 1);

  std::vector<uint16_t> data(image.total(),
                             std::numeric_limits<uint16_t>::max());
  std::memcpy(data.data(), image.data, image.total() * sizeof(uint16_t));

  rec->log(entity,
           rerun::DepthImage(data.data(), {static_cast<uint32_t>(image.cols),
                                           static_cast<uint32_t>(image.rows)})
               .with_meter(depth_factor)
               .with_colormap(rerun::components::Colormap::Turbo)
               .with_draw_order(z_index));
}

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity, std::vector<cv::Rect> rects,
             std::optional<std::vector<std::string>> labels, bool show_label,
             std::optional<std::vector<uint16_t>> class_ids, int64_t z_index) {
  if (!rec) {
    return;
  }

  std::vector<rerun::Vec2D> mins;
  std::vector<rerun::Vec2D> sizes;
  mins.reserve(rects.size());
  sizes.reserve(rects.size());

  for (const auto &rect : rects) {
    mins.emplace_back(static_cast<float>(rect.x), static_cast<float>(rect.y));
    sizes.emplace_back(static_cast<float>(rect.width),
                       static_cast<float>(rect.height));
  }

  auto boxes =
      rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
          .with_draw_order(z_index)
          .with_labels(
              labels.value_or(std::vector<std::string>(rects.size(), "")))
          .with_show_labels(
              rerun::components::ShowLabels(show_label && labels.has_value()))
          .with_class_ids(
              class_ids.value_or(std::vector<uint16_t>(rects.size(), 0)));

  rec->log(entity, boxes);
}

} // namespace rerun