#pragma once
#include <opencv2/core.hpp>
#include <optional>
#include <rerun.hpp>

namespace rerun {
void logData(
    const std::shared_ptr<rerun::RecordingStream> &rec,
    const std::string &entity, const cv::Mat &image,
    const std::string &color_format = "bgr", // "bgr" or "rgb" or "gray"
    const float opacity = 1.0f, int64_t z_index = -20);

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity, const cv::Mat &image,
             const float depth_factor, int64_t z_index = -20);

void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity, std::vector<cv::Rect> rects,
             std::optional<std::vector<std::string>> labels = std::nullopt,
             bool show_label = true,
             std::optional<std::vector<uint16_t>> class_ids = std::nullopt,
             int64_t z_index = 0);
} // namespace rerun