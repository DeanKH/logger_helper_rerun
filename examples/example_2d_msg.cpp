#include <filesystem>
#include <logger_helper_rerun/logger_2d/log_data.hpp>
#include <logger_helper_rerun/logger_basic/log_data.hpp>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <thread>

int main() {
  auto rec = std::make_shared<rerun::RecordingStream>("rerun_example_log_msg");
  rec->spawn().exit_on_failure();
  std::filesystem::path img_path =
      std::filesystem::path(EXAMPLE_INPUT_DIR) / "carton_crop.png";

  // set current time.
  auto current = std::chrono::system_clock::now();
  rec->set_time_timestamp("time", current);
  //   rec->set_time_sequence("time", 0);
  logData(rec, "log", img_path.string(), logger_basic::LogLevel::Debug);

  cv::Mat img = cv::imread(img_path.string());

  rec->log_static(
      "/image/boxes",
      rerun::AnnotationContext({
          rerun::AnnotationInfo(0, "red", rerun::Rgba32(255, 0, 0)),
          rerun::AnnotationInfo(2, "green", rerun::Rgba32(0, 255, 0)),
      }));

  for (size_t i = 1; i < 6; ++i) {
    auto current = std::chrono::system_clock::now();
    // rec->set_time_sequence("time", i);
    rec->set_time_timestamp("time", current);
    logData(rec, "image/color", img, "bgr", 0.8f, -10);

    std::vector<cv::Rect> rects = {cv::Rect(50 + i * 5, 50, 100, 150),
                                   cv::Rect(200, 80 - i * 5, 120, 160)};
    std::vector<uint16_t> class_ids = {0, 2};
    std::vector<std::string> labels = {"box_" + std::to_string(i),
                                       "box_" + std::to_string(i)};
    logData(rec, "image/boxes", rects, labels, true, class_ids);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}