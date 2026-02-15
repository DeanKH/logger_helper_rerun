#include <logger_helper_rerun/logger_basic/log_data.hpp>
#include <rerun.hpp>

int main() {
  auto rec = std::make_shared<rerun::RecordingStream>("rerun_example_log_msg");
  rec->spawn().exit_on_failure();

  logData(rec, "log", "This is a debug message", logger_basic::LogLevel::Debug);

  std::vector<double> values = {1.0, 2.0, 3.0, 4.0, 5.0};
  for (size_t i = 0; i < 5; ++i) {
    auto copy_values = values;
    for (auto &v : copy_values) {
      v += i;
    }
    logData(rec, "joint_angles", copy_values);
  }
  return 0;
}