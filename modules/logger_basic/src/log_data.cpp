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