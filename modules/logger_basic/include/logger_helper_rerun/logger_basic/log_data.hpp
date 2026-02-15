#pragma once

#include <rerun.hpp>
#include <string>

namespace logger_basic {
enum class LogLevel { Debug, Info, Warning, Error, Fatal };
}

namespace rerun {
/**
 * Logs a message to the Rerun recording stream with a specified log level.
 * @param rec The Rerun recording stream to log the message to.
 * @param entity_path The path of the entity to which the log message is
 * associated.
 * @param msg The log message to be recorded.
 * @param level The log level indicating the severity of the message (default is
 * Info).
 */
void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity_path, const std::string &msg,
             logger_basic::LogLevel level = logger_basic::LogLevel::Info);

/**
 * Logs a vector of double values to the Rerun recording stream.
 * @param rec The Rerun recording stream to log the values to.
 * @param entity_path The path of the entity to which the log data is
 * associated.
 * @param values The vector of double values to be recorded.
 */
void logData(const std::shared_ptr<rerun::RecordingStream> &rec,
             const std::string &entity_path, const std::vector<double> &values);

/**
 * Clears the log data associated with the specified entity path in the Rerun
 * recording stream. recording stream.
 * @param rec The Rerun recording stream from which to clear the log data.
 * @param entity_path The path of the entity whose log data should be cleared.
 */
void clearView(const std::shared_ptr<rerun::RecordingStream> &rec,
               const std::string &entity_path);
} // namespace rerun