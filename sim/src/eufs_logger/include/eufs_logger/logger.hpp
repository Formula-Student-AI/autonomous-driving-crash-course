#ifndef INCLUDE_EUFS_LOGGER_LOGGER_HPP_
#define INCLUDE_EUFS_LOGGER_LOGGER_HPP_

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <string>
#include <memory>

namespace eufs::logging {

// Define types
using EUFSLogger = spdlog::logger;
using EUFSLoggerPtr = std::shared_ptr<EUFSLogger>;

/**
 * @brief Instantiate new logger
 *
 * @param log_name Name of the logger
 * @param console_log_level
 * @param file_log_level
 */
EUFSLoggerPtr InitialiseLogger(
    std::string log_name,
    spdlog::level::level_enum console_log_level = spdlog::level::info,
    spdlog::level::level_enum file_log_level = spdlog::level::trace
);

/**
 * @brief Generate a path to a log file. Will expand any environment variables
 * of the form ${ENV_VARIABLE} and will add a ".log" extension at the end.
 *
 * @param dir Path to the file (no extension)
 * @param stamp Whether to add timestamp to file_name
 * @return Returns a expanded path to the file
 */
std::string GenerateLogPath(std::string path, bool stamp);

/**
 * @brief Expand environment variables of the form `${VARIABLE}` in place.
 *
 * Source: https://stackoverflow.com/a/23442780
 *
 * @param path Path to expand
*/
void AutoExpandEnvironmentVariables(std::string &path);  // NOLINT

/**
 * @brief Expand environment variables of the form `${VARIABLE}`.
 *
 * Source: https://stackoverflow.com/a/23442780
 *
 * @param path Path to expand
 * @returns Expanded path
*/
std::string ExpandEnvironmentVariables(const std::string &path);

/**
 * @brief Generate the current time in %Y-%m-%d_%H-%M-%S format
*/
std::string GenerateTimeStamp();

}  // namespace eufs::logging

#endif  // INCLUDE_EUFS_LOGGER_LOGGER_HPP_
