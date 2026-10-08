#include "eufs_logger/logger.hpp"

#include <chrono>  // NOLINT
#include <fstream>
#include <iomanip>
#include <regex>  // NOLINT
#include <sstream>
#include <string>
#include <filesystem>
namespace eufs::logging {

std::shared_ptr<spdlog::logger> InitialiseLogger(std::string log_name,
                                                 spdlog::level::level_enum console_log_level,
                                                 spdlog::level::level_enum file_log_level) {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(console_log_level);

  std::string log_path =
  GenerateLogPath("${EUFS_MASTER}/eufs_log/" + log_name +"/" +log_name, true);

  std::filesystem::path parentPath = std::filesystem::path(log_path).parent_path();

  // Create directories leading to the file if they don't exist
  if (!std::filesystem::exists(parentPath)) {
    std::filesystem::create_directories(parentPath);
  }

    // make file if it doesn't exist
    std::ifstream infile(log_path.c_str());
    if (!infile.good()) {
      // If the file does not exist, create it
      std::ofstream outfile(log_path.c_str());
      outfile.close();
      }

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path, true);
    file_sink->set_level(file_log_level);

    spdlog::sinks_init_list sink_list = {file_sink, console_sink};

    auto logger =
        std::make_shared<spdlog::logger>("eufs_logger", sink_list.begin(), sink_list.end());
    logger->set_level(spdlog::level::trace);
    logger->flush_on(spdlog::level::trace);
    return logger;
  }

std::string GenerateLogPath(std::string path, bool stamp) {
  auto expanded_path = ExpandEnvironmentVariables(path);
  if (stamp) {
    expanded_path += "_" + GenerateTimeStamp();
  }
  return expanded_path + ".log";
}

void AutoExpandEnvironmentVariables(std::string &path) {  // NOLINT
  static std::regex env("\\$\\{([^}]+)\\}");
  std::smatch match;
  while (std::regex_search(path, match, env)) {
    const char *s = getenv(match[1].str().c_str());
    const std::string var(s == NULL ? "" : s);
    path.replace(match[0].first, match[0].second, var);
  }
}

std::string ExpandEnvironmentVariables(const std::string &path) {
  std::string text = path;
  AutoExpandEnvironmentVariables(text);
  return text;
}

std::string GenerateTimeStamp() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream datetime;
  datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
  return datetime.str();
}

}  // namespace eufs::logging
