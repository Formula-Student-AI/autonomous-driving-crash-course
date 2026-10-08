#ifndef INCLUDE_MAP_LIB_IO_READER_HPP_
#define INCLUDE_MAP_LIB_IO_READER_HPP_

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>  //NOLINT
#include <fmt/core.h>   //NOLINT
#include <geometry_msgs/msg/pose2_d.hpp>

#include "map_lib/transform.hpp"

using geometry_msgs::msg::Pose2D;
namespace eufs::map::io {

// The header of the EUFSFormat
const std::array<std::string, 7> EUFS_HEADER{
    "tag", "x", "y", "direction", "x_variance", "y_variance", "xy_covariance"};

/**
 * @brief Check if the string matches the EUFSFormat header
 *
 * @param header Comma separate values of CSV column names
 */
bool ValidEUFSFormat(const std::string header);

/**
 * @brief Initialise cone based on row elements of a CSV file in EUFSFormat
 *
 * @param row Vector of elements representing a row of the CSV file in EUFSFormat
 */
template <typename ConeType>
ConeType ConvertToCone(const std::vector<std::string> &row) {
  std::array<double, 2> pos{std::atof(row.at(1).c_str()), std::atof(row.at(2).c_str())};
  std::array<double, 4> cov{
      std::atof(row.at(4).c_str()),
      std::atof(row.at(6).c_str()),
      std::atof(row.at(6).c_str()),
      std::atof(row.at(5).c_str()),
  };

  return ConeType({pos.at(0), pos.at(1)}, cov, ConeType::ToColor(row.at(0)));
}

/**
 * @brief Check if string matches one of "blue", "yellow", "orange", "big_orange", "unknown"
 *
 * @return True if there is a match, false otherwise
 */
inline bool IsValidConeColor(const std::string &color) {
  return color == "blue" || color == "yellow" || color == "orange" || color == "big_orange" ||
         color == "unknown";
}

/**
 * @brief Load a map and construct a vector of ConeType
 *
 * @param map_file_path File path to the CSV storing the map
 * @return Vector of ConeType
 */
template <typename ConeType>
std::vector<ConeType> LoadCones(const std::string map_file_path) {
  std::vector<ConeType> result;
  std::fstream file(map_file_path, std::ios::in);  // Read file
  bool car_pos_set = false;
  Pose2D car_pos;

  if (!file.is_open()) {
    throw std::ifstream::failure(
      fmt::format("Map failed to open. Is the path correct? {}", map_file_path));
  }

  // Check valid CSV format
  std::string line, word;
  std::getline(file, line);  // Get CSV header
  if (!ValidEUFSFormat(line)) {
    throw std::runtime_error("Invalid map format!");
  }

  // Loop through all lines in the file
  while (std::getline(file, line)) {
    std::stringstream line_ss(line);
    // Collect row as a vector of strings
    std::vector<std::string> row;
    for (size_t i = 0; i < EUFS_HEADER.size(); ++i) {
      if (std::getline(line_ss, word, ',')) {
        row.push_back(word);
      } else {
        return {};
      }
    }
    // Convert to ConeType
    if (IsValidConeColor(row.at(0))) {
      result.push_back(ConvertToCone<ConeType>(row));
    }
    // Load car start position
    if (row.at(0) == "car_start") {
      car_pos.x = std::stod(row[1]);
      car_pos.y = std::stod(row[2]);
      car_pos.theta = std::stod(row[3]);
      car_pos_set = true;
    }
  }
  // Transform cone positions relative to start position
  if (car_pos_set) {
    Eigen::Vector2d car_position(car_pos.x, car_pos.y);
    eufs::map::Transformation::Translate(result, -car_position);
    eufs::map::Transformation::Rotate(result, -car_pos.theta);
  }
  return result;
}

}  // namespace eufs::map::io
#endif  // INCLUDE_MAP_LIB_IO_READER_HPP_
