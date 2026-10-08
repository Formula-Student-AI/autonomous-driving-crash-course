#ifndef INCLUDE_MAP_LIB_IO_WRITER_HPP_
#define INCLUDE_MAP_LIB_IO_WRITER_HPP_

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

#include "map_lib/io/reader.hpp"
#include "map_lib/type/cone.hpp"

namespace eufs::map::io {

/**
 * @brief Convert an Enum representing the color of a cone to String
 *
 * @param color Enum to convert to string
 * @return String representing the color
*/
template <typename ConeType> std::string GetConeColorStr(const Color &color) {
    switch (color) {
        case Color::BLUE:
            return "blue";
            break;
        case Color::YELLOW:
            return "yellow";
            break;
        case Color::ORANGE:
            return "orange";
            break;
        default:
            return "unknown";
    }
}

/**
 * @brief Create a row of the CSV file based on a cone object
 *
 * @param cone Cone object to convert to string
 * @return String representing a cone in EUFSFormat as a row of the CSV file
*/
template <typename ConeType> std::string ConvertToRow(const ConeType &cone) {
    // Get the color of the cone (convert from Enum to string)
    std::string color = GetConeColorStr<ConeType>(cone.GetColor());

    // Create string to represent cone
    std::stringstream ss;
    ss << color << "," << cone.position(0) << "," << cone.position(1) << ",0,"
       << cone.covariance(0, 0) << "," << cone.covariance(1, 1) << ","
       << cone.covariance(0, 1) << "\n";

    return ss.str();
}

/**
 * @brief Save a vector of ConeType to a csv file
 *
 * @param cones Vector of elements representing cone objects in the map
 * @param save_file_path File path to the CSV to save the map to
*/
template <typename ConeType> void SaveCones(const std::vector<ConeType> &cones,
                                            const std::string save_file_path) {
    std::fstream file(save_file_path, std::ios::out);  // Write file

    if (file.is_open()) {
        // Create and write EUFS Header line to file
        for (size_t i = 0; i < EUFS_HEADER.size(); ++i) {
            file << EUFS_HEADER[i] << ((i == EUFS_HEADER.size() - 1) ? "\n" : ",");
        }

        // Convert ConeType to row strings, and write to file
        for (const auto &cone : cones) {
            file << ConvertToRow(cone);  // Convert to row string and write to file
        }
    } else {
        throw std::invalid_argument("Unable to open CSV file.");
    }

    file.close();
}

}  // namespace eufs::map::io

#endif  // INCLUDE_MAP_LIB_IO_WRITER_HPP_
