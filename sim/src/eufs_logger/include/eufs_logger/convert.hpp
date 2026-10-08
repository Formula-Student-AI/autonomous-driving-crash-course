#ifndef INCLUDE_EUFS_LOGGER_CONVERT_HPP_
#define INCLUDE_EUFS_LOGGER_CONVERT_HPP_

#include <Eigen/Dense>

#include <sstream>
#include <string>

namespace eufs::logging {

template <typename T>
std::string ToString(const T &s) {
  std::stringstream ss;
  ss << s;
  return ss.str();
}

template <typename T>
std::string EigenString(const T &mat) {
  Eigen::Map<const Eigen::VectorXd> a(mat.data(), mat.size());
  std::stringstream ss;
  ss << a.transpose();
  return ss.str();
}

}  // namespace eufs::logging

#endif  // INCLUDE_EUFS_LOGGER_CONVERT_HPP_
