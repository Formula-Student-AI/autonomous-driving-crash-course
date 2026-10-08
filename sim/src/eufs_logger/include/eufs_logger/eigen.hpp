#ifndef INCLUDE_EUFS_LOGGER_EIGEN_HPP_
#define INCLUDE_EUFS_LOGGER_EIGEN_HPP_

#include <Eigen/Dense>
#include <string>

namespace eufs::logging::eigen {

std::string to_str(const Eigen::MatrixXd &vec);

}  // namespace eufs::logging::eigen

#endif  // INCLUDE_EUFS_LOGGER_EIGEN_HPP_
