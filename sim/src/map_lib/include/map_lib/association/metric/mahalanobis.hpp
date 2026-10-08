#ifndef INCLUDE_MAP_LIB_ASSOCIATION_METRIC_MAHALANOBIS_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_METRIC_MAHALANOBIS_HPP_

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>
#include <string>

#include "map_lib/association/matrix/safe_inverse.hpp"

namespace eufs::map::association {

struct Mahalanobis {
  /**
   * @brief Calculate Mahalanobis distance between two cones
   *
   * @param cone_1 First cone
   * @param cone_2 Second cone
   * @return double Mahalanobis distance between given cones
   */
  template <typename ConeType1, typename ConeType2>
  double operator()(const ConeType1 &cone_1, const ConeType2 &cone_2) const {
    const Eigen::Vector2d diff = cone_1.position - cone_2.position;
    const Eigen::Matrix2d cov = cone_1.covariance + cone_2.covariance;
    const Eigen::Matrix2d cov_inv = SafeInverse(cov);
    return std::sqrt(diff.transpose() * cov_inv * diff);
  }

  /**
   * @brief Name of association
   * @return name
   */
  static std::string ToString() { return "mahalanobis"; }
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_METRIC_MAHALANOBIS_HPP_
