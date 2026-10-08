#ifndef INCLUDE_MAP_LIB_ASSOCIATION_METRIC_BHATTACHARYYA_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_METRIC_BHATTACHARYYA_HPP_

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>
#include <string>

#include "map_lib/association/matrix/safe_inverse.hpp"

namespace eufs::map::association {

struct Bhattacharyya {
  /**
   * @brief Calculate Bhattacharyya distance between two cones
   *
   * @param cone_1 First cone
   * @param cone_2 Second cone
   * @return double Bhattacharyya distance between given cones
   */
  template <typename ConeType1, typename ConeType2>
  double operator()(const ConeType1 &cone_1, const ConeType2 &cone_2) const {
    const Eigen::Vector2d diff = cone_1.position - cone_2.position;
    const Eigen::Matrix2d cov = (cone_1.covariance + cone_2.covariance) / 2;
    const Eigen::Matrix2d cov_inv = SafeInverse(cov);

    // See: https://en.wikipedia.org/wiki/Bhattacharyya_distance#Properties
    const double average_covariance = diff.transpose() * cov_inv * diff;
    const double term = cov.determinant() / std::sqrt(cone_1.covariance.determinant() *
                                                      cone_2.covariance.determinant());
    return (average_covariance / 8) + 0.5 * std::log(term);
  }

  /**
   * @brief Name of association
   * @return name
   */
  static std::string ToString() { return "bhattacharyya"; }
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_METRIC_BHATTACHARYYA_HPP_
