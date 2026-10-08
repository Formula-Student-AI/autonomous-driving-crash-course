#ifndef INCLUDE_MAP_LIB_ASSOCIATION_METRIC_NEGATIVE_LOG_LIKELIHOOD_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_METRIC_NEGATIVE_LOG_LIKELIHOOD_HPP_

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

#include "map_lib/association/matrix/safe_inverse.hpp"

namespace eufs::map::association {

struct NegativeLogLikelihood {
  /**
   * @brief Calculate negative log likelihood of association between two cones
   *
   * @param cone_1 First cone
   * @param cone_2 Second cone
   * @return double Negative log likelihood of association between two cones
   */
  template <typename ConeType1, typename ConeType2>
  double operator()(const ConeType1 &cone_1, const ConeType2 &cone_2) const {
    const Eigen::Vector2d d = cone_1.position - cone_2.position;
    const Eigen::Matrix2d Q = cone_1.covariance + cone_2.covariance;
    const Eigen::Matrix2d Q_inv = SafeInverse(Q);

    return std::log((2 * M_PI * Q).determinant()) / 2 +
           static_cast<double>((d.transpose() * Q_inv * d)) / 2;
  }

  /**
   * @brief Name of association
   * @return name
   */
  static std::string ToString() { return "negative_log_likelihood"; }
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_METRIC_NEGATIVE_LOG_LIKELIHOOD_HPP_
