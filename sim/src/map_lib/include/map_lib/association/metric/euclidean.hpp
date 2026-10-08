#ifndef INCLUDE_MAP_LIB_ASSOCIATION_METRIC_EUCLIDEAN_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_METRIC_EUCLIDEAN_HPP_

#include <string>

namespace eufs::map::association {

struct Euclidean {
  /**
   * @brief Calculate Euclidean distance between two cones
   *
   * @param cone_1 First cone
   * @param cone_2 Second cone
   * @return double Euclidean distance between given cones
   */
  template <typename ConeType1, typename ConeType2>
  double operator()(const ConeType1 &cone_1, const ConeType2 &cone_2) const {
    return (cone_1.position - cone_2.position).norm();
  }

  /**
   * @brief Name of association
   * @return name
   */
  static std::string ToString() { return "euclidean"; }
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_METRIC_EUCLIDEAN_HPP_
