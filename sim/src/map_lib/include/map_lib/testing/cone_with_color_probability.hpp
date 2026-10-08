#ifndef INCLUDE_MAP_LIB_TESTING_CONE_WITH_COLOR_PROBABILITY_HPP_
#define INCLUDE_MAP_LIB_TESTING_CONE_WITH_COLOR_PROBABILITY_HPP_

#include <gmock/gmock.h>

#include <eufs_gmock_matchers/eigen_matchers.hpp>

#include "map_lib/testing/color_probabilities.hpp"
#include "map_lib/type/cone_with_color_probability.hpp"

namespace eufs::testing {

inline auto ConeWithColorProbabilityEq(const eufs::map::ConeWithColorProbability &exp) {
  return ::testing::AllOf(
      ::testing::Field("position", &eufs::map::ConeWithColorProbability::position,
                       eufs::testing::matchers::EigenEq(exp.position)),
      ::testing::Field("covariance", &eufs::map::ConeWithColorProbability::covariance,
                       eufs::testing::matchers::EigenEq(exp.covariance)),
      ::testing::Property("GetColorProbabilities",
                          &eufs::map::ConeWithColorProbability::GetColorProbabilities,
                          ColorProbabilitiesEq(exp.GetColorProbabilities())));
}

}  // namespace eufs::testing

#endif  // INCLUDE_MAP_LIB_TESTING_CONE_WITH_COLOR_PROBABILITY_HPP_
