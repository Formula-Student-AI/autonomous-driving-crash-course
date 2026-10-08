#ifndef INCLUDE_MAP_LIB_TESTING_COLOR_PROBABILITIES_HPP_
#define INCLUDE_MAP_LIB_TESTING_COLOR_PROBABILITIES_HPP_

#include <gmock/gmock.h>

#include "map_lib/type/color_probabilities.hpp"

namespace eufs::testing {

inline auto ColorProbabilitiesEq(const eufs::map::ColorProbabilities &exp) {
  return ::testing::AllOf(
      ::testing::Field("blue", &eufs::map::ColorProbabilities::blue,
                       ::testing::DoubleEq(exp.blue)),
      ::testing::Field("yellow", &eufs::map::ColorProbabilities::yellow,
                       ::testing::DoubleEq(exp.yellow)),
      ::testing::Field("orange", &eufs::map::ColorProbabilities::orange,
                       ::testing::DoubleEq(exp.orange)),
      ::testing::Field("big_orange", &eufs::map::ColorProbabilities::big_orange,
                       ::testing::DoubleEq(exp.big_orange)),
      ::testing::Field("unknown", &eufs::map::ColorProbabilities::unknown,
                       ::testing::DoubleEq(exp.unknown)));
}

}  // namespace eufs::testing

#endif  // INCLUDE_MAP_LIB_TESTING_COLOR_PROBABILITIES_HPP_
