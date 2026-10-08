#include <gmock/gmock.h>

#include "map_lib/association/metric/negative_log_likelihood.hpp"

#include "make_types.hpp"  // NOLINT

using eufs::map::association::NegativeLogLikelihood;

//////////////////////
//  Check operator  //
//////////////////////

TEST(CheckNegativeLogLikelihood, ZeroCovariance) {
  auto cone_1 = MakeCone({1.4, 1.3});
  auto cone_2 = MakeCone({1.0, 0.5});
  EXPECT_ANY_THROW(NegativeLogLikelihood{}(cone_1, cone_2));  // NOLINT
}

TEST(CheckNegativeLogLikelihood, ZeroDifference) {
  auto cone_1 = MakeCone({1.2, 1.6}, {0.1, 0.0, 0.0, 0.1});
  auto cone_2 = MakeCone({1.2, 1.6}, {0.4, 0.0, 0.0, 0.3});
  auto exp = std::log(2 * M_PI) + 0.5 * std::log(0.2);
  EXPECT_DOUBLE_EQ(NegativeLogLikelihood{}(cone_1, cone_2), exp);  // NOLINT
}

TEST(CheckNegativeLogLikelihood, NonZeroDifference) {
  auto cone_1 = MakeCone({1.4, 1.3}, {0.1, 0.0, 0.0, 0.1});
  auto cone_2 = MakeCone({1.0, 0.5}, {0.4, 0.0, 0.0, 0.3});
  auto exp = std::log(2 * M_PI) + 0.5 * std::log(0.2) + 0.5 * 1.92;
  EXPECT_DOUBLE_EQ(NegativeLogLikelihood{}(cone_1, cone_2), exp);  // NOLINT
}
