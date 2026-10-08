#include <gmock/gmock.h>

#include "map_lib/association/metric/euclidean.hpp"

#include "make_types.hpp"  // NOLINT

using eufs::map::association::Euclidean;

//////////////////////
//  Check operator  //
//////////////////////

TEST(CheckEuclidean, Zero) {
  auto cone_1 = MakeCone({1.2, 1.6});
  auto cone_2 = MakeCone({1.2, 1.6});
  EXPECT_DOUBLE_EQ(Euclidean{}(cone_1, cone_2), 0.0);  // NOLINT
}

TEST(CheckEuclidean, ZeroCovariance) {
  auto cone_1 = MakeCone({1.4, 1.3});
  auto cone_2 = MakeCone({1.0, 0.5});
  EXPECT_DOUBLE_EQ(Euclidean{}(cone_1, cone_2), std::sqrt(0.8));  // NOLINT
}

TEST(CheckEuclidean, NonZeroCovariance) {
  auto cone_1 = MakeCone({1.4, 1.3}, {0.1, 0.0, 0.0, 0.2});
  auto cone_2 = MakeCone({1.0, 0.5}, {0.4, 0.0, 0.0, 0.3});
  EXPECT_DOUBLE_EQ(Euclidean{}(cone_1, cone_2), std::sqrt(0.8));  // NOLINT
}
