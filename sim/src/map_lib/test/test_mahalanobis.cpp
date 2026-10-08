#include <gmock/gmock.h>

#include "map_lib/association/metric/mahalanobis.hpp"

#include "make_types.hpp"  // NOLINT

using eufs::map::association::Mahalanobis;

//////////////////////
//  Check operator  //
//////////////////////

TEST(CheckMahalanobis, ZeroCovariance) {
  auto cone_1 = MakeCone({1.4, 1.3});
  auto cone_2 = MakeCone({1.0, 0.5});
  EXPECT_ANY_THROW(Mahalanobis{}(cone_1, cone_2));  // NOLINT
}

TEST(CheckMahalanobis, ZeroDifference) {
  auto cone_1 = MakeCone({1.2, 1.6}, {0.1, 0.0, 0.0, 0.1});
  auto cone_2 = MakeCone({1.2, 1.6}, {0.4, 0.0, 0.0, 0.3});
  EXPECT_DOUBLE_EQ(Mahalanobis{}(cone_1, cone_2), 0.0);  // NOLINT
}

TEST(CheckMahalanobis, NonZeroDifference) {
  auto cone_1 = MakeCone({1.4, 1.3}, {0.1, 0.0, 0.0, 0.1});
  auto cone_2 = MakeCone({1.0, 0.5}, {0.4, 0.0, 0.0, 0.3});
  EXPECT_DOUBLE_EQ(Mahalanobis{}(cone_1, cone_2), std::sqrt(1.92));  // NOLINT
}
