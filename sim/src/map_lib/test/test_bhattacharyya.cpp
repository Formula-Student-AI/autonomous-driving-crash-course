#include <gmock/gmock.h>

#include "map_lib/association/metric/bhattacharyya.hpp"

#include "make_types.hpp"  // NOLINT

using eufs::map::association::Bhattacharyya;

//////////////////////
//  Check operator  //
//////////////////////

TEST(CheckBhattacharyya, ZeroCovariance) {
  auto cone_1 = MakeCone({1.4, 1.3});
  auto cone_2 = MakeCone({1.0, 0.5});
  EXPECT_ANY_THROW(Bhattacharyya{}(cone_1, cone_2));  // NOLINT
}

TEST(CheckBhattacharyya, ZeroDifference) {
  auto cone_1 = MakeCone({1.2, 1.6}, {0.1, 0.0, 0.0, 0.1});
  auto cone_2 = MakeCone({1.2, 1.6}, {0.4, 0.0, 0.0, 0.3});
  auto exp = 0.5 * std::log(0.05 / std::sqrt(0.01 * 0.12));
  EXPECT_DOUBLE_EQ(Bhattacharyya{}(cone_1, cone_2), exp);  // NOLINT
}

TEST(CheckBhattacharyya, NonZeroDifference) {
  auto cone_1 = MakeCone({1.4, 1.3}, {0.1, 0.0, 0.0, 0.1});
  auto cone_2 = MakeCone({1.0, 0.5}, {0.4, 0.0, 0.0, 0.3});
  auto exp = 3.84 / 8 + 0.5 * std::log(0.05 / std::sqrt(0.01 * 0.12));
  EXPECT_DOUBLE_EQ(Bhattacharyya{}(cone_1, cone_2), exp);  // NOLINT
}
