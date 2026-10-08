#include <gmock/gmock.h>

#include "map_lib/association/filter/thresholding.hpp"
#include "map_lib/association/metric/euclidean.hpp"
#include "make_types.hpp"  // NOLINT

using namespace eufs::map::association;  // NOLINT

////////////////////////
//  Check operator()  //
////////////////////////

TEST(CheckOperator, Zero) {
  auto cone_1 = MakeCone({1, 1});
  auto cone_2 = MakeCone({1, 1});
  EXPECT_FALSE(Threshold<Euclidean>{}(cone_1, cone_2, 0));
}

TEST(CheckOperator, NonZero) {
  auto cone_1 = MakeCone({1.2, 1});
  auto cone_2 = MakeCone({1, 1.1});
  EXPECT_TRUE(Threshold<Euclidean>{}(cone_1, cone_2, 0.5));
}
