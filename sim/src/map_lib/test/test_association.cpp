#include <gmock/gmock.h>

#include <map_lib/type/cone.hpp>
#include <map_lib/type/map.hpp>

#include "map_lib/association/type/association.hpp"
#include "map_lib/fov.hpp"

#include "make_types.hpp"  // NOLINT

using eufs::map::association::Association;
using eufs::map::association::FillRange;
using eufs::map::SectorParameters;
using eufs::map::WithinSectorChecker;
using eufs::map::association::MissedLandmarks;
using eufs::map::Map;
using eufs::map::Cone;

/////////////////////
// Check FillRange //
/////////////////////

TEST(CheckFillRange, ZeroRange) {
  Association association;
  FillRange(association.unmatched, 0, 0);
  EXPECT_EQ(association.unmatched.size(), 0lu);
}

TEST(CheckFillRange, NonZeroRange) {
  Association association;
  FillRange(association.unmatched, 0, 3);
  EXPECT_EQ(association.unmatched.size(), 3lu);
  EXPECT_THAT(association.unmatched, ::testing::UnorderedElementsAreArray({0, 1, 2}));
}

TEST(CheckFillRange, ClearSet) {
  Association association;
  association.unmatched.emplace(10);
  FillRange(association.unmatched, 0, 3);
  EXPECT_EQ(association.unmatched.size(), 3lu);
  EXPECT_THAT(association.unmatched, ::testing::UnorderedElementsAreArray({0, 1, 2}));
}

TEST(CheckFillRange, StepSize) {
  Association association;
  association.unmatched.emplace(10);
  FillRange(association.unmatched, 0, 3, 2);
  EXPECT_EQ(association.unmatched.size(), 2lu);
  EXPECT_THAT(association.unmatched, ::testing::UnorderedElementsAreArray({0, 2}));
}

///////////////////////////
// Check MissedLandmarks //
///////////////////////////

TEST(CheckMissedLandmarks, Sanity) {
  // Map
  Map<Cone> map{std::vector<Cone>{MakeCone({2.0, 0.1}, {0.4, 0.0, 0.0, 0.4}),
                                  MakeCone({1.0, 1.1}, {0.4, 0.0, 0.0, 0.4}),
                                  MakeCone({2.0, -0.1}, {0.4, 0.0, 0.0, 0.4})}};

  // Association
  Association association;
  association.unmatched.emplace(0);
  association.unmatched.emplace(1);
  association.matched.emplace(1, 2);

  // Predicate
  auto pred = WithinSectorChecker(SectorParameters{5.0, 60 * M_PI / 180});

  // check result
  auto res = MissedLandmarks(map, association, pred);
  EXPECT_EQ(res.size(), 1lu);
  EXPECT_THAT(res, ::testing::UnorderedElementsAreArray({0}));
}
