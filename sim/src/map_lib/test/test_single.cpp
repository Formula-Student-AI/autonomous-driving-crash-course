#include <gmock/gmock.h>

#include <map_lib/type/cone.hpp>
#include <map_lib/type/map.hpp>

#include "map_lib/association/single.hpp"
#include "map_lib/association/filter/thresholding.hpp"
#include "map_lib/association/metric/euclidean.hpp"

#include "make_types.hpp"  // NOLINT

using eufs::map::association::Euclidean;
using eufs::map::association::SingleAssociation;
using eufs::map::association::Threshold;
using eufs::map::Map;
using eufs::map::Cone;

///////////////////
//  Check Build  //
///////////////////

TEST(CheckBuild, Build) { SingleAssociation<Threshold<Euclidean>, Euclidean>(1.0); }

/////////////////
// Check Match //
/////////////////

TEST(CheckMatch, SingleObservationTwoValidOptions) {
  std::vector<Cone> observations{MakeCone({1.0, 1.2}, {0.1, 0.0, 0.0, 0.1})};
  Map<Cone> map{std::vector<Cone>{MakeCone({0.5, 1.3}, {0.4, 0.0, 0.0, 0.4}),
                                  MakeCone({1.0, 1.1}, {0.4, 0.0, 0.0, 0.4})}};
  auto association = SingleAssociation<Threshold<Euclidean>, Euclidean>(1.0);
  auto res = association.Match(observations, map);
  EXPECT_THAT(res.matched, ::testing::UnorderedElementsAreArray({::testing::Pair(0lu, 1lu)}));
  EXPECT_THAT(res.unmatched.size(), 0lu);
}

TEST(CheckMatch, SingleObservationOneValidOption) {
  std::vector<Cone> observations{MakeCone({1.0, 1.2}, {0.1, 0.0, 0.0, 0.1})};
  Map<Cone> map{std::vector<Cone>{MakeCone({0.5, 1.3}, {0.4, 0.0, 0.0, 0.4}),
                                  MakeCone({1.0, 1.1}, {0.4, 0.0, 0.0, 0.4})}};
  auto association = SingleAssociation<Threshold<Euclidean>, Euclidean>(0.25);
  auto res = association.Match(observations, map);
  EXPECT_THAT(res.matched, ::testing::UnorderedElementsAreArray({::testing::Pair(0lu, 1lu)}));
  EXPECT_THAT(res.unmatched.size(), 0lu);
}

TEST(CheckMatch, SingleObservationNoValidOption) {
  std::vector<Cone> observations{MakeCone({1.5, 2.0}, {0.1, 0.0, 0.0, 0.1})};
  Map<Cone> map{std::vector<Cone>{MakeCone({0.5, 1.3}, {0.4, 0.0, 0.0, 0.4}),
                                  MakeCone({1.0, 1.1}, {0.4, 0.0, 0.0, 0.4})}};
  auto association = SingleAssociation<Threshold<Euclidean>, Euclidean>(0.25);
  auto res = association.Match(observations, map);
  EXPECT_THAT(res.matched.size(), 0lu);
  EXPECT_THAT(res.unmatched, ::testing::UnorderedElementsAreArray({0lu}));
}

TEST(CheckMatch, UnmatchedObservation) {
  std::vector<Cone> observations{
      MakeCone({1.0, 1.2}, {0.1, 0.0, 0.0, 0.1}),
      MakeCone({5.9, 1.2}, {0.1, 0.0, 0.0, 0.1}),
  };
  Map<Cone> map{std::vector<Cone>{
      MakeCone({0.5, 1.3}, {0.4, 0.0, 0.0, 0.4}),
      MakeCone({1.0, 1.1}, {0.4, 0.0, 0.0, 0.4}),
      MakeCone({5.0, 1.1}, {0.4, 0.0, 0.0, 0.4}),
  }};
  auto association = SingleAssociation<Threshold<Euclidean>, Euclidean>(0.25);
  auto res = association.Match(observations, map);
  EXPECT_THAT(res.matched, ::testing::UnorderedElementsAreArray({
                               ::testing::Pair(0lu, 1lu),
                           }));
  EXPECT_THAT(res.unmatched, ::testing::UnorderedElementsAreArray({1lu}));
}

TEST(CheckMatch, DuplicateAssociation) {
  std::vector<Cone> observations{
      MakeCone({1.0, 1.2}, {0.1, 0.0, 0.0, 0.1}),
      MakeCone({0.9, 1.2}, {0.1, 0.0, 0.0, 0.1}),
  };
  Map<Cone> map{std::vector<Cone>{
      MakeCone({0.5, 1.3}, {0.4, 0.0, 0.0, 0.4}),
      MakeCone({1.0, 1.1}, {0.4, 0.0, 0.0, 0.4}),
      MakeCone({5.0, 1.1}, {0.4, 0.0, 0.0, 0.4}),
  }};
  auto association = SingleAssociation<Threshold<Euclidean>, Euclidean>(0.25);
  auto res = association.Match(observations, map);
  EXPECT_THAT(res.matched, ::testing::UnorderedElementsAreArray({
                               ::testing::Pair(0lu, 1lu),
                               ::testing::Pair(1lu, 1lu),
                           }));
  EXPECT_THAT(res.unmatched.size(), 0lu);
}
