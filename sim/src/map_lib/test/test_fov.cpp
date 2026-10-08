#include <gmock/gmock.h>

#include "map_lib/fov.hpp"
#include "make_types.hpp"  // NOLINT

using namespace eufs::map;  // NOLINT

////////////////////////////////////////////
//  Check WithinSectorChecker.operator()  //
////////////////////////////////////////////

TEST(CheckWithinSectorOperator, WithinLeft) {
auto cone = MakeCone({2, 0.5});
EXPECT_TRUE(WithinSectorChecker(SectorParameters{5.0, 60 * M_PI / 180})(cone));
}

TEST(CheckWithinSectorOperator, WithinRight) {
auto cone = MakeCone({2, -0.5});
EXPECT_TRUE(WithinSectorChecker(SectorParameters{5.0, 60 * M_PI / 180})(cone));
}

TEST(CheckWithinSectorOperator, OutsideLeftForward) {
auto cone = MakeCone({1, 1});
EXPECT_FALSE(WithinSectorChecker(SectorParameters{5.0, 60 * M_PI / 180})(cone));
}

TEST(CheckWithinSectorOperator, OutsideRightForward) {
auto cone = MakeCone({1, -1});
EXPECT_FALSE(WithinSectorChecker(SectorParameters{5.0, 60 * M_PI / 180})(cone));
}

TEST(CheckWithinSectorOperator, OutsideLeftBackward) {
auto cone = MakeCone({-2, 1});
EXPECT_FALSE(WithinSectorChecker(SectorParameters{5.0, 60 * M_PI / 180})(cone));
}

TEST(CheckWithinSectorOperator, OutsideRightBackward) {
auto cone = MakeCone({-2, -1});
EXPECT_FALSE(WithinSectorChecker(SectorParameters{5.0, 60 * M_PI / 180})(cone));
}

/////////////////////////////////////////////
//  Check WithinAnnulusChecker.operator()  //
/////////////////////////////////////////////

TEST(CheckWithinAnnulusOperator, WithinLeft) {
auto cone = MakeCone({-2, -0.5});
EXPECT_TRUE(WithinAnnulusChecker(AnnulusParameters{5.0, 60 * M_PI / 180, 1.0, M_PI})(cone));
}

TEST(CheckWithinAnnulusOperator, WithinRight) {
auto cone = MakeCone({-2, 0.5});
EXPECT_TRUE(WithinAnnulusChecker(AnnulusParameters{5.0, 60 * M_PI / 180, 1.0, M_PI})(cone));
}

TEST(CheckWithinAnnulusOperator, OutsideLeftForward) {
auto cone = MakeCone({-1, -1});
EXPECT_FALSE(WithinAnnulusChecker(AnnulusParameters{5.0, 60 * M_PI / 180, 1.0, M_PI})(cone));
}

TEST(CheckWithinAnnulusOperator, OutsideRightForward) {
auto cone = MakeCone({-1, 1});
EXPECT_FALSE(WithinAnnulusChecker(AnnulusParameters{5.0, 60 * M_PI / 180, 1.0, M_PI})(cone));
}

TEST(CheckWithinAnnulusOperator, OutsideLeftBackward) {
auto cone = MakeCone({2, -1});
EXPECT_FALSE(WithinAnnulusChecker(AnnulusParameters{5.0, 60 * M_PI / 180, 1.0, M_PI})(cone));
}

TEST(CheckWithinAnnulusOperator, OutsideRightBackward) {
auto cone = MakeCone({2, 1});
EXPECT_FALSE(WithinAnnulusChecker(AnnulusParameters{5.0, 60 * M_PI / 180, 1.0, M_PI})(cone));
}

TEST(CheckWithinAnnulusOperator, InOccludedRadius) {
auto cone = MakeCone({-0.4, 0.1});
EXPECT_FALSE(WithinAnnulusChecker(AnnulusParameters{5.0, 60 * M_PI / 180, 1.0, M_PI})(cone));
}
