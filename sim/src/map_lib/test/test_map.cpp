#include <gmock/gmock.h>

#include <array>
#include <algorithm>

#include "map_lib/type/cone.hpp"
#include "map_lib/type/map.hpp"
#include "map_lib/testing/cone.hpp"

using eufs::map::Map;
using eufs::map::Cone;
using eufs::map::Color;

class MapTest : public ::testing::Test {
 protected:
  MapTest() : cones_{MapTest::MakeCone({0.1, 0.4}, {0.2, 0.0, 0.0, 0.1}, Color::BLUE),
                     MapTest::MakeCone({1.1, 1.4}, {1.2, 0.0, 0.0, 1.1}, Color::YELLOW),
                     MapTest::MakeCone({2.1, 2.4}, {2.2, 0.0, 0.0, 2.1}, Color::ORANGE),
                     MapTest::MakeCone({3.1, 3.4}, {3.2, 0.0, 0.0, 3.1}, Color::ORANGE),
                     MapTest::MakeCone({4.1, 4.4}, {4.2, 0.0, 0.0, 4.1}, Color::UNKNOWN)} {
  }

 public:
  static Cone MakeCone(const std::array<double, 2> &pos, const std::array<double, 4> &cov,
              const Color &color = Color::UNKNOWN) {
    return {pos, cov, color};
  }

 protected:
  const std::vector<Cone> cones_;
};


///////////////////
//  Check Build  //
///////////////////

TEST(CheckBuild, Build) { auto a = Map<Cone>(); }


//////////////////////////
//  Check Adding Cones  //
//////////////////////////

TEST_F(MapTest, CheckAddCone) {  // check add one cone
  auto map = Map<Cone>();

  // Test for empty map
  ASSERT_THAT(map.ToCones(), ::testing::IsEmpty());
  EXPECT_EQ(map.begin(), map.end());

  // Add one cone to empty map
  map.Add(cones_[0]);
  ASSERT_THAT(map.ToCones(), ::testing::ElementsAreArray(
          {eufs::testing::ConeEq(cones_[0])}));

  // Add another cone to map
  map.Add(cones_[1]);
  ASSERT_THAT(map.ToCones(), ::testing::ElementsAreArray(
          {eufs::testing::ConeEq(cones_[0]),
          eufs::testing::ConeEq(cones_[1])}));
}

TEST_F(MapTest, CheckAddCones) {  // check add multiple cones
  auto map = Map<Cone>();

  map.Add(cones_);  // add vector of cones

  // check cones in map
  ASSERT_THAT(map.ToCones(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[0]),
      eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[2]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[4])}));
}


////////////////////////////
//  Check Removing Cones  //
////////////////////////////

TEST_F(MapTest, CheckRemoveCone) {
  auto map = Map<Cone>(cones_);
  map.Remove(0);  // remove first cone in vector
  EXPECT_THAT(map.ToCones(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[2]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[4])}));

  map.Remove(3);  // remove last cone in vector
  EXPECT_THAT(map.ToCones(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[2]),
      eufs::testing::ConeEq(cones_[3])}));

  map.Remove(1);  // remove middle cone in vector
  EXPECT_THAT(map.ToCones(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[3])}));
}

////////////////////////////
//  Check Removing Cones  //
////////////////////////////

TEST_F(MapTest, CheckEraseIf) {
  auto map = Map<Cone>(cones_);
  map.EraseIf([](const Cone &cone){
    return cone.GetColor() == Color::ORANGE;
  });

  EXPECT_THAT(map.ToCones(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[0]),
      eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[4])}));
}

///////////////////////
//  Check Accessors  //
///////////////////////

TEST_F(MapTest, CheckAccessorNoColor) {
  auto map = Map<Cone>(cones_);

  ASSERT_THAT(map.Get(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[0]),
      eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[2]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[4])}));
}

TEST_F(MapTest, CheckAccessorColor) {
  auto map = Map<Cone>(cones_);

  ASSERT_THAT(map.Get(Color::ORANGE), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[2]),
      eufs::testing::ConeEq(cones_[3])}));
}

TEST_F(MapTest, CheckAccessorIndices) {
  auto map = Map<Cone>(cones_);

  const std::vector<size_t> indices{1, 3, 4};

  ASSERT_THAT(map.Get(indices), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[4])}));
}

TEST_F(MapTest, CheckPositionAccessorNoColor) {
  auto map = Map<Cone>(cones_);

  ASSERT_THAT(map.GetPositions(), ::testing::ElementsAreArray(
     {eufs::testing::matchers::EigenEq(cones_[0].position),
      eufs::testing::matchers::EigenEq(cones_[1].position),
      eufs::testing::matchers::EigenEq(cones_[2].position),
      eufs::testing::matchers::EigenEq(cones_[3].position),
      eufs::testing::matchers::EigenEq(cones_[4].position)}));
}

TEST_F(MapTest, CheckPositionAccessorColor) {
  auto map = Map<Cone>(cones_);

  ASSERT_THAT(map.GetPositions(Color::ORANGE), ::testing::ElementsAreArray(
     {eufs::testing::matchers::EigenEq(cones_[2].position),
      eufs::testing::matchers::EigenEq(cones_[3].position)}));
}

TEST_F(MapTest, CheckPositionAccessorIndices) {
  auto map = Map<Cone>(cones_);

  const std::vector<size_t> indices{1, 3, 4};

  ASSERT_THAT(map.GetPositions(indices), ::testing::ElementsAreArray(
     {eufs::testing::matchers::EigenEq(cones_[1].position),
      eufs::testing::matchers::EigenEq(cones_[3].position),
      eufs::testing::matchers::EigenEq(cones_[4].position)}));
}

///////////////////////
//  Check Iterators  //
///////////////////////

TEST_F(MapTest, CheckIterators) {
  auto map = Map<Cone>(cones_);
  for (auto &c : map) {
    c = cones_[3];
  }
  ASSERT_THAT(map.ToCones(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[3])}));
}

TEST_F(MapTest, CheckIteratorsConst) {
  size_t i = 0;
  const auto const_map = Map<Cone>(cones_);
  for (const auto &c : const_map) {
    ASSERT_THAT(c, eufs::testing::ConeEq(cones_[i++]));
  }
}

TEST_F(MapTest, CheckConstIterators) {
  const auto const_map = Map<Cone>(cones_);
  std::vector<Cone> vec;
  std::transform(const_map.cbegin(), const_map.cend(), std::back_inserter(vec),
                 [](const auto &c){ return c;});
  ASSERT_THAT(vec, ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(cones_[0]),
       eufs::testing::ConeEq(cones_[1]),
       eufs::testing::ConeEq(cones_[2]),
       eufs::testing::ConeEq(cones_[3]),
       eufs::testing::ConeEq(cones_[4])}));
}

////////////////////////////
//  Check Access Methods  //
////////////////////////////

TEST_F(MapTest, CheckAccessMethods) {  // check 2 different access methods
  // test the non-constant access method
  auto map = Map<Cone>(cones_);
  map.at(0) = cones_[1];
  EXPECT_THAT(map.at(0), eufs::testing::ConeEq(cones_[1]));
  EXPECT_THAT(map.at(1), eufs::testing::ConeEq(cones_[1]));
  EXPECT_THAT(map.at(2), eufs::testing::ConeEq(cones_[2]));
  EXPECT_THAT(map.at(3), eufs::testing::ConeEq(cones_[3]));
  EXPECT_THAT(map.at(4), eufs::testing::ConeEq(cones_[4]));

  // test the constant access method
  const auto map_const = Map<Cone>(cones_);
  EXPECT_THAT(map_const.at(0), eufs::testing::ConeEq(cones_[0]));
  EXPECT_THAT(map_const.at(1), eufs::testing::ConeEq(cones_[1]));
  EXPECT_THAT(map_const.at(2), eufs::testing::ConeEq(cones_[2]));
  EXPECT_THAT(map_const.at(3), eufs::testing::ConeEq(cones_[3]));
  EXPECT_THAT(map_const.at(4), eufs::testing::ConeEq(cones_[4]));
}


//////////////////////////
//  Check Size Methods  //
//////////////////////////

TEST_F(MapTest, CheckSizeMethods) {
  auto map = Map<Cone>(cones_);
  EXPECT_EQ(5lu, map.size());
  map.Remove(0);
  EXPECT_EQ(4lu, map.size());
  map.Remove(0);
  EXPECT_EQ(3lu, map.size());
  map.Remove(0);
  EXPECT_EQ(2lu, map.size());
  map.Remove(0);
  EXPECT_EQ(1lu, map.size());
  map.Remove(0);
  EXPECT_EQ(0lu, map.size());
}


/////////////////////////////////
//  Check Map Representations  //
/////////////////////////////////

TEST_F(MapTest, CheckToCones) {
  auto map = Map<Cone>(cones_);
  ASSERT_THAT(map.ToCones(), ::testing::ElementsAreArray(
     {eufs::testing::ConeEq(cones_[0]),
      eufs::testing::ConeEq(cones_[1]),
      eufs::testing::ConeEq(cones_[2]),
      eufs::testing::ConeEq(cones_[3]),
      eufs::testing::ConeEq(cones_[4])}));
}
