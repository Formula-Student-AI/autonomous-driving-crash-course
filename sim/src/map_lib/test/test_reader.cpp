#include <gmock/gmock.h>

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "map_lib/io/reader.hpp"
#include "map_lib/type/cone.hpp"
#include "map_lib/testing/cone.hpp"

using eufs::map::Color;
using eufs::map::Cone;

///////////////////////////////////
//  Helper method to make cones  //
///////////////////////////////////

Cone MakeCone(const std::array<double, 2> &pos, const std::array<double, 4> &cov,
              const Color &color = Color::UNKNOWN) {
  return Cone(pos, cov, color);
}

////////////////////////////////////
//  Check ValidEUFSFormat Method  //
////////////////////////////////////

TEST(CheckValidEUFSFormat, CheckTrue) {
  EXPECT_TRUE(
      eufs::map::io::ValidEUFSFormat("tag,x,y,direction,x_variance,y_variance,xy_covariance"));
  EXPECT_TRUE(
      eufs::map::io::ValidEUFSFormat("tag,x,y,direction,x_variance,y_variance,xy_covariance,a,b"));
}

TEST(CheckValidEUFSFormat, CheckFalse) {
  // Different values
  EXPECT_FALSE(
      eufs::map::io::ValidEUFSFormat("type,x,y,direction,x_variance,y_variance,xy_covariance"));
  EXPECT_FALSE(
      eufs::map::io::ValidEUFSFormat("tag,z,y,direction,x_variance,y_variance,xy_covariance"));
  EXPECT_FALSE(
      eufs::map::io::ValidEUFSFormat("tag,x,p,direction,x_variance,y_variance,xy_covariance"));
  EXPECT_FALSE(eufs::map::io::ValidEUFSFormat("tag,x,y,d,x_variance,y_variance,xy_covariance"));
  EXPECT_FALSE(eufs::map::io::ValidEUFSFormat("tag,x,y,direction,x_var,y_variance,xy_covariance"));
  EXPECT_FALSE(eufs::map::io::ValidEUFSFormat("tag,x,y,direction,x_variance,y_var,xy_covariance"));
  EXPECT_FALSE(eufs::map::io::ValidEUFSFormat("tag,x,y,direction,x_variance,y_variance,xy_cov"));
  // Different lengths
  EXPECT_FALSE(eufs::map::io::ValidEUFSFormat("tag,x,y,x_variance,y_variance,xy_covariance"));
}

/////////////////////////////////
//  Check ConvertToCone Method //
/////////////////////////////////

TEST(CheckConvertToCone, CheckSuccess) {
  EXPECT_THAT(
      eufs::map::io::ConvertToCone<Cone>({"blue", "1.0", "0.9", "0", "0.5", "0.3", "0.1"}),
      eufs::testing::ConeEq(MakeCone({1.0, 0.9}, {0.5, 0.1, 0.1, 0.3}, Color::BLUE)));
  EXPECT_THAT(
      eufs::map::io::ConvertToCone<Cone>({"yellow", "1.0", "0.9", "0", "0.5", "0.3", "0.1"}),
      eufs::testing::ConeEq(MakeCone({1.0, 0.9}, {0.5, 0.1, 0.1, 0.3}, Color::YELLOW)));
  EXPECT_THAT(
      eufs::map::io::ConvertToCone<Cone>({"orange", "1.0", "0.9", "0", "0.5", "0.3", "0.1"}),
      eufs::testing::ConeEq(MakeCone({1.0, 0.9}, {0.5, 0.1, 0.1, 0.3}, Color::ORANGE)));
  EXPECT_THAT(
      eufs::map::io::ConvertToCone<Cone>({"big_orange", "1.0", "0.9", "0", "0.5", "0.3", "0.1"}),
      eufs::testing::ConeEq(MakeCone({1.0, 0.9}, {0.5, 0.1, 0.1, 0.3}, Color::ORANGE)));
  EXPECT_THAT(
      eufs::map::io::ConvertToCone<Cone>({"unknown", "1.0", "0.9", "0", "0.5", "0.3", "0.1"}),
      eufs::testing::ConeEq(MakeCone({1.0, 0.9}, {0.5, 0.1, 0.1, 0.3}, Color::UNKNOWN)));
}

//////////////////////////////
//  Check LoadCones Method  //
//////////////////////////////

TEST(CheckLoadCones, CheckSuccess) {
  auto package_share_directory = ament_index_cpp::get_package_share_directory("map_lib");
  auto map_path = std::filesystem::path(package_share_directory) / "test" / "test_map.csv";
  EXPECT_THAT(eufs::map::io::LoadCones<Cone>(map_path),
              ::testing::ElementsAreArray({
                eufs::testing::ConeEq(MakeCone({1.1, 0.9}, {0.1, 0.3, 0.3, 0.2}, Color::BLUE)),
                eufs::testing::ConeEq(MakeCone({0.1, 0.2}, {0.4, 0.6, 0.6, 0.5}, Color::YELLOW))
              }));
}
