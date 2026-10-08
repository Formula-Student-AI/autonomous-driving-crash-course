#include <gmock/gmock.h>

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eufs_gmock_matchers/eigen_matchers.hpp>

#include "map_lib/association/align/align_svd.hpp"
#include "map_lib/io/reader.hpp"
#include "map_lib/transform.hpp"
#include "map_lib/type/cone.hpp"
#include "map_lib/type/map.hpp"

using eufs::map::Cone;
using eufs::map::Map;

TEST(CheckICP, NormalRotation) {
  auto package_share_directory = ament_index_cpp::get_package_share_directory("map_lib");
  auto map_path = std::filesystem::path(package_share_directory) / "test" / "reference_map.csv";
  const Map<Cone> map(eufs::map::io::LoadCones<Cone>(map_path));

  Eigen::Vector2d trans{0.2, 0.1};

  const double angle{7 * M_PI / 36};
  const Map<Cone> transformed_map(eufs::map::Transformation::Transform(map, angle, trans));

  const auto alignment =
    eufs::map::association::align::AlignSVD(map.GetPositions(), transformed_map.GetPositions());
  const Eigen::Affine2d res = alignment.FindTransformation();

  ASSERT_THAT(res.translation(), eufs::testing::matchers::EigenEq(trans));
  ASSERT_THAT(res.rotation(), eufs::testing::matchers::EigenEq(
                      Eigen::Rotation2D<double>(angle).toRotationMatrix()));
}
