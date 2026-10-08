#include <gmock/gmock.h>

#include <filesystem>

#include "map_lib/io/reader.hpp"
#include "map_lib/testing/cone.hpp"
#include "map_lib/transform.hpp"
#include "map_lib/type/cone.hpp"
#include "map_lib/type/map.hpp"

#include "make_types.hpp"  // NOLINT

#include <ament_index_cpp/get_package_share_directory.hpp>

using eufs::map::Color;
using eufs::map::Cone;
using eufs::map::Map;

using eufs::map::Transformation;

class TransformTests : public ::testing::Test {
 public:
  Map<Cone> map;

  void SetUp() override {
    auto package_share_directory = ament_index_cpp::get_package_share_directory("map_lib");
    auto map_path = std::filesystem::path(package_share_directory) / "test" / "reference_map.csv";
    map = eufs::map::io::LoadCones<Cone>(map_path);
  }
};

inline auto UnchangedResultEq() {
  return ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(MakeCone({0.0, 0.0}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({2.4, 2.0}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({0.0, 2.4}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({2.2, 0.0}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({1.2, 1.0}, {0.4, 0.0, 0.0, 0.2}, Color::ORANGE)),
       eufs::testing::ConeEq(MakeCone({2.0, 2.2}, {0.2, 0.4, 0.4, 0.4}, Color::ORANGE))});
}

TEST_F(TransformTests, CheckRotationZeroInPlace) {
  Transformation::Rotate(map, 0.0);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckRotationZeroCopy) {
  const Map<Cone> result = Transformation::Rotate(std::as_const(map), 0.0);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckInverseRotationZeroInPlace) {
  Transformation::Rotate<Map<Cone>, true>(map, 0.0);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckInverseRotationZeroCopy) {
  const Map<Cone> result = Transformation::Rotate<Map<Cone>, true>(std::as_const(map), 0.0);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

inline auto RotationResultEq() {
  return ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(MakeCone({0.0, 0.0}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({-2.4, -2.0}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({0.0, -2.4}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({-2.2, 0.0}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({-1.2, -1.0}, {0.4, 0.0, 0.0, 0.2}, Color::ORANGE)),
       eufs::testing::ConeEq(MakeCone({-2.0, -2.2}, {0.2, 0.4, 0.4, 0.4}, Color::ORANGE))});
}

TEST_F(TransformTests, CheckRotationNonZeroInPlace) {
  Transformation::Rotate(map, M_PI);

  ASSERT_THAT(map.ToCones(), RotationResultEq());
}

TEST_F(TransformTests, CheckRotationNonZeroCopy) {
  const Map<Cone> result = Transformation::Rotate(std::as_const(map), M_PI);

  ASSERT_THAT(result.ToCones(), RotationResultEq());
}

inline auto InverseRotationResultEq() {
  return ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(MakeCone({0.0, 0.0}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({2.0, -2.4}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({2.4, 0.0}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({0.0, -2.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({1.0, -1.2}, {0.4, 0.0, 0.0, 0.2}, Color::ORANGE)),
       eufs::testing::ConeEq(MakeCone({2.2, -2.0}, {0.2, 0.4, 0.4, 0.4}, Color::ORANGE))});
}

TEST_F(TransformTests, CheckInverseRotationNonZeroInPlace) {
  Transformation::Rotate<Map<Cone>, true>(map, M_PI / 2);

  ASSERT_THAT(map.ToCones(), InverseRotationResultEq());
}

TEST_F(TransformTests, CheckInverseRotationNonZeroCopy) {
  const Map<Cone> result = Transformation::Rotate<Map<Cone>, true>(std::as_const(map), M_PI / 2);

  ASSERT_THAT(result.ToCones(), InverseRotationResultEq());
}

TEST_F(TransformTests, CheckTranslationZeroInPlace) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  Transformation::Translate(map, trans);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckTranslationZeroCopy) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  const Map<Cone> result = Transformation::Translate(std::as_const(map), trans);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckInverseTranslationZeroInPlace) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  Transformation::Translate<Map<Cone>, true>(map, trans);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckInverseTranslationZeroCopy) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  const Map<Cone> result = Transformation::Translate<Map<Cone>, true>(std::as_const(map), trans);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

inline auto TranslationResultEq() {
  return ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(MakeCone({-0.2, 0.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({2.2, 2.2}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({-0.2, 2.6}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({2.0, 0.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({1.0, 1.2}, {0.4, 0.0, 0.0, 0.2}, Color::ORANGE)),
       eufs::testing::ConeEq(MakeCone({1.8, 2.4}, {0.2, 0.4, 0.4, 0.4}, Color::ORANGE))});
}

TEST_F(TransformTests, CheckTranslationNonZeroInPlace) {
  const Eigen::Vector2d trans{-0.2, 0.2};

  Transformation::Translate(map, trans);

  ASSERT_THAT(map.ToCones(), TranslationResultEq());
}

TEST_F(TransformTests, CheckTranslationNonZeroCopy) {
  const Eigen::Vector2d trans{-0.2, 0.2};

  const Map<Cone> result = Transformation::Translate(std::as_const(map), trans);

  ASSERT_THAT(result.ToCones(), TranslationResultEq());
}

inline auto InverseTranslationResultEq() {
  return ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(MakeCone({0.2, -0.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({2.6, 1.8}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({0.2, 2.2}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({2.4, -0.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({1.4, 0.8}, {0.4, 0.0, 0.0, 0.2}, Color::ORANGE)),
       eufs::testing::ConeEq(MakeCone({2.2, 2.0}, {0.2, 0.4, 0.4, 0.4}, Color::ORANGE))});
}

TEST_F(TransformTests, CheckInverseTranslationNonZeroInPlace) {
  const Eigen::Vector2d trans{-0.2, 0.2};

  Transformation::Translate<Map<Cone>, true>(map, trans);

  ASSERT_THAT(map.ToCones(), InverseTranslationResultEq());
}

TEST_F(TransformTests, CheckInverseTranslationNonZeroCopy) {
  const Eigen::Vector2d trans{-0.2, 0.2};

  const Map<Cone> result = Transformation::Translate<Map<Cone>, true>(std::as_const(map), trans);

  ASSERT_THAT(result.ToCones(), InverseTranslationResultEq());
}

TEST_F(TransformTests, CheckTransformationZeroInPlace) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  Transformation::Transform(map, 0.0, trans);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckTransformationZeroCopy) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  const Map<Cone> result = Transformation::Transform(std::as_const(map), 0.0, trans);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckInverseTransformationZeroInPlace) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  Transformation::Transform<Map<Cone>, true>(map, 0.0, trans);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckInverseTransformationZeroCopy) {
  const Eigen::Vector2d trans = Eigen::Vector2d::Zero();
  const Map<Cone> result =
    Transformation::Transform<Map<Cone>, true>(std::as_const(map), 0.0, trans);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

inline auto TransformationResultEq() {
  return ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(MakeCone({-0.2, 0.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({-2.6, -1.8}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({-0.2, -2.2}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({-2.4, 0.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({-1.4, -0.8}, {0.4, 0.0, 0.0, 0.2}, Color::ORANGE)),
       eufs::testing::ConeEq(MakeCone({-2.2, -2.0}, {0.2, 0.4, 0.4, 0.4}, Color::ORANGE))});
}

TEST_F(TransformTests, CheckTransformationNonZeroInPlace) {
  const Eigen::Vector2d trans{-0.2, 0.2};
  Transformation::Transform(map, M_PI, trans);

  ASSERT_THAT(map.ToCones(), TransformationResultEq());
}

TEST_F(TransformTests, CheckTransformationNonZeroCopy) {
  const Eigen::Vector2d trans{-0.2, 0.2};
  const Map<Cone> result = Transformation::Transform(std::as_const(map), M_PI, trans);

  ASSERT_THAT(result.ToCones(), TransformationResultEq());
}

inline auto InverseTransformationResultEq() {
  return ::testing::ElementsAreArray(
      {eufs::testing::ConeEq(MakeCone({-0.2, -0.2}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({1.8, -2.6}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({2.2, -0.2}, {0.4, 0.0, 0.0, 0.2}, Color::BLUE)),
       eufs::testing::ConeEq(MakeCone({-0.2, -2.4}, {0.4, 0.0, 0.0, 0.2}, Color::YELLOW)),
       eufs::testing::ConeEq(MakeCone({0.8, -1.4}, {0.4, 0.0, 0.0, 0.2}, Color::ORANGE)),
       eufs::testing::ConeEq(MakeCone({2.0, -2.2}, {0.2, 0.4, 0.4, 0.4}, Color::ORANGE))});
}

TEST_F(TransformTests, CheckInverseTransformationNonZeroInPlace) {
  const Eigen::Vector2d trans{-0.2, 0.2};
  Transformation::Transform<Map<Cone>, true>(map, M_PI / 2, trans);

  ASSERT_THAT(map.ToCones(), InverseTransformationResultEq());
}

TEST_F(TransformTests, CheckInverseTransformationNonZeroCopy) {
  const Eigen::Vector2d trans{-0.2, 0.2};
  const Map<Cone> result =
    Transformation::Transform<Map<Cone>, true>(std::as_const(map), M_PI / 2, trans);

  ASSERT_THAT(result.ToCones(), InverseTransformationResultEq());
}

TEST_F(TransformTests, CheckIdentityAffineTransformationInPlace) {
  const Eigen::Isometry2d affine_transform = Eigen::Isometry2d::Identity();
  Transformation::Transform(map, affine_transform);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckIdentityAffineTransformationCopy) {
  const Eigen::Isometry2d affine_transform = Eigen::Isometry2d::Identity();
  const Map<Cone> result = Transformation::Transform(std::as_const(map), affine_transform);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckIdentityAffineInverseTransformationInPlace) {
  const Eigen::Isometry2d affine_transform = Eigen::Isometry2d::Identity();
  Transformation::Transform<Map<Cone>, true>(map, affine_transform);

  ASSERT_THAT(map.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckIdentityAffineInverseTransformationCopy) {
  const Eigen::Isometry2d affine_transform = Eigen::Isometry2d::Identity();
  const Map<Cone> result =
    Transformation::Transform<Map<Cone>, true>(std::as_const(map), affine_transform);

  ASSERT_THAT(result.ToCones(), UnchangedResultEq());
}

TEST_F(TransformTests, CheckAffineTransformationInPlace) {
  Eigen::Isometry2d isometry;
  isometry.linear() = Eigen::Rotation2D(M_PI).matrix();
  isometry.translation() = Eigen::Vector2d{-0.2, 0.2};

  Transformation::Transform(map, isometry);

  ASSERT_THAT(map.ToCones(), TransformationResultEq());
}

TEST_F(TransformTests, CheckAffineTransformationCopy) {
  Eigen::Isometry2d isometry;
  isometry.linear() = Eigen::Rotation2D(M_PI).matrix();
  isometry.translation() = Eigen::Vector2d{-0.2, 0.2};
  const Map<Cone> result = Transformation::Transform(std::as_const(map), isometry);

  ASSERT_THAT(result.ToCones(), TransformationResultEq());
}

TEST_F(TransformTests, CheckAffineInverseTransformationInPlace) {
  Eigen::Isometry2d isometry;
  isometry.linear() = Eigen::Rotation2D(M_PI / 2).matrix();
  isometry.translation() = Eigen::Vector2d{-0.2, 0.2};

  Transformation::Transform<Map<Cone>, true>(map, isometry);

  ASSERT_THAT(map.ToCones(), InverseTransformationResultEq());
}

TEST_F(TransformTests, CheckAffineInverseTransformationCopy) {
  Eigen::Isometry2d isometry;
  isometry.linear() = Eigen::Rotation2D(M_PI / 2).matrix();
  isometry.translation() = Eigen::Vector2d{-0.2, 0.2};
  const Map<Cone> result = Transformation::Transform<Map<Cone>, true>(std::as_const(map), isometry);

  ASSERT_THAT(result.ToCones(), InverseTransformationResultEq());
}

