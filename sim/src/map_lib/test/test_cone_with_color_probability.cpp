#include <eigen3/Eigen/Dense>
#include <gmock/gmock.h>

#include <array>
#include <rclcpp/time.hpp>
#include <eufs_gmock_matchers/eufs_matchers.hpp>

#include "map_lib/type/cone_with_color_probability.hpp"
#include "map_lib/type/color_probabilities.hpp"

#include "map_lib/testing/color_probabilities.hpp"
#include "map_lib/testing/cone_with_color_probability.hpp"

using eufs::map::ConeWithColorProbability;
using eufs::map::ColorProbabilities;

using eufs::map::ToMsg;
using eufs::map::FromMsg;

rclcpp::Time MakeTime(int seconds) {
  return {seconds, 0, rcl_clock_type_t::RCL_ROS_TIME};
}

Eigen::Matrix2d MakeCovariance(const std::array<double, 4> &cov) {
  Eigen::Matrix2d mat;
  mat << cov[0], cov[2], cov[1], cov[3];  // column-major
  return mat;
}

ConeWithColorProbability MakeCone(const std::array<double, 2> &pos,
  const std::array<double, 4> &cov,
  const ColorProbabilities &color_probabilities = {
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.}) {
  return {pos, cov, color_probabilities};
}

auto MakeConeMsg(const std::array<double, 2> &pos, const std::array<double, 4> &cov) {
  eufs_msgs::msg::ConeWithCovariance msg;
  msg.point.x = pos[0];
  msg.point.y = pos[1];
  msg.covariance = cov;
  return msg;
}

auto MakeConeWithColorProbabilityMsg(
  const std::array<double, 2> &pos,
  const std::array<double, 4> &cov,
  const ColorProbabilities &color_probabilities = {
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.}) {
  eufs_msgs::msg::ConeWithColorProbability msg;
  msg.point.x = pos[0];
  msg.point.y = pos[1];

  msg.blue_prob = color_probabilities.blue;
  msg.yellow_prob = color_probabilities.yellow;
  msg.orange_prob = color_probabilities.orange;
  msg.big_orange_prob = color_probabilities.big_orange;
  msg.unknown_prob = color_probabilities.unknown;

  msg.covariance = cov;
  return msg;
}

auto MakeHeader(const rclcpp::Time &time, const std::string &frame_id) {
  std_msgs::msg::Header msg;
  msg.stamp = time, msg.frame_id = frame_id;
  return msg;
}

auto MakeConeArrayMsg(
    const std::vector<eufs_msgs::msg::ConeWithCovariance> &blue,
    const std::vector<eufs_msgs::msg::ConeWithCovariance> &yellow,
    const std::vector<eufs_msgs::msg::ConeWithCovariance> &orange,
    const std::vector<eufs_msgs::msg::ConeWithCovariance> &big_orange,
    const std::vector<eufs_msgs::msg::ConeWithCovariance> &unknown,
    const std_msgs::msg::Header &header = MakeHeader(rclcpp::Time(0), "")) {
  eufs_msgs::msg::ConeArrayWithCovariance msg;
  msg.header = header;
  msg.blue_cones = blue;
  msg.yellow_cones = yellow;
  msg.orange_cones = orange;
  msg.big_orange_cones = big_orange;
  msg.unknown_color_cones = unknown;
  return msg;
}

auto MakeConeWithColorProbabilityArrayMsg(
    const std::vector<eufs_msgs::msg::ConeWithColorProbability> &cones,
    const std_msgs::msg::Header &header = MakeHeader(rclcpp::Time(0), "")) {
  eufs_msgs::msg::ConeWithColorProbabilityArray msg;
  msg.header = header;
  msg.cones = cones;
  return msg;
}

///////////////////////
// Check Constructor //
///////////////////////

TEST(CheckConstructor, Default) {
  auto cone = ConeWithColorProbability();

  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d::Zero()));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(Eigen::Matrix2d::Zero()));

  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.
  }));
}

TEST(CheckConstructor, EigenPositionNoColor) {
  auto cone = ConeWithColorProbability(Eigen::Vector2d{1.0, 9.0});
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.0
  }));
}

TEST(CheckConstructor, EigenPositionColor) {
  auto cone = ConeWithColorProbability(Eigen::Vector2d{1.0, 9.0}, ColorProbabilities{
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  });
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  }));
}

TEST(CheckConstructor, ArrayPositionNoColor) {
  auto cone = ConeWithColorProbability(std::array<double, 2>{1.0, 9.0});
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.0
  }));
}

TEST(CheckConstructor, ArrayPositionColor) {
  auto cone = ConeWithColorProbability(std::array<double, 2>{1.0, 9.0}, {
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  });
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  }));
}

TEST(CheckConstructor, EigenPositionCovarianceNoColor) {
  auto cone = ConeWithColorProbability(
    Eigen::Vector2d{1.0, 9.0},
    MakeCovariance({0.1, 0.2, 0.3, 0.4}));
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.0
  }));
}

TEST(CheckConstructor, EigenPositionCovarianceColor) {
  auto cone =
      ConeWithColorProbability(Eigen::Vector2d{1.0, 9.0}, MakeCovariance({0.1, 0.2, 0.3, 0.4}), {
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  });
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  }));
}

TEST(CheckConstructor, ArrayPositionCovarianceNoColor) {
  auto cone = ConeWithColorProbability(
    std::array<double, 2>{1.0, 9.0},
    std::array<double, 4>{0.1, 0.2, 0.3, 0.4});
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.0
  }));
}

TEST(CheckConstructor, ArrayPositionCovarianceColor) {
  auto cone = ConeWithColorProbability(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, {
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  });
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  }));
}

TEST(CheckConstructor, MsgNoColor) {
  auto msg = MakeConeMsg(std::array<double, 2>{0.1, 0.2}, {0.1, 0.2, 0.3, 0.4});
  auto cone = ConeWithColorProbability(msg);
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{0.1, 0.2}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.0
  }));
}

TEST(CheckConstructor, MsgColor) {
  auto msg = MakeConeMsg({0.1, 0.2}, {0.1, 0.2, 0.3, 0.4});
  auto cone = ConeWithColorProbability(msg, {
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  });
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{0.1, 0.2}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
    .blue = .0,
    .yellow = 1.0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  }));
}

/////////////////////////////////
// Check GetColorProbabilities //
/////////////////////////////////

TEST(CheckGetColorConst, Blue) {
  auto cone = ConeWithColorProbability(
    std::array<double, 2>{1.0, 9.0},
    {0.1, 0.2, 0.3, 0.4},
    {
      .blue = 1.0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
      .blue = 1.0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    }));
}

TEST(CheckGetColorConst, Yellow) {
  auto cone = ConeWithColorProbability(
    std::array<double, 2>{1.0, 9.0},
    {0.1, 0.2, 0.3, 0.4},
    {
      .blue = .0,
      .yellow = 1.0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
      .blue = .0,
      .yellow = 1.0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    }));
}

TEST(CheckGetColorConst, Orange) {
  auto cone = ConeWithColorProbability(
    std::array<double, 2>{1.0, 9.0},
    {0.1, 0.2, 0.3, 0.4},
    {
      .blue = .0,
      .yellow = .0,
      .orange = 1.0,
      .big_orange = .0,
      .unknown = .0
    });
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
      .blue = .0,
      .yellow = .0,
      .orange = 1.0,
      .big_orange = .0,
      .unknown = .0
    }));
}

TEST(CheckGetColorConst, Unknown) {
  auto cone = ConeWithColorProbability(
    std::array<double, 2>{1.0, 9.0},
    {0.1, 0.2, 0.3, 0.4},
    {
      .blue = .0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = 1.0
    });
  ASSERT_THAT(cone.GetColorProbabilities(), eufs::testing::ColorProbabilitiesEq({
      .blue = .0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = 1.0
    }));
}


// /////////////////////
// //  Check FromMsg  //
// /////////////////////

TEST(CheckFromMsg, None) {
  auto result = FromMsg<ConeWithColorProbability>(MakeConeArrayMsg({}, {}, {}, {}, {}));
  EXPECT_EQ(result.size(), 0lu);
}

TEST(CheckFromMsg, One) {
  auto blues = {MakeConeMsg({0.1, 0.4}, {0.2, 0.0, 0.0, 0.1})};
  auto result = FromMsg<ConeWithColorProbability>(MakeConeArrayMsg(blues, {}, {}, {}, {}));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {0.1, 0.4},
              {0.2, 0.0, 0.0, 0.1},
              {
                .blue = 1.0,
                .yellow = .0,
                .orange = .0,
                .big_orange = .0,
                .unknown = .0
              })),
      }));
}

TEST(CheckFromMsg, OneOfEachColor) {
  auto blues = {MakeConeMsg({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1})};
  auto yellows = {MakeConeMsg({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1})};
  auto oranges = {MakeConeMsg({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1})};
  auto big_oranges = {MakeConeMsg({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1})};
  auto unknowns = {MakeConeMsg({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1})};
  auto result = FromMsg<ConeWithColorProbability>(
    MakeConeArrayMsg(blues, yellows, oranges, big_oranges, unknowns));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {0.1, 0.4},
              {1.2, 0.0, 0.0, 1.1},
              {
                .blue = 1.0,
                .yellow = .0,
                .orange = .0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {1.1, 1.4},
              {2.2, 0.0, 0.0, 2.1},
              {
                .blue = .0,
                .yellow = 1.0,
                .orange = .0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {2.1, 2.4},
              {3.2, 0.0, 0.0, 3.1},
              {
                .blue = .0,
                .yellow = .0,
                .orange = 1.0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {3.1, 3.4},
              {4.2, 0.0, 0.0, 4.1},
              {
                .blue = .0,
                .yellow = .0,
                .orange = 1.0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {4.1, 4.4},
              {5.2, 0.0, 0.0, 5.1},
              {
                .blue = .0,
                .yellow = .0,
                .orange = .0,
                .big_orange = .0,
                .unknown = 1.0
              })),
      }));
}

TEST(CheckFromMsg, MultipleColor) {
  auto blues = {MakeConeMsg({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1}),
                MakeConeMsg({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1})};
  auto oranges = {MakeConeMsg({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1})};
  auto big_oranges = {MakeConeMsg({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1})};
  auto unknowns = {MakeConeMsg({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1})};
  auto result = FromMsg<ConeWithColorProbability>(
    MakeConeArrayMsg(blues, {}, oranges, big_oranges, unknowns));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {0.1, 0.4},
              {1.2, 0.0, 0.0, 1.1},
              {
                .blue = 1.0,
                .yellow = .0,
                .orange = .0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {1.1, 1.4},
              {2.2, 0.0, 0.0, 2.1},
              {
                .blue = 1.0,
                .yellow = .0,
                .orange = .0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {2.1, 2.4},
              {3.2, 0.0, 0.0, 3.1},
              {
                .blue = .0,
                .yellow = .0,
                .orange = 1.0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {3.1, 3.4},
              {4.2, 0.0, 0.0, 4.1},
              {
                .blue = .0,
                .yellow = .0,
                .orange = 1.0,
                .big_orange = .0,
                .unknown = .0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {4.1, 4.4},
              {5.2, 0.0, 0.0, 5.1},
              {
                .blue = .0,
                .yellow = .0,
                .orange = .0,
                .big_orange = .0,
                .unknown = 1.0
              })),
      }));
}

TEST(CheckFromMsg, NoneWithColorProbability) {
  auto result = FromMsg<ConeWithColorProbability>(MakeConeWithColorProbabilityArrayMsg({}));
  EXPECT_EQ(result.size(), 0lu);
}

TEST(CheckFromMsg, OneWithColorProbability) {
  auto cones = {MakeConeWithColorProbabilityMsg(
    {0.1, 0.4},
    {0.2, 0.0, 0.0, 0.1},
    {
      .blue = 0.6,
      .yellow = 0.2,
      .orange = 0.1,
      .big_orange = 0.0,
      .unknown = 0.1
    })};
  auto result = FromMsg<ConeWithColorProbability>(MakeConeWithColorProbabilityArrayMsg(cones));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {0.1, 0.4},
              {0.2, 0.0, 0.0, 0.1},
              {
                .blue = 0.6,
                .yellow = 0.2,
                .orange = 0.1,
                .big_orange = 0.0,
                .unknown = 0.1
              }))
      }));
}

TEST(CheckFromMsg, OneOfEachColorWithColorProbability) {
  auto cones = {
    MakeConeWithColorProbabilityMsg(
      {0.1, 0.4},
      {1.2, 0.0, 0.0, 1.1},
      {
        .blue = 0.6,
        .yellow = 0.2,
        .orange = 0.1,
        .big_orange = 0.0,
        .unknown = 0.1
      }),
    MakeConeWithColorProbabilityMsg(
      {1.1, 1.4},
      {2.2, 0.0, 0.0, 2.1},
      {
        .blue = 0.0,
        .yellow = 0.5,
        .orange = 0.0,
        .big_orange = 0.0,
        .unknown = 0.5
      }),
    MakeConeWithColorProbabilityMsg(
      {2.1, 2.4},
      {3.2, 0.0, 0.0, 3.1},
      {
        .blue = 0.0,
        .yellow = 0.0,
        .orange = 0.8,
        .big_orange = 0.1,
        .unknown = 0.1
      }),
    MakeConeWithColorProbabilityMsg(
      {3.1, 3.4},
      {4.2, 0.0, 0.0, 4.1},
      {
        .blue = 0.0,
        .yellow = 0.0,
        .orange = 0.0,
        .big_orange = 1.0,
        .unknown = 0.0
      }),
    MakeConeWithColorProbabilityMsg(
      {4.1, 4.4},
      {5.2, 0.0, 0.0, 5.1},
      {
        .blue = 0.2,
        .yellow = 0.1,
        .orange = 0.1,
        .big_orange = 0.1,
        .unknown = 0.5
      })
  };
  auto result = FromMsg<ConeWithColorProbability>(MakeConeWithColorProbabilityArrayMsg(cones));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {0.1, 0.4},
              {1.2, 0.0, 0.0, 1.1},
              {
                .blue = 0.6,
                .yellow = 0.2,
                .orange = 0.1,
                .big_orange = 0.0,
                .unknown = 0.1
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {1.1, 1.4},
              {2.2, 0.0, 0.0, 2.1},
              {
                .blue = 0.0,
                .yellow = 0.5,
                .orange = 0.0,
                .big_orange = 0.0,
                .unknown = 0.5
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {2.1, 2.4},
              {3.2, 0.0, 0.0, 3.1},
              {
                .blue = 0.0,
                .yellow = 0.0,
                .orange = 0.8,
                .big_orange = 0.1,
                .unknown = 0.1
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {3.1, 3.4},
              {4.2, 0.0, 0.0, 4.1},
              {
                .blue = 0.0,
                .yellow = 0.0,
                .orange = 0.0,
                .big_orange = 1.0,
                .unknown = 0.0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {4.1, 4.4},
              {5.2, 0.0, 0.0, 5.1},
              {
                .blue = 0.2,
                .yellow = 0.1,
                .orange = 0.1,
                .big_orange = 0.1,
                .unknown = 0.5
              }))
      }));
}

TEST(CheckFromMsg, MultipleColorWithColorProbability) {
  auto cones = {
    MakeConeWithColorProbabilityMsg(
      {0.1, 0.4},
      {1.2, 0.0, 0.0, 1.1},
      {
        .blue = 0.6,
        .yellow = 0.2,
        .orange = 0.1,
        .big_orange = 0.0,
        .unknown = 0.1
      }),
    MakeConeWithColorProbabilityMsg(
      {1.1, 1.4},
      {2.2, 0.0, 0.0, 2.1},
      {
        .blue = 0.5,
        .yellow = 0.0,
        .orange = 0.0,
        .big_orange = 0.0,
        .unknown = 0.5
      }),
    MakeConeWithColorProbabilityMsg(
      {2.1, 2.4},
      {3.2, 0.0, 0.0, 3.1},
      {
        .blue = 0.0,
        .yellow = 0.0,
        .orange = 0.8,
        .big_orange = 0.1,
        .unknown = 0.1
      }),
    MakeConeWithColorProbabilityMsg(
      {3.1, 3.4},
      {4.2, 0.0, 0.0, 4.1},
      {
        .blue = 0.0,
        .yellow = 0.0,
        .orange = 0.0,
        .big_orange = 1.0,
        .unknown = 0.0
      }),
    MakeConeWithColorProbabilityMsg(
      {4.1, 4.4},
      {5.2, 0.0, 0.0, 5.1},
      {
        .blue = 0.2,
        .yellow = 0.1,
        .orange = 0.1,
        .big_orange = 0.1,
        .unknown = 0.5
      })
  };
  auto result = FromMsg<ConeWithColorProbability>(MakeConeWithColorProbabilityArrayMsg(cones));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {0.1, 0.4},
              {1.2, 0.0, 0.0, 1.1},
              {
                .blue = 0.6,
                .yellow = 0.2,
                .orange = 0.1,
                .big_orange = 0.0,
                .unknown = 0.1
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {1.1, 1.4},
              {2.2, 0.0, 0.0, 2.1},
              {
                .blue = 0.5,
                .yellow = 0.0,
                .orange = 0.0,
                .big_orange = 0.0,
                .unknown = 0.5
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {2.1, 2.4},
              {3.2, 0.0, 0.0, 3.1},
              {
                .blue = 0.0,
                .yellow = 0.0,
                .orange = 0.8,
                .big_orange = 0.1,
                .unknown = 0.1
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {3.1, 3.4},
              {4.2, 0.0, 0.0, 4.1},
              {
                .blue = 0.0,
                .yellow = 0.0,
                .orange = 0.0,
                .big_orange = 1.0,
                .unknown = 0.0
              })),
          eufs::testing::ConeWithColorProbabilityEq(
            MakeCone(
              {4.1, 4.4},
              {5.2, 0.0, 0.0, 5.1},
              {
                .blue = 0.2,
                .yellow = 0.1,
                .orange = 0.1,
                .big_orange = 0.1,
                .unknown = 0.5
              }))
      }));
}

// //////////////////////////
// //  Check ToMsg (Cone)  //
// //////////////////////////

TEST(CheckToMsgCone, Check) {
  auto result = ToMsg(MakeCone(
    {0.1, 0.2},
    {0.1, 0.2, 0.2, 0.3},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    }));
  auto exp = MakeConeWithColorProbabilityMsg(
    {0.1, 0.2},
    {0.1, 0.2, 0.2, 0.3},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityMsgEq(exp));
}

// //////////////////////////
// //  Check ToMsg (Cones) //
// //////////////////////////

TEST(CheckToMsgConesNoHeader, None) {
  auto result = ToMsg(std::vector<ConeWithColorProbability>{});
  auto exp = MakeConeWithColorProbabilityArrayMsg({});
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(exp));
}

TEST(CheckToMsgConesNoHeader, OneBlue) {
  auto blue = MakeCone(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto result = ToMsg<ConeWithColorProbability>(std::vector<ConeWithColorProbability>{blue});
  auto exp_cone = MakeConeWithColorProbabilityMsg(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto exp = MakeConeWithColorProbabilityArrayMsg({exp_cone});
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(exp));
}

TEST(CheckToMsgConesNoHeader, OneOfEachColor) {
  auto blue = MakeCone(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto yellow = MakeCone(
    {1.1, 1.2},
    {1.1, 0.0, 0.0, 1.2},
    {
      .blue = .0,
      .yellow = 1.,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto orange = MakeCone(
    {2.1, 2.2},
    {2.1, 0.0, 0.0, 2.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = 1.,
      .big_orange = .0,
      .unknown = .0
    });
  auto unknown = MakeCone(
    {4.1, 4.2},
    {4.1, 0.0, 0.0, 4.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = 1.
    });
  auto result = ToMsg<ConeWithColorProbability>({blue, yellow, orange, unknown});
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(
    MakeConeWithColorProbabilityArrayMsg({
      MakeConeWithColorProbabilityMsg(
        {0.1, 0.2},
        {0.1, 0.0, 0.0, 0.2},
        {
          .blue = 1.,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {1.1, 1.2},
        {1.1, 0.0, 0.0, 1.2},
        {
          .blue = .0,
          .yellow = 1.,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {2.1, 2.2},
        {2.1, 0.0, 0.0, 2.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = 1.,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {4.1, 4.2},
        {4.1, 0.0, 0.0, 4.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = 1.
        })})));
}

TEST(CheckToMsgConesNoHeader, MultipleColor) {
  auto blue_1 = MakeCone(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto blue_2 = MakeCone(
    {1.1, 1.2},
    {1.1, 0.0, 0.0, 1.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto orange = MakeCone(
    {2.1, 2.2},
    {2.1, 0.0, 0.0, 2.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = 1.,
      .big_orange = .0,
      .unknown = .0
    });
  auto unknown = MakeCone(
    {4.1, 4.2},
    {4.1, 0.0, 0.0, 4.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = 1.
    });
  auto result = ToMsg<ConeWithColorProbability>({blue_1, blue_2, orange, unknown});
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(
    MakeConeWithColorProbabilityArrayMsg({
      MakeConeWithColorProbabilityMsg(
        {0.1, 0.2},
        {0.1, 0.0, 0.0, 0.2},
        {
          .blue = 1.,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {1.1, 1.2},
        {1.1, 0.0, 0.0, 1.2},
        {
          .blue = 1.,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {2.1, 2.2},
        {2.1, 0.0, 0.0, 2.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = 1.,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {4.1, 4.2},
        {4.1, 0.0, 0.0, 4.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = 1.
        })})));
}

// ////////////////////////////////////
// //  Check ToMsg (Cones w/ Header) //
// ////////////////////////////////////

TEST(CheckToMsgConesHeader, None) {
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<ConeWithColorProbability>(std::vector<ConeWithColorProbability>{}, header);
  auto exp = MakeConeWithColorProbabilityArrayMsg({}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(exp));
}

TEST(CheckToMsgConesHeader, OneBlue) {
  auto blue = MakeCone(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<ConeWithColorProbability>({blue}, header);
  auto exp_cone = MakeConeWithColorProbabilityMsg(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto exp = MakeConeWithColorProbabilityArrayMsg({exp_cone}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(exp));
}

TEST(CheckToMsgConesHeader, OneOfEachColor) {
  auto blue = MakeCone(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto yellow = MakeCone(
    {1.1, 1.2},
    {1.1, 0.0, 0.0, 1.2},
    {
      .blue = .0,
      .yellow = 1.,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto orange = MakeCone(
    {2.1, 2.2},
    {2.1, 0.0, 0.0, 2.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = 1.,
      .big_orange = .0,
      .unknown = .0
    });
  auto unknown = MakeCone(
    {4.1, 4.2},
    {4.1, 0.0, 0.0, 4.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = 1.
    });
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<ConeWithColorProbability>({blue, yellow, orange, unknown}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(
    MakeConeWithColorProbabilityArrayMsg({
      MakeConeWithColorProbabilityMsg(
        {0.1, 0.2},
        {0.1, 0.0, 0.0, 0.2},
        {
          .blue = 1.,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {1.1, 1.2},
        {1.1, 0.0, 0.0, 1.2},
        {
          .blue = .0,
          .yellow = 1.,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {2.1, 2.2},
        {2.1, 0.0, 0.0, 2.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = 1.,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {4.1, 4.2},
        {4.1, 0.0, 0.0, 4.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = 1.
        })}, header)));
}

TEST(CheckToMsgConesHeader, MultipleColor) {
  auto blue_1 = MakeCone(
    {0.1, 0.2},
    {0.1, 0.0, 0.0, 0.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto blue_2 = MakeCone(
    {1.1, 1.2},
    {1.1, 0.0, 0.0, 1.2},
    {
      .blue = 1.,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = .0
    });
  auto orange = MakeCone(
    {2.1, 2.2},
    {2.1, 0.0, 0.0, 2.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = 1.,
      .big_orange = .0,
      .unknown = .0
    });
  auto unknown = MakeCone(
    {4.1, 4.2},
    {4.1, 0.0, 0.0, 4.2},
    {
      .blue = .0,
      .yellow = .0,
      .orange = .0,
      .big_orange = .0,
      .unknown = 1.
    });
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<ConeWithColorProbability>({blue_1, blue_2, orange, unknown}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithColorProbabilityArrayMsgEq(
    MakeConeWithColorProbabilityArrayMsg({
      MakeConeWithColorProbabilityMsg(
        {0.1, 0.2},
        {0.1, 0.0, 0.0, 0.2},
        {
          .blue = 1.,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {1.1, 1.2},
        {1.1, 0.0, 0.0, 1.2},
        {
          .blue = 1.,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {2.1, 2.2},
        {2.1, 0.0, 0.0, 2.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = 1.,
          .big_orange = .0,
          .unknown = .0
        }),
      MakeConeWithColorProbabilityMsg(
        {4.1, 4.2},
        {4.1, 0.0, 0.0, 4.2},
        {
          .blue = .0,
          .yellow = .0,
          .orange = .0,
          .big_orange = .0,
          .unknown = 1.
        })}, header)));
}
