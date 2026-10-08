#include <eigen3/Eigen/Dense>
#include <gmock/gmock.h>

#include <array>
#include <rclcpp/time.hpp>
#include <eufs_gmock_matchers/eufs_matchers.hpp>
#include <eufs_gmock_matchers/eigen_matchers.hpp>

#include "map_lib/testing/cone.hpp"
#include "map_lib/type/cone.hpp"
#include "map_lib/type/color_probabilities.hpp"

using eufs::map::Color;
using eufs::map::Cone;
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

Cone MakeCone(const std::array<double, 2> &pos, const std::array<double, 4> &cov,
              const Color &color = Color::UNKNOWN) {
  return {pos, cov, color};
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

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(Cone()); }

TEST(CheckConstructor, EigenPositionNoColor) {
  auto cone = Cone(Eigen::Vector2d{1.0, 9.0});
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColor(), Color::UNKNOWN);
}

TEST(CheckConstructor, EigenPositionColor) {
  auto cone = Cone(Eigen::Vector2d{1.0, 9.0}, Color::YELLOW);
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColor(), Color::YELLOW);
}

TEST(CheckConstructor, ArrayPositionNoColor) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0});
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColor(), Color::UNKNOWN);
}

TEST(CheckConstructor, ArrayPositionColor) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, Color::YELLOW);
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.GetColor(), Color::YELLOW);
}

TEST(CheckConstructor, EigenPositionCovarianceNoColor) {
  auto cone = Cone(Eigen::Vector2d{1.0, 9.0}, MakeCovariance({0.1, 0.2, 0.3, 0.4}));
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColor(), Color::UNKNOWN);
}

TEST(CheckConstructor, EigenPositionCovarianceColor) {
  auto cone =
      Cone(Eigen::Vector2d{1.0, 9.0}, MakeCovariance({0.1, 0.2, 0.3, 0.4}), Color::YELLOW);
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColor(), Color::YELLOW);
}

TEST(CheckConstructor, ArrayPositionCovarianceNoColor) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4});
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColor(), Color::UNKNOWN);
}

TEST(CheckConstructor, ArrayPositionCovarianceColor) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::YELLOW);
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{1.0, 9.0}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColor(), Color::YELLOW);
}

TEST(CheckConstructor, MsgNoColor) {
  auto msg = MakeConeMsg(std::array<double, 2>{0.1, 0.2}, {0.1, 0.2, 0.3, 0.4});
  auto cone = Cone(msg);
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{0.1, 0.2}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColor(), Color::UNKNOWN);
}

TEST(CheckConstructor, MsgColor) {
  auto msg = MakeConeMsg({0.1, 0.2}, {0.1, 0.2, 0.3, 0.4});
  auto cone = Cone(msg, Color::YELLOW);
  ASSERT_THAT(cone.position, eufs::testing::matchers::EigenEq(Eigen::Vector2d{0.1, 0.2}));
  ASSERT_THAT(cone.covariance,
              eufs::testing::matchers::EigenEq(MakeCovariance({0.1, 0.2, 0.3, 0.4})));
  ASSERT_THAT(cone.GetColor(), Color::YELLOW);
}

///////////////////
// Check ToColor //
///////////////////

TEST(CheckToColor, Blue) { ASSERT_EQ(Cone::ToColor("blue"), Color::BLUE); }

TEST(CheckToColor, Yellow) { ASSERT_EQ(Cone::ToColor("yellow"), Color::YELLOW); }

TEST(CheckToColor, Orange) { ASSERT_EQ(Cone::ToColor("orange"), Color::ORANGE); }

TEST(CheckToColor, BigOrange) { ASSERT_EQ(Cone::ToColor("big_orange"), Color::ORANGE); }

TEST(CheckToColor, Unknown) { ASSERT_EQ(Cone::ToColor("unknown"), Color::UNKNOWN); }

TEST(CheckToColor, Random) { ASSERT_EQ(Cone::ToColor("ros"), Color::UNKNOWN); }

////////////////////
// Check GetColor //
////////////////////

TEST(CheckGetColorConst, Blue) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::BLUE);
  ASSERT_EQ(cone.GetColor(), Color::BLUE);
}

TEST(CheckGetColorConst, Yellow) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::YELLOW);
  ASSERT_EQ(cone.GetColor(), Color::YELLOW);
}

TEST(CheckGetColorConst, Orange) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::ORANGE);
  ASSERT_EQ(cone.GetColor(), Color::ORANGE);
}

TEST(CheckGetColorConst, Unknown) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::UNKNOWN);
  ASSERT_EQ(cone.GetColor(), Color::UNKNOWN);
}

TEST(CheckGetColor, Blue) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::BLUE);
  cone.GetColor() = Color::YELLOW;
  ASSERT_EQ(cone.GetColor(), Color::YELLOW);
}

TEST(CheckGetColor, Yellow) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::YELLOW);
  cone.GetColor() = Color::BLUE;
  ASSERT_EQ(cone.GetColor(), Color::BLUE);
}

TEST(CheckGetColor, Orange) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::ORANGE);
  cone.GetColor() = Color::UNKNOWN;
  ASSERT_EQ(cone.GetColor(), Color::UNKNOWN);
}

TEST(CheckGetColor, Unknown) {
  auto cone = Cone(std::array<double, 2>{1.0, 9.0}, {0.1, 0.2, 0.3, 0.4}, Color::UNKNOWN);
  cone.GetColor() = Color::ORANGE;
  ASSERT_EQ(cone.GetColor(), Color::ORANGE);
}


/////////////////////
//  Check FromMsg  //
/////////////////////

TEST(CheckFromMsg, None) {
  auto result = FromMsg<Cone>(MakeConeArrayMsg({}, {}, {}, {}, {}));
  EXPECT_EQ(result.size(), 0lu);
}

TEST(CheckFromMsg, One) {
  auto blues = {MakeConeMsg({0.1, 0.4}, {0.2, 0.0, 0.0, 0.1})};
  auto result = FromMsg<Cone>(MakeConeArrayMsg(blues, {}, {}, {}, {}));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeEq(MakeCone({0.1, 0.4}, {0.2, 0.0, 0.0, 0.1}, Color::BLUE)),
      }));
}

TEST(CheckFromMsg, OneOfEachColor) {
  auto blues = {MakeConeMsg({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1})};
  auto yellows = {MakeConeMsg({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1})};
  auto oranges = {MakeConeMsg({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1})};
  auto big_oranges = {MakeConeMsg({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1})};
  auto unknowns = {MakeConeMsg({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1})};
  auto result = FromMsg<Cone>(MakeConeArrayMsg(blues, yellows, oranges, big_oranges, unknowns));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeEq(MakeCone({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1}, Color::BLUE)),
          eufs::testing::ConeEq(MakeCone({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1}, Color::YELLOW)),
          eufs::testing::ConeEq(MakeCone({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1}, Color::UNKNOWN)),
      }));
}

TEST(CheckFromMsg, MultipleColor) {
  auto blues = {MakeConeMsg({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1}),
                MakeConeMsg({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1})};
  auto oranges = {MakeConeMsg({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1})};
  auto big_oranges = {MakeConeMsg({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1})};
  auto unknowns = {MakeConeMsg({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1})};
  auto result = FromMsg<Cone>(MakeConeArrayMsg(blues, {}, oranges, big_oranges, unknowns));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeEq(MakeCone({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1}, Color::BLUE)),
          eufs::testing::ConeEq(MakeCone({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1}, Color::BLUE)),
          eufs::testing::ConeEq(MakeCone({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1}, Color::UNKNOWN)),
      }));
}

TEST(CheckFromMsg, NoneWithColorProbability) {
  auto result = FromMsg<Cone>(MakeConeWithColorProbabilityArrayMsg({}));
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
  auto result = FromMsg<Cone>(MakeConeWithColorProbabilityArrayMsg(cones));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeEq(MakeCone({0.1, 0.4}, {0.2, 0.0, 0.0, 0.1}, Color::BLUE)),
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
  auto result = FromMsg<Cone>(MakeConeWithColorProbabilityArrayMsg(cones));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeEq(MakeCone({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1}, Color::BLUE)),
          eufs::testing::ConeEq(MakeCone({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1}, Color::YELLOW)),
          eufs::testing::ConeEq(MakeCone({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1}, Color::UNKNOWN)),
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
  auto result = FromMsg<Cone>(MakeConeWithColorProbabilityArrayMsg(cones));
  EXPECT_THAT(
      result,
      ::testing::UnorderedElementsAreArray({
          eufs::testing::ConeEq(MakeCone({0.1, 0.4}, {1.2, 0.0, 0.0, 1.1}, Color::BLUE)),
          eufs::testing::ConeEq(MakeCone({1.1, 1.4}, {2.2, 0.0, 0.0, 2.1}, Color::BLUE)),
          eufs::testing::ConeEq(MakeCone({2.1, 2.4}, {3.2, 0.0, 0.0, 3.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({3.1, 3.4}, {4.2, 0.0, 0.0, 4.1}, Color::ORANGE)),
          eufs::testing::ConeEq(MakeCone({4.1, 4.4}, {5.2, 0.0, 0.0, 5.1}, Color::UNKNOWN)),
      }));
}

//////////////////////////
//  Check ToMsg (Cone)  //
//////////////////////////

TEST(CheckToMsgCone, Check) {
  auto result = ToMsg(MakeCone({0.1, 0.2}, {0.1, 0.2, 0.2, 0.3}, Color::BLUE));
  auto exp = MakeConeMsg({0.1, 0.2}, {0.1, 0.2, 0.2, 0.3});
  EXPECT_THAT(result, eufs::testing::matchers::ConeWithCovarianceMsgEq(exp));
}

//////////////////////////
//  Check ToMsg (Cones) //
//////////////////////////

TEST(CheckToMsgConesNoHeader, None) {
  auto result = ToMsg(std::vector<Cone>{});
  auto exp = MakeConeArrayMsg({}, {}, {}, {}, {});
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(exp));
}

TEST(CheckToMsgConesNoHeader, OneBlue) {
  auto blue = MakeCone({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}, Color::BLUE);
  auto result = ToMsg<Cone>(std::vector<Cone>{blue});
  auto exp_cone = MakeConeMsg({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2});
  auto exp = MakeConeArrayMsg({exp_cone}, {}, {}, {}, {});
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(exp));
}

TEST(CheckToMsgConesNoHeader, OneOfEachColor) {
  auto blue = MakeCone({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}, Color::BLUE);
  auto yellow = MakeCone({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2}, Color::YELLOW);
  auto orange = MakeCone({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2}, Color::ORANGE);
  auto unknown = MakeCone({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2}, Color::UNKNOWN);
  auto result = ToMsg<Cone>({blue, yellow, orange, unknown});
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(MakeConeArrayMsg(
                          {MakeConeMsg({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2})},
                          {MakeConeMsg({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2})},
                          {MakeConeMsg({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2})}, {},
                          {MakeConeMsg({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2})})));
}

TEST(CheckToMsgConesNoHeader, MultipleColor) {
  auto blue_1 = MakeCone({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}, Color::BLUE);
  auto blue_2 = MakeCone({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2}, Color::BLUE);
  auto orange = MakeCone({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2}, Color::ORANGE);
  auto unknown = MakeCone({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2}, Color::UNKNOWN);
  auto result = ToMsg<Cone>({blue_1, blue_2, orange, unknown});
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(MakeConeArrayMsg(
                          {MakeConeMsg({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}),
                           MakeConeMsg({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2})},
                          {}, {MakeConeMsg({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2})}, {},
                          {MakeConeMsg({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2})})));
}

////////////////////////////////////
//  Check ToMsg (Cones w/ Header) //
////////////////////////////////////

TEST(CheckToMsgConesHeader, None) {
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<Cone>(std::vector<Cone>{}, header);
  auto exp = MakeConeArrayMsg({}, {}, {}, {}, {}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(exp));
}

TEST(CheckToMsgConesHeader, OneBlue) {
  auto blue = MakeCone({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}, Color::BLUE);
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<Cone>({blue}, header);
  auto exp_cone = MakeConeMsg({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2});
  auto exp = MakeConeArrayMsg({exp_cone}, {}, {}, {}, {}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(exp));
}

TEST(CheckToMsgConesHeader, OneOfEachColor) {
  auto blue = MakeCone({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}, Color::BLUE);
  auto yellow = MakeCone({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2}, Color::YELLOW);
  auto orange = MakeCone({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2}, Color::ORANGE);
  auto unknown = MakeCone({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2}, Color::UNKNOWN);
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<Cone>({blue, yellow, orange, unknown}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(MakeConeArrayMsg(
                          {MakeConeMsg({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2})},
                          {MakeConeMsg({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2})},
                          {MakeConeMsg({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2})}, {},
                          {MakeConeMsg({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2})}, header)));
}

TEST(CheckToMsgConesHeader, MultipleColor) {
  auto blue_1 = MakeCone({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}, Color::BLUE);
  auto blue_2 = MakeCone({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2}, Color::BLUE);
  auto orange = MakeCone({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2}, Color::ORANGE);
  auto unknown = MakeCone({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2}, Color::UNKNOWN);
  auto header = MakeHeader(rclcpp::Time(1.0), "test");
  auto result = ToMsg<Cone>({blue_1, blue_2, orange, unknown}, header);
  EXPECT_THAT(result, eufs::testing::matchers::ConeArrayWithCovarianceMsgEq(MakeConeArrayMsg(
                          {MakeConeMsg({0.1, 0.2}, {0.1, 0.0, 0.0, 0.2}),
                           MakeConeMsg({1.1, 1.2}, {1.1, 0.0, 0.0, 1.2})},
                          {}, {MakeConeMsg({2.1, 2.2}, {2.1, 0.0, 0.0, 2.2})}, {},
                          {MakeConeMsg({4.1, 4.2}, {4.1, 0.0, 0.0, 4.2})}, header)));
}
