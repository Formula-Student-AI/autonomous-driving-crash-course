#ifndef INCLUDE_EUFS_GMOCK_MATCHERS_EUFS_MATCHERS_HPP_
#define INCLUDE_EUFS_GMOCK_MATCHERS_EUFS_MATCHERS_HPP_

#include <gmock/gmock.h>

#include <vector>
#include <iterator>
#include <algorithm>

#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_color_probability.hpp>
#include <eufs_msgs/msg/cone_with_color_probability_array.hpp>

#include "eufs_gmock_matchers/geometry_matchers.hpp"
#include "eufs_gmock_matchers/std_matchers.hpp"

namespace eufs::testing::matchers {

inline auto ConeWithCovarianceMsgEq(const eufs_msgs::msg::ConeWithCovariance &exp) {
  return ::testing::AllOf(
      ::testing::Field("point", &eufs_msgs::msg::ConeWithCovariance::point, PointMsgEq(exp.point)),
      ::testing::Field("covariance", &eufs_msgs::msg::ConeWithCovariance::covariance,
                       ::testing::ElementsAreArray(exp.covariance)));
}

inline auto ConeWithCovarianceMsgEq(const std::vector<eufs_msgs::msg::ConeWithCovariance> &exp) {
  std::vector<::testing::Matcher<eufs_msgs::msg::ConeWithCovariance>> exp_matchers;
  std::transform(exp.cbegin(), exp.cend(), std::back_inserter(exp_matchers),
                 [](const auto &c) { return ConeWithCovarianceMsgEq(c); });
  return ::testing::ElementsAreArray(exp_matchers);
}

inline auto ConeArrayWithCovarianceMsgEq(const eufs_msgs::msg::ConeArrayWithCovariance &exp) {
  return ::testing::AllOf(
      ::testing::Field("header", &eufs_msgs::msg::ConeArrayWithCovariance::header,
                       HeaderMsgEq(exp.header)),
      ::testing::Field("blue_cones", &eufs_msgs::msg::ConeArrayWithCovariance::blue_cones,
                       ConeWithCovarianceMsgEq(exp.blue_cones)),
      ::testing::Field("yellow_cones", &eufs_msgs::msg::ConeArrayWithCovariance::yellow_cones,
                       ConeWithCovarianceMsgEq(exp.yellow_cones)),
      ::testing::Field("orange_cones", &eufs_msgs::msg::ConeArrayWithCovariance::orange_cones,
                       ConeWithCovarianceMsgEq(exp.orange_cones)),
      ::testing::Field("big_orange_cones",
                       &eufs_msgs::msg::ConeArrayWithCovariance::big_orange_cones,
                       ConeWithCovarianceMsgEq(exp.big_orange_cones)),
      ::testing::Field("unknown_color_cones",
                       &eufs_msgs::msg::ConeArrayWithCovariance::unknown_color_cones,
                       ConeWithCovarianceMsgEq(exp.unknown_color_cones)));
}

inline auto ConeWithColorProbabilityMsgEq(const eufs_msgs::msg::ConeWithColorProbability &exp) {
  return ::testing::AllOf(
      ::testing::Field("id", &eufs_msgs::msg::ConeWithColorProbability::id,
        ::testing::Eq(exp.id)),
      ::testing::Field("blue_prob", &eufs_msgs::msg::ConeWithColorProbability::blue_prob,
        ::testing::DoubleEq(exp.blue_prob)),
      ::testing::Field("yellow_prob", &eufs_msgs::msg::ConeWithColorProbability::yellow_prob,
        ::testing::DoubleEq(exp.yellow_prob)),
      ::testing::Field("orange_prob",
        &eufs_msgs::msg::ConeWithColorProbability::orange_prob,
        ::testing::DoubleEq(exp.orange_prob)),
      ::testing::Field("big_orange_prob",
        &eufs_msgs::msg::ConeWithColorProbability::big_orange_prob,
        ::testing::DoubleEq(exp.big_orange_prob)),
      ::testing::Field("unknown_prob",
        &eufs_msgs::msg::ConeWithColorProbability::unknown_prob,
        ::testing::DoubleEq(exp.unknown_prob)),
      ::testing::Field("point",
        &eufs_msgs::msg::ConeWithColorProbability::point, PointMsgEq(exp.point)),
      ::testing::Field("covariance",
        &eufs_msgs::msg::ConeWithColorProbability::covariance,
                       ::testing::ElementsAreArray(exp.covariance)));
}

inline auto ConeWithColorProbabilityMsgEq(
  const std::vector<eufs_msgs::msg::ConeWithColorProbability> &exp) {
  std::vector<::testing::Matcher<eufs_msgs::msg::ConeWithColorProbability>> exp_matchers;
  std::transform(exp.cbegin(), exp.cend(), std::back_inserter(exp_matchers),
                 [](const auto &c) { return ConeWithColorProbabilityMsgEq(c); });
  return ::testing::ElementsAreArray(exp_matchers);
}

inline auto ConeWithColorProbabilityArrayMsgEq(
  const eufs_msgs::msg::ConeWithColorProbabilityArray &exp) {
  return ::testing::AllOf(
      ::testing::Field("header", &eufs_msgs::msg::ConeWithColorProbabilityArray::header,
                       HeaderMsgEq(exp.header)),
      ::testing::Field("cones", &eufs_msgs::msg::ConeWithColorProbabilityArray::cones,
                       ConeWithColorProbabilityMsgEq(exp.cones)));
}

}  // namespace eufs::testing::matchers

#endif  // INCLUDE_EUFS_GMOCK_MATCHERS_EUFS_MATCHERS_HPP_
