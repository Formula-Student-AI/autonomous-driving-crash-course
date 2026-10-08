#ifndef INCLUDE_EUFS_GMOCK_MATCHERS_NAV_MATCHERS_HPP_
#define INCLUDE_EUFS_GMOCK_MATCHERS_NAV_MATCHERS_HPP_

#include <vector>
#include <algorithm>
#include <nav_msgs/msg/odometry.hpp>

#include "eufs_gmock_matchers/std_matchers.hpp"
#include "eufs_gmock_matchers/geometry_matchers.hpp"

namespace eufs::testing::matchers {

inline auto OdomMsgEq(const nav_msgs::msg::Odometry &exp) {
  using OdometryMsg = nav_msgs::msg::Odometry;
  return ::testing::AllOf(
      ::testing::Field("header", &OdometryMsg::header, HeaderMsgEq(exp.header)),
      ::testing::Field("pose", &OdometryMsg::pose, PoseWithCovarianceMsgEq(exp.pose)),
      ::testing::Field("twist", &OdometryMsg::twist, TwistWithCovarianceMsgEq(exp.twist)));
}

inline auto OdomMsgEq(const std::vector<nav_msgs::msg::Odometry> &exp) {
  std::vector<::testing::Matcher<nav_msgs::msg::Odometry>> exp_matchers;
  std::transform(exp.cbegin(), exp.cend(), std::back_inserter(exp_matchers),
                 [](const auto &c) { return OdomMsgEq(c); });
  return ::testing::ElementsAreArray(exp_matchers);
}

}  // namespace eufs::testing::matchers

#endif  // INCLUDE_EUFS_GMOCK_MATCHERS_NAV_MATCHERS_HPP_
