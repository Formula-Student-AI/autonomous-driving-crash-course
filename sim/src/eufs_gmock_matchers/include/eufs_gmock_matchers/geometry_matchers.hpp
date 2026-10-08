#ifndef INCLUDE_EUFS_GMOCK_MATCHERS_GEOMETRY_MATCHERS_HPP_
#define INCLUDE_EUFS_GMOCK_MATCHERS_GEOMETRY_MATCHERS_HPP_

#include <vector>
#include <algorithm>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include "eufs_gmock_matchers/std_matchers.hpp"

namespace eufs::testing::matchers {

auto QuaternionMsgEq(const geometry_msgs::msg::Quaternion &exp) {
  using QuaternionMsg = geometry_msgs::msg::Quaternion;
  return ::testing::AllOf(::testing::Field("w", &QuaternionMsg::w, ::testing::DoubleEq(exp.w)),
                          ::testing::Field("x", &QuaternionMsg::x, ::testing::DoubleEq(exp.x)),
                          ::testing::Field("y", &QuaternionMsg::y, ::testing::DoubleEq(exp.y)),
                          ::testing::Field("z", &QuaternionMsg::z, ::testing::DoubleEq(exp.z)));
}

auto PointMsgEq(const geometry_msgs::msg::Point &exp) {
  using PointMsg = geometry_msgs::msg::Point;
  return ::testing::AllOf(::testing::Field("x", &PointMsg::x, ::testing::DoubleEq(exp.x)),
                          ::testing::Field("y", &PointMsg::y, ::testing::DoubleEq(exp.y)),
                          ::testing::Field("z", &PointMsg::z, ::testing::DoubleEq(exp.z)));
}

auto PointStampedMsgEq(const geometry_msgs::msg::PointStamped &exp) {
  using PointStampedMsg = geometry_msgs::msg::PointStamped;
  return ::testing::AllOf(
      ::testing::Field("header", &PointStampedMsg::header, HeaderMsgEq(exp.header)),
      ::testing::Field("point", &PointStampedMsg::point, PointMsgEq(exp.point)));
}

auto PoseMsgEq(const geometry_msgs::msg::Pose &exp) {
  using PoseMsg = geometry_msgs::msg::Pose;
  return ::testing::AllOf(
      ::testing::Field("position", &PoseMsg::position, PointMsgEq(exp.position)),
      ::testing::Field("orientation", &PoseMsg::orientation, QuaternionMsgEq(exp.orientation)));
}

auto PoseWithCovarianceMsgEq(const geometry_msgs::msg::PoseWithCovariance &exp) {
  using PoseWithCovarianceMsg = geometry_msgs::msg::PoseWithCovariance;
  return ::testing::AllOf(
    ::testing::Field("pose", &PoseWithCovarianceMsg::pose, PoseMsgEq(exp.pose)),
    ::testing::Field("covariance", &PoseWithCovarianceMsg::covariance,
                     ::testing::ElementsAreArray(exp.covariance)));
}

auto Vector3MsgEq(const geometry_msgs::msg::Vector3 &exp) {
  using Vector3Msg = geometry_msgs::msg::Vector3;
  return ::testing::AllOf(::testing::Field("x", &Vector3Msg::x, ::testing::DoubleEq(exp.x)),
                          ::testing::Field("y", &Vector3Msg::y, ::testing::DoubleEq(exp.y)),
                          ::testing::Field("z", &Vector3Msg::z, ::testing::DoubleEq(exp.z)));
}

auto TransformMsgEq(const geometry_msgs::msg::Transform &exp) {
  using Transform = geometry_msgs::msg::Transform;
  return ::testing::AllOf(
      ::testing::Field("translation", &Transform::translation, Vector3MsgEq(exp.translation)),
      ::testing::Field("rotation", &Transform::rotation, QuaternionMsgEq(exp.rotation)));
}


auto TwistMsgEq(const geometry_msgs::msg::Twist &exp) {
  using TwistMsg = geometry_msgs::msg::Twist;
  return ::testing::AllOf(
    ::testing::Field("linear", &TwistMsg::linear, Vector3MsgEq(exp.linear)),
    ::testing::Field("angular", &TwistMsg::angular, Vector3MsgEq(exp.angular)));
}

auto TwistWithCovarianceMsgEq(const geometry_msgs::msg::TwistWithCovariance &exp) {
  using TwistWithCovarianceMsg = geometry_msgs::msg::TwistWithCovariance;
  return ::testing::AllOf(
      ::testing::Field("twist", &TwistWithCovarianceMsg::twist, TwistMsgEq(exp.twist)),
      ::testing::Field("covariance", &TwistWithCovarianceMsg::covariance,
                       ::testing::ElementsAreArray(exp.covariance)));
}

///////////////////////////////////////
//  Matchers for arrays of messages  //
///////////////////////////////////////

auto PointStampedMsgEq(const std::vector<geometry_msgs::msg::PointStamped> &exp) {
  std::vector<::testing::Matcher<geometry_msgs::msg::PointStamped>> exp_matchers;
  std::transform(exp.cbegin(), exp.cend(), std::back_inserter(exp_matchers),
                 [](const auto &c) { return PointStampedMsgEq(c); });
  return ::testing::ElementsAreArray(exp_matchers);
}

}  // namespace eufs::testing::matchers

#endif  // INCLUDE_EUFS_GMOCK_MATCHERS_GEOMETRY_MATCHERS_HPP_
