#ifndef INCLUDE_EUFS_GMOCK_MATCHERS_STD_MATCHERS_HPP_
#define INCLUDE_EUFS_GMOCK_MATCHERS_STD_MATCHERS_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>

namespace eufs::testing::matchers {

inline auto StampMsgEq(const builtin_interfaces::msg::Time &exp) {
  return ::testing::AllOf(
      ::testing::Field("sec", &builtin_interfaces::msg::Time::sec, ::testing::Eq(exp.sec)),
      ::testing::Field("nanosec", &builtin_interfaces::msg::Time::nanosec,
                       ::testing::Eq(exp.nanosec)));
}

inline auto HeaderMsgEq(const std_msgs::msg::Header &exp) {
  return ::testing::AllOf(
      ::testing::Field("frame_id", &std_msgs::msg::Header::frame_id,
                       ::testing::StrEq(exp.frame_id)),
      ::testing::Field("stamp", &std_msgs::msg::Header::stamp, StampMsgEq(exp.stamp)));
}

}  // namespace eufs::testing::matchers

#endif  // INCLUDE_EUFS_GMOCK_MATCHERS_STD_MATCHERS_HPP_
