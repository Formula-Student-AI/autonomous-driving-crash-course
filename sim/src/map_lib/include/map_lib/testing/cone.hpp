#ifndef INCLUDE_MAP_LIB_TESTING_CONE_HPP_
#define INCLUDE_MAP_LIB_TESTING_CONE_HPP_

#include <gmock/gmock.h>

#include <eufs_gmock_matchers/eigen_matchers.hpp>

#include "map_lib/type/cone.hpp"

namespace eufs::testing {

inline auto ConeEq(const eufs::map::Cone &exp) {
  return ::testing::AllOf(
      ::testing::Field("position", &eufs::map::Cone::position,
                       eufs::testing::matchers::EigenEq(exp.position)),
      ::testing::Field("covariance", &eufs::map::Cone::covariance,
                       eufs::testing::matchers::EigenEq(exp.covariance)),
      ::testing::Property(
          "GetColor",
          static_cast<eufs::map::Color (eufs::map::Cone::*)() const>(&eufs::map::Cone::GetColor),
          ::testing::Eq(exp.GetColor())),
      ::testing::Property("id", &eufs::map::Cone::GetId, ::testing::Eq(exp.GetId())));
}

}  // namespace eufs::testing

#endif  // INCLUDE_MAP_LIB_TESTING_CONE_HPP_
