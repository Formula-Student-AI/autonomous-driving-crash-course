#ifndef INCLUDE_MAP_LIB_CONSTRAINT_IN_FOV_CHECKER_HPP_
#define INCLUDE_MAP_LIB_CONSTRAINT_IN_FOV_CHECKER_HPP_

#include <concepts>

#include "map_lib/constraint/cone.hpp"

namespace eufs::map::constraint {

template<typename T, typename ConeType>
concept InFOVChecker = Cone<ConeType> &&
    requires(T checker, ConeType cone) {
        { checker(cone) } -> std::convertible_to<bool>;
    };// NOLINT

}

#endif  // INCLUDE_MAP_LIB_CONSTRAINT_IN_FOV_CHECKER_HPP_
