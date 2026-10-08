#ifndef INCLUDE_MAP_LIB_CONSTRAINT_CONES_MESSAGE_CONVERTIBLE_TO_HPP_
#define INCLUDE_MAP_LIB_CONSTRAINT_CONES_MESSAGE_CONVERTIBLE_TO_HPP_

#include "map_lib/type/cone.hpp"
#include "map_lib/type/map.hpp"

namespace eufs::map::constraint {

template <typename T, typename ConeType>
concept ConesMessageConvertibleTo =
    Cone<ConeType> && requires(T a) { FromMsg<ConeType>(a); };  // NOLINT

}  // namespace eufs::map::constraint

#endif  // INCLUDE_MAP_LIB_CONSTRAINT_CONES_MESSAGE_CONVERTIBLE_TO_HPP_
