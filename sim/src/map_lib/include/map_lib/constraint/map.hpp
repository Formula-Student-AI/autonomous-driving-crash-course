#ifndef INCLUDE_MAP_LIB_CONSTRAINT_MAP_HPP_
#define INCLUDE_MAP_LIB_CONSTRAINT_MAP_HPP_

#include "map_lib/constraint/cone.hpp"

namespace eufs::map::constraint {

template<typename T>
concept Map = Cone<typename T::value_type>;
}

#endif  // INCLUDE_MAP_LIB_CONSTRAINT_MAP_HPP_
