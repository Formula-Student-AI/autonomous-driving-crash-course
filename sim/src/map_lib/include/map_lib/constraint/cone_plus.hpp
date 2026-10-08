#ifndef INCLUDE_MAP_LIB_CONSTRAINT_CONE_PLUS_HPP_
#define INCLUDE_MAP_LIB_CONSTRAINT_CONE_PLUS_HPP_

#include "map_lib/constraint/cone.hpp"

namespace eufs::map::constraint {

template<typename T>
concept ConePlus = Cone<T> &&
    requires(T a, double discount_factor) {
        a.GetColorProbabilities(discount_factor);
        a.GetId();
    };// NOLINT

}

#endif  // INCLUDE_MAP_LIB_CONSTRAINT_CONE_PLUS_HPP_
