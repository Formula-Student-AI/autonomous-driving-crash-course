#ifndef INCLUDE_MAP_LIB_CONSTRAINT_CONE_WITH_COLOR_PROBABILITY_HPP_
#define INCLUDE_MAP_LIB_CONSTRAINT_CONE_WITH_COLOR_PROBABILITY_HPP_

#include "map_lib/constraint/cone.hpp"

namespace eufs::map::constraint {

template<typename T>
concept ConeWithColorProbability = Cone<T> &&
    requires(T a) {
        a.GetColorProbabilities();
    };// NOLINT

}

#endif  // INCLUDE_MAP_LIB_CONSTRAINT_CONE_WITH_COLOR_PROBABILITY_HPP_
