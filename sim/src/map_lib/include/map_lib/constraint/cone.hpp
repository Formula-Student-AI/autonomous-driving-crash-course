#ifndef INCLUDE_MAP_LIB_CONSTRAINT_CONE_HPP_
#define INCLUDE_MAP_LIB_CONSTRAINT_CONE_HPP_

namespace eufs::map::constraint {

template<typename T>
concept Cone =
    requires(T a) {
        a.position;
        a.covariance;
    };// NOLINT

}

#endif  // INCLUDE_MAP_LIB_CONSTRAINT_CONE_HPP_
