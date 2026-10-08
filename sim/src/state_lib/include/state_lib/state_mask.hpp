#ifndef INCLUDE_STATE_LIB_STATE_MASK_HPP_
#define INCLUDE_STATE_LIB_STATE_MASK_HPP_

#include <Eigen/Dense>
#include <array>
#include <type_traits>
#include <unordered_map>

#include "state_lib/state_vars.hpp"

namespace eufs::state {

template <StateElement... vars>
struct StateMask {
  constexpr StateMask() {}
  constexpr explicit StateMask(StateVars<vars...>) {}
};

template <template <StateElement...> class VarCollection1,
          template <StateElement...> class VarCollection2, StateElement... e1, StateElement... e2>
constexpr StateMask<e1..., e2...> operator,(VarCollection1<e1...>, VarCollection2<e2...>) {
  return {};
}

template <class T>
struct IsStateMask : std::false_type {};

template <StateElement... T>
struct IsStateMask<StateMask<T...>> : std::true_type {};

template <StateElement... elems>
constexpr auto GetMaskSize(const StateMask<elems...> &) {
  return sizeof...(elems);
}

}  // namespace eufs::state

#endif  // INCLUDE_STATE_LIB_STATE_MASK_HPP_
