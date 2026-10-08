#ifndef INCLUDE_STATE_LIB_STATE_VARS_HPP_
#define INCLUDE_STATE_LIB_STATE_VARS_HPP_

#include <cstddef>
#include <type_traits>

namespace eufs::state {

using StateElement = size_t;

template <StateElement... elems>
struct StateVars {};

template <StateElement... e1, StateElement... e2>
constexpr StateVars<e1..., e2...> operator&(StateVars<e1...>, StateVars<e2...>) {
  return {};
}

template <class T>
struct IsVars : std::false_type {};

template <StateElement... T>
struct IsVars<StateVars<T...>> : std::true_type {};

template <StateElement... T>
struct IsVars<const StateVars<T...>> : std::true_type {};

template <class T>
struct IsSingleVariable : std::false_type {};

template <StateElement T>
struct IsSingleVariable<StateVars<T>> : std::true_type {};

template <StateElement T>
struct IsSingleVariable<const StateVars<T>> : std::true_type {};

template <StateElement... elems>
constexpr auto GetVarsNumber(const StateVars<elems...> &) {
  return sizeof...(elems);
}

}  // namespace eufs::state

#endif  // INCLUDE_STATE_LIB_STATE_VARS_HPP_
