#ifndef INCLUDE_STATE_LIB_CONSTRAINT_SINGLE_VARIABLE_HPP_
#define INCLUDE_STATE_LIB_CONSTRAINT_SINGLE_VARIABLE_HPP_

#include "state_lib/state_vars.hpp"

namespace eufs::state::constraint {

template<typename T>
concept SingleVariable = IsSingleVariable<T>::value;

}  // namespace eufs::state::constraint

#endif  // INCLUDE_STATE_LIB_CONSTRAINT_SINGLE_VARIABLE_HPP_
