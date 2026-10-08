#ifndef INCLUDE_STATE_LIB_CONSTRAINT_MULTIPLE_VARIABLES_HPP_
#define INCLUDE_STATE_LIB_CONSTRAINT_MULTIPLE_VARIABLES_HPP_

#include <type_traits>

#include "state_lib/state_vars.hpp"

namespace eufs::state::constraint {

template<typename T>
concept MultipleVariables = std::negation_v<IsSingleVariable<T>>;

}  // namespace eufs::state::constraint

#endif  // INCLUDE_STATE_LIB_CONSTRAINT_MULTIPLE_VARIABLES_HPP_
