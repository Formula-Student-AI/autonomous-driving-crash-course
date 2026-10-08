#ifndef INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_MULTIPLE_MUTABLE_HPP_
#define INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_MULTIPLE_MUTABLE_HPP_

#include <array>

#include "state_lib/state_mask.hpp"

namespace eufs::state::constraint {

template<typename T, StateVars vars, typename DataType>
concept MultipleMutable = requires(
    T a,
    std::array<DataType, GetVarsNumber(vars)> values
) {
    a[StateMask(vars)] = values;
};  // NOLINT

}  // namespace eufs::state::constraint

#endif  // INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_MULTIPLE_MUTABLE_HPP_
