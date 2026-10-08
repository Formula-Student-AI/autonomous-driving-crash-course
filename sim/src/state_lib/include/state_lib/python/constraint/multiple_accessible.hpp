#ifndef INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_MULTIPLE_ACCESSIBLE_HPP_
#define INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_MULTIPLE_ACCESSIBLE_HPP_

#include <Eigen/Dense>

#include <concepts>

#include "state_lib/state_mask.hpp"
#include "state_lib/state_vars.hpp"

namespace eufs::state::constraint {

template<typename T, StateVars vars, typename DataType>
concept MultipleAccessible = requires(T a) {
    { a(StateMask(vars)) }
      -> std::convertible_to<Eigen::Matrix<DataType, GetVarsNumber(vars), 1>>;
};  // NOLINT

}  // namespace eufs::state::constraint

#endif  // INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_MULTIPLE_ACCESSIBLE_HPP_
