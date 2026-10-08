#ifndef INCLUDE_STATE_LIB_PYTHON_ACCESS_HPP_
#define INCLUDE_STATE_LIB_PYTHON_ACCESS_HPP_

#include "state_lib/state_vars.hpp"
#include "state_lib/state_mask.hpp"
#include "state_lib/state_vector.hpp"
#include "state_lib/constraint/single_variable.hpp"
#include "state_lib/constraint/multiple_variables.hpp"
#include "state_lib/python/constraint/single_accessible.hpp"
#include "state_lib/python/constraint/multiple_accessible.hpp"

namespace eufs::state::python {

/**
 * @brief Access one entry of a container
 * 
 * @tparam AccessibleType Type of the container
 * @tparam vars The single entry accessed
 * @tparam DataType The type of container entries
 * @param a The container
 */
template <typename AccessibleType, StateVars vars, typename DataType>
  requires constraint::SingleVariable<decltype(vars)>
        && constraint::SingleAccessible<AccessibleType, vars, DataType>
double Access(const AccessibleType &a) {
  return a(vars);
}

/**
 * @brief Access multiple entries of a container
 * 
 * @tparam AccessibleType Type of the container
 * @tparam vars The collection of entries accessed
 * @tparam DataType The type of container entries
 * @param a The container
 */
template <typename AccessibleType, StateVars vars, typename DataType>
  requires constraint::MultipleVariables<decltype(vars)>
        && constraint::MultipleAccessible<AccessibleType, vars, DataType>
Eigen::VectorXd Access(const AccessibleType &a) {
  return a(StateMask(vars));
}

}  // namespace eufs::state::python

#endif  // INCLUDE_STATE_LIB_PYTHON_ACCESS_HPP_
