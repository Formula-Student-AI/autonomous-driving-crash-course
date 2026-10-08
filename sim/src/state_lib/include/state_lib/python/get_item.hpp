#ifndef INCLUDE_STATE_LIB_PYTHON_GET_ITEM_HPP_
#define INCLUDE_STATE_LIB_PYTHON_GET_ITEM_HPP_

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
 * @tparam StateVars The type of the requested variable
 * @tparam DataType The type of container entries
 * @param a The container
 * @param vars The single entry accessed
 */
template <typename AccessibleType, constraint::SingleVariable StateVars, typename DataType>
  requires constraint::SingleAccessible<AccessibleType, StateVars{}, DataType>
double GetItem(const AccessibleType &a, StateVars vars) {
  return a(vars);
}

/**
 * @brief Access multiple entries of a container
 * 
 * @tparam AccessibleType Type of the container
 * @tparam StateVars The types of the requested variables
 * @tparam DataType The type of container entries
 * @param a The container
 * @param vars The collection of entries accessed
 */
template <typename AccessibleType, constraint::MultipleVariables StateVars, typename DataType>
  requires constraint::MultipleAccessible<AccessibleType, StateVars{}, DataType>
Eigen::VectorXd GetItem(const AccessibleType &a, StateVars vars) {
  return a(StateMask(vars));
}

}  // namespace eufs::state::python

#endif  // INCLUDE_STATE_LIB_PYTHON_GET_ITEM_HPP_
