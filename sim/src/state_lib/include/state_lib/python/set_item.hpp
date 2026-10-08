#ifndef INCLUDE_STATE_LIB_PYTHON_SET_ITEM_HPP_
#define INCLUDE_STATE_LIB_PYTHON_SET_ITEM_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "state_lib/state_mask.hpp"
#include "state_lib/constraint/single_variable.hpp"
#include "state_lib/constraint/multiple_variables.hpp"
#include "state_lib/python/constraint/single_mutable.hpp"
#include "state_lib/python/constraint/multiple_mutable.hpp"

namespace eufs::state::python {

/**
 * @brief Mutate one entry of a container
 * 
 * @tparam MutableType Type of the container
 * @tparam StateVars The variable type targeted in the update
 * @tparam DataType The type of container entries
 * @param a The container
 * @param vars The single entry to update
 * @param value The value used to update the container
 */
template <typename MutableType, constraint::SingleVariable StateVars, typename DataType>
  requires constraint::SingleMutable<MutableType, StateVars{}, DataType>
void SetItem(MutableType &a, StateVars vars, const double value) {
  a[vars] = value;
}

/**
 * @brief Mutate multiple entries of a container
 * 
 * @tparam MutableType Type of the container
 * @tparam StateVars The type of variables targeted in the update
 * @tparam DataType The type of container entries
 * @param a The container
 * @param vars The collection of entries to update
 * @param obj The values used to update the container
 */
template <typename MutableType, constraint::MultipleVariables StateVars, typename DataType>
  requires constraint::MultipleMutable<MutableType, StateVars{}, DataType>
void SetItem(MutableType &a, StateVars vars, pybind11::object &obj) {
  if (pybind11::isinstance<pybind11::list>(obj)) {
    a[StateMask(vars)] = obj.cast<std::array<double, GetVarsNumber(vars)>>();
    return;
  } else if (pybind11::isinstance<pybind11::array>(obj)) {
    a[StateMask(vars)] = obj.cast<std::array<double, GetVarsNumber(vars)>>();
    return;
  }
  throw std::invalid_argument("Cannot cast object while mutating vector!");
}

}  // namespace eufs::state::python

#endif  // INCLUDE_STATE_LIB_PYTHON_SET_ITEM_HPP_
