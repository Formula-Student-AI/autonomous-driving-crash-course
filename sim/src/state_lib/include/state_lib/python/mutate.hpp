#ifndef INCLUDE_STATE_LIB_PYTHON_MUTATE_HPP_
#define INCLUDE_STATE_LIB_PYTHON_MUTATE_HPP_

#include <fmt/format.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "state_lib/state_vars.hpp"
#include "state_lib/state_vector.hpp"
#include "state_lib/constraint/single_variable.hpp"
#include "state_lib/constraint/multiple_variables.hpp"
#include "state_lib/python/constraint/single_mutable.hpp"
#include "state_lib/python/constraint/multiple_mutable.hpp"

namespace eufs::state::python {

/**
 * @brief Mutate one entry of a container
 * 
 * @tparam MutableType Type of the container
 * @tparam vars The single entry to update
 * @tparam DataType The type of container entries
 * @param a The container
 * @param value The value used to update the container
 */
template <typename MutableType, StateVars vars, typename DataType>
  requires constraint::SingleVariable<decltype(vars)>
        && constraint::SingleMutable<MutableType, vars, DataType>
void Mutate(MutableType &a, const DataType value) {
  a[vars] = value;
}

/**
 * @brief Mutate multiple entries of a container
 * 
 * @tparam MutableType Type of the container
 * @tparam vars The collection of entries to update
 * @tparam DataType The type of container entries
 * @param a The container
 * @param obj The values used to update the container
 */
template <typename MutableType, StateVars vars, typename DataType>
  requires constraint::MultipleVariables<decltype(vars)>
        && constraint::MultipleMutable<MutableType, vars, DataType>
void Mutate(MutableType &a, pybind11::object &obj) {
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

#endif  // INCLUDE_STATE_LIB_PYTHON_MUTATE_HPP_
