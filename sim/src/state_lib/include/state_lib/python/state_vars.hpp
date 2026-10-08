#ifndef INCLUDE_STATE_LIB_PYTHON_STATE_VARS_HPP_
#define INCLUDE_STATE_LIB_PYTHON_STATE_VARS_HPP_

#include <string>

#include "state_lib/state_vars.hpp"

namespace eufs::state::python {

template <typename StateVars>
pybind11::class_<StateVars> defineStateVarsType(pybind11::object module, const std::string &name) {
  return pybind11::class_<StateVars>(module, name.c_str())
    .def(pybind11::init<>());
}

}  // namespace eufs::state::python

#endif  // INCLUDE_STATE_LIB_PYTHON_STATE_VARS_HPP_
