#ifndef INCLUDE_STATE_LIB_COMMON_TYPE_PYTHON_STATE_2D_VECTOR_HPP_
#define INCLUDE_STATE_LIB_COMMON_TYPE_PYTHON_STATE_2D_VECTOR_HPP_

#include <pybind11/pybind11.h>

#include "state_lib/state_vars.hpp"

namespace eufs::state::example::python {

void defineState2DVector(pybind11::object module);

}  // namespace eufs::state::example::python

#endif  // INCLUDE_STATE_LIB_COMMON_TYPE_PYTHON_STATE_2D_VECTOR_HPP_
