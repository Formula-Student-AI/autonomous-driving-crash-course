#ifndef INCLUDE_MAP_LIB_PYTHON_FOV_HPP_
#define INCLUDE_MAP_LIB_PYTHON_FOV_HPP_

#include <pybind11/pybind11.h>

namespace eufs::map::python {

void defineFOVCheckers(pybind11::object module);

}

#endif  // INCLUDE_MAP_LIB_PYTHON_FOV_HPP_
