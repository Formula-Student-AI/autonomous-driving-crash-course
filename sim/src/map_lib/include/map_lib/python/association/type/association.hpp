#ifndef INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_TYPE_ASSOCIATION_HPP_
#define INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_TYPE_ASSOCIATION_HPP_

#include <pybind11/pybind11.h>

namespace eufs::map::association::python {

void defineAssociationTypes(pybind11::object module);

}  // namespace eufs::map::association::python

#endif  // INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_TYPE_ASSOCIATION_HPP_
