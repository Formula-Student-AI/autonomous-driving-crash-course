#include "map_lib/python/association/type/association.hpp"

#include <pybind11/stl.h>

#include "map_lib/association/type/association.hpp"

namespace eufs::map::association::python {

void defineAssociationTypes(pybind11::object module) {
    pybind11::class_<Association>(module, "Association")
      .def_readwrite("matched", &Association::matched)
      .def_readwrite("unmatched", &Association::unmatched);
}

}  // namespace eufs::map::association::python
