#include <pybind11/pybind11.h>

#include "map_lib/python/type/cone.hpp"
#include "map_lib/python/type/map.hpp"

namespace eufs::map::python {

PYBIND11_MODULE(_type, m) {
  m.doc() = "Map lib types.";

  defineConeTypes(m);
  defineMapTypes(m);
}

}  // namespace eufs::map::python
