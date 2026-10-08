#include <pybind11/pybind11.h>

#include "map_lib/python/association/single.hpp"
#include "map_lib/python/association/batch/sequential_global.hpp"
#include "map_lib/python/association/batch/sequential_random.hpp"

#include "map_lib/python/association/type/association.hpp"

namespace eufs::map::association::python {

PYBIND11_MODULE(_association, m) {
  m.doc() = "Map lib association.";

  defineAssociationTypes(m);

  defineSingleAssociations(m);
  defineSequentialGlobalAssociations(m);
  defineSequentialRandomAssociations(m);
}

}  // namespace eufs::map::association::python
