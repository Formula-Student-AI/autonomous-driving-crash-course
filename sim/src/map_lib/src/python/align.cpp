#include <pybind11/pybind11.h>

#include "map_lib/python/association/align/align_svd.hpp"

namespace eufs::map::association::align::python {

PYBIND11_MODULE(_align, m) {
  m.doc() = "Map lib alignment.";

  defineAlignSVD(m);
}

}  // namespace eufs::map::association::align::python
