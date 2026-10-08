#include <pybind11/pybind11.h>

#include "map_lib/python/io/reader.hpp"

namespace eufs::map::io::python {

PYBIND11_MODULE(_io, m) {
  m.doc() = "Map lib io.";

  defineReaderConstants(m);
}

}  // namespace eufs::map::io::python
