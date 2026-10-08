#include "map_lib/python/io/reader.hpp"

#include <pybind11/stl.h>

#include <eigen3/Eigen/Dense>

#include "map_lib/io/reader.hpp"

namespace eufs::map::io::python {

void defineReaderConstants(pybind11::object module) {
    module.attr("EUFS_HEADER") = EUFS_HEADER;
}

}  // namespace eufs::map::io::python
