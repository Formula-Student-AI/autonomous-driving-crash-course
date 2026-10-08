#ifndef INCLUDE_MAP_LIB_PYTHON_IO_READER_HPP_
#define INCLUDE_MAP_LIB_PYTHON_IO_READER_HPP_

#include <pybind11/pybind11.h>

namespace eufs::map::io::python {

void defineReaderConstants(pybind11::object module);

}

#endif  // INCLUDE_MAP_LIB_PYTHON_IO_READER_HPP_
