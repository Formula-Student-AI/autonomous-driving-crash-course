#include <pybind11/pybind11.h>

#include "state_lib/common_type/python/state_2d_vector.hpp"

namespace eufs::state::example::python {

PYBIND11_MODULE(_common_type, m) {
  m.doc() = "State lib examples.";

  defineState2DVector(m);
}

}  // namespace eufs::state::example::python
