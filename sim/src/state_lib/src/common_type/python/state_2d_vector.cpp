#include "state_lib/common_type/python/state_2d_vector.hpp"

#include <pybind11/pybind11.h>

#include "state_lib/common_type/state_2d.hpp"
#include "state_lib/python/state_vector.hpp"
#include "state_lib/python/state_vars.hpp"
#include "state_lib/python/mutate.hpp"
#include "state_lib/python/access.hpp"
#include "state_lib/state_mask.hpp"

namespace eufs::state::example::python {

void defineState2DVector(pybind11::object module) {
  using namespace pybind11::literals;  // NOLINT

  auto vec_type = state::python::defineStateVectorType<State2DVars::vars>(module, "State2DVector");

  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_x)>(module, vec_type, "x");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_y)>(module, vec_type, "y");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_yaw)>(module, vec_type, "yaw");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_v_x)>(module, vec_type, "v_x");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_v_y)>(module, vec_type, "v_y");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_v_yaw)>(module, vec_type, "v_yaw");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_a_x)>(module, vec_type, "a_x");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::_a_y)>(module, vec_type, "a_y");
  state::python::defineInteractions<
    State2DVector, decltype(State2DVars::pose)>(module, vec_type, "pose");
}

}  // namespace eufs::state::example::python
