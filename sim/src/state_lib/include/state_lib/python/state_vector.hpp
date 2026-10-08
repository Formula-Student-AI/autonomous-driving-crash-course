#ifndef INCLUDE_STATE_LIB_PYTHON_STATE_VECTOR_HPP_
#define INCLUDE_STATE_LIB_PYTHON_STATE_VECTOR_HPP_

#include <fmt/core.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include <string>

#include "state_lib/state_vars.hpp"
#include "state_lib/python/state_vars.hpp"
#include "state_lib/python/access.hpp"
#include "state_lib/python/mutate.hpp"
#include "state_lib/python/get_item.hpp"
#include "state_lib/python/set_item.hpp"
#include "state_lib/constraint/variables.hpp"
#include "state_lib/constraint/vector.hpp"

namespace eufs::state::python {

template <StateVars vars, typename DataType = double>
pybind11::class_<StateVector<vars, double>> defineStateVectorType(
    pybind11::object module, const std::string &name) {
  using namespace pybind11::literals;  // NOLINT
  using Vector = StateVector<vars, double>;
  using Array = std::array<double, GetVarsNumber(vars)>;

  auto factory = pybind11::class_<Vector>(module, name.c_str())
    .def(pybind11::init<>())
    .def(pybind11::init<const Eigen::VectorXd &>())
    .def(pybind11::init<Array>())
    .def("__iadd__", [](Vector &vec, const Eigen::VectorXd& v) {
      vec += v;
      return vec;
    }, "other"_a)
    .def("__iadd__", [](Vector &vec, Array v) {
      vec += v;
      return vec;
    }, "other"_a)
    .def("__isub__", [](Vector &vec, const Eigen::VectorXd& v) {
      vec -= v;
      return vec;
    }, "other"_a)
    .def("__isub__", [](Vector &vec, Array v) {
      vec -= v;
      return vec;
    }, "other"_a)
    .def("to_numpy", [](const Vector &vec) {
      return vec();
    });

  return factory;
}

template <constraint::Vector StateVector, constraint::Variables Vars, typename DataType = double>
pybind11::class_<std::remove_cv_t<Vars>> defineInteractions(
    pybind11::object module, pybind11::class_<StateVector> &vec, const std::string &name) {
  constexpr Vars var = {};

  vec.def_property(name.c_str(),
      &state::python::Access<StateVector, var, DataType>,
      &state::python::Mutate<StateVector, var, DataType>)
    .def("__getitem__", &GetItem<StateVector, Vars, DataType>)
    .def("__setitem__", &SetItem<StateVector, Vars, DataType>);

  // This works, but normally removing cv is a very bad idea
  return defineStateVarsType<std::remove_cv_t<Vars>>(module, fmt::format("{}_type", name));
}

}  // namespace eufs::state::python

#endif  // INCLUDE_STATE_LIB_PYTHON_STATE_VECTOR_HPP_
