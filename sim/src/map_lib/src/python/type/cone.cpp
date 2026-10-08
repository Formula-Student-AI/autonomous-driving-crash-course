#include "map_lib/python/type/cone.hpp"

#include <fmt/core.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <eigen3/Eigen/Dense>
#include <eufs_logger/eigen.hpp>

#include "map_lib/type/cone.hpp"

namespace eufs::map::python {

void defineConeTypes(pybind11::object module) {
  using namespace pybind11::literals;  // NOLINT

  pybind11::enum_<Color>(module, "Color")
      .value("BLUE", Color::BLUE)
      .value("YELLOW", Color::YELLOW)
      .value("ORANGE", Color::ORANGE)
      .value("UNKNOWN", Color::UNKNOWN);

  pybind11::class_<Cone>(module, "Cone")
      .def(pybind11::init<>())
      .def(pybind11::init<const Eigen::Vector2d &, const Color &>(),
        "position"_a, "color"_a = Color::UNKNOWN
      )
      .def(pybind11::init<const std::array<double, 2> &, const Color &>(),
        "position"_a, "color"_a = Color::UNKNOWN
      )
      .def(pybind11::init<const Eigen::Vector2d &, const Eigen::Matrix2d &, const Color &>(),
        "position"_a, "covariance"_a, "color"_a = Color::UNKNOWN
      )
      .def(pybind11::init<const std::array<double, 2> &, const std::array<double, 4> &,
                          const Color &>(),
        "position"_a, "covariance"_a, "color"_a = Color::UNKNOWN
      )
      .def(pybind11::init<const eufs_msgs::msg::ConeWithCovariance &, const Color &>(),
        "cone"_a, "color"_a = Color::UNKNOWN
      )

      .def_property("color", pybind11::overload_cast<>(&Cone::GetColor, pybind11::const_),
                    [](Cone &cone, Color color) {
                      cone.GetColor() = color;
                    })

      .def_readwrite("position", &Cone::position)
      .def_readwrite("covariance", &Cone::covariance)

      .def("__eq__",
           [&](const Cone &cone, const Cone &other) {
             return cone.position == other.position && cone.covariance == other.covariance &&
                    cone.GetColor() == other.GetColor();
           })

      .def("__repr__", [&](const Cone &cone) {
        pybind11::object obj = pybind11::cast(cone.GetColor());

        return fmt::format(
            "Position: {}\nCovariance: {}\nColor: {}\n", logging::eigen::to_str(cone.position),
            logging::eigen::to_str(cone.covariance), pybind11::repr(obj).cast<std::string>());
      });
}

}  // namespace eufs::map::python
