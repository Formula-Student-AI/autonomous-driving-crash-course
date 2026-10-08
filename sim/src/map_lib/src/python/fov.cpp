#include <pybind11/pybind11.h>

#include "map_lib/fov.hpp"
#include "map_lib/python/fov.hpp"
#include "map_lib/type/cone.hpp"

namespace eufs::map::python {

void defineFOVCheckers(pybind11::object module) {
  using namespace pybind11::literals;  // NOLINT

  pybind11::class_<WithinSectorChecker>(module, "WithinSectorChecker")
    .def(pybind11::init(
      [](const double max_horizonal_radius, const double horizontal_fov) {
        return WithinSectorChecker({max_horizonal_radius, horizontal_fov});
      })
    )
    .def(pybind11::init<const SectorParameters &>())
    .def("inFOV", [](WithinSectorChecker *checker, const Cone &cone) { return (*checker)(cone); });

  pybind11::class_<WithinAnnulusChecker>(module, "WithinAnnulusChecker")
    .def(pybind11::init(
      [](
        const double max_horizonal_radius,
        const double horizontal_fov,
        const double min_horizonal_radius,
        const double fov_center
      ) {
        return WithinAnnulusChecker(
          {
            max_horizonal_radius,
            horizontal_fov,
            min_horizonal_radius,
            fov_center
          });
      })
    )
    .def(pybind11::init<const AnnulusParameters &>())
    .def("inFOV", [](WithinAnnulusChecker *checker, const Cone &cone) { return (*checker)(cone); });
}

PYBIND11_MODULE(_fov, m) {
  m.doc() = "Map lib FOV checkers.";

  defineFOVCheckers(m);
}

}  // namespace eufs::map::python
