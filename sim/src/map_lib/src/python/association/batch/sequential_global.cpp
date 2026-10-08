#include "map_lib/python/association/batch/sequential_global.hpp"

#include <pybind11/stl.h>

#include "map_lib/type/cone.hpp"
#include "map_lib/type/map.hpp"

#include "map_lib/association/batch/sequential_global.hpp"
#include "map_lib/python/association/base_association.hpp"
#include "map_lib/python/association/association_factory.hpp"

namespace eufs::map::association::python {

void defineSequentialGlobalAssociations(pybind11::object module) {
    using namespace pybind11::literals;  // NOLINT
    using Association = BaseAssociation<SequentialGlobalAssociation>;

    pybind11::class_<Association>(module, "SequentialGlobalAssociation")
      .def(pybind11::init(
        pybind11::overload_cast<const double, const std::string &, const std::string &>(
        &AssociationFactory<SequentialGlobalAssociation>::Create)),
        "filter_value"_a, "filter_type"_a = "color_threshold", "metric_type"_a = "euclidean")
      .def("match",
        pybind11::overload_cast<const std::vector<Cone> &, const Map<Cone> &>
        (&Association::Match), "obs"_a, "map"_a)
      .def("match",
        pybind11::overload_cast<const std::vector<Cone> &, const std::vector<Cone> &>
        (&Association::Match), "obs"_a, "map"_a);
}

}  // namespace eufs::map::association::python
