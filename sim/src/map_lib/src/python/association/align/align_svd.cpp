
#include "map_lib/python/association/align/align_svd.hpp"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "map_lib/association/align/align_svd.hpp"

namespace eufs::map::association::align::python {

void defineAlignSVD(pybind11::object module) {
    pybind11::class_<AlignSVD<std::vector>, std::shared_ptr<AlignSVD<std::vector>>>
      (module, "AlignSVD")
      .def(pybind11::init<
        const std::vector<Eigen::Vector2d> &,
        const std::vector<Eigen::Vector2d> &>())
      .def("find_transformation", [](std::shared_ptr<AlignSVD<std::vector>> &aligner){
        return aligner->FindTransformation().matrix();
      });
}

}  // namespace eufs::map::association::align::python
