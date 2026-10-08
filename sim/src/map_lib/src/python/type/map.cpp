#include "map_lib/python/type/map.hpp"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <memory>
#include <vector>

#include "map_lib/io/reader.hpp"
#include "map_lib/io/writer.hpp"
#include "map_lib/transform.hpp"
#include "map_lib/type/cone.hpp"
#include "map_lib/type/map.hpp"

namespace eufs::map::python {

void defineMapTypes(pybind11::object module) {
  using namespace pybind11::literals;  // NOLINT

  pybind11::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const std::ifstream::failure &e) {
      PyErr_SetString(PyExc_FileNotFoundError, e.what());
    }
  });

  pybind11::class_<Map<Cone>, std::shared_ptr<Map<Cone>>>(module, "Map")
      .def(pybind11::init<>())
      .def(pybind11::init<const std::vector<Cone> &>())
      .def("add", pybind11::overload_cast<const Cone &>(&Map<Cone>::Add))
      .def("add", pybind11::overload_cast<const std::vector<Cone> &>(&Map<Cone>::Add))

      .def("delete",
           [](Map<Cone> &map, size_t idx) {
             map.Remove(idx);
             return map;
           })

      .def("cones",
           static_cast<std::vector<Cone> (Map<Cone>::*)(std::optional<Color>) const>(
               &Map<Cone>::Get),
           "color"_a = pybind11::none())

      .def("cones",
           static_cast<std::vector<Cone> (Map<Cone>::*)(const std::vector<std::size_t> &) const>(
               &Map<Cone>::Get),
           "indices"_a)

      .def("positions",
           static_cast<std::vector<Eigen::Vector2d> (Map<Cone>::*)(std::optional<Color>) const>(
               &Map<Cone>::GetPositions),
           "color"_a = pybind11::none())

      .def("positions",
           static_cast<std::vector<Eigen::Vector2d> (Map<Cone>::*)(const std::vector<std::size_t> &)
                           const>(&Map<Cone>::GetPositions),
           "indices"_a)

      .def("at", pybind11::overload_cast<size_t>(&Map<Cone>::at, pybind11::const_))
      .def("size", &Map<Cone>::size)

      .def("copy", [](const Map<Cone> &map) { return map; })

      .def("__getitem__",
           [](const Map<Cone> &map, ssize_t idx) {
             if (0 <= idx && idx < std::ssize(map)) {
               return map.at(idx);
             } else if (-idx < std::ssize(map)) {
               return map.at(std::ssize(map) + idx);
             } else {
               throw pybind11::index_error();
             }
           })
      .def("__setitem__",
           [](Map<Cone> &map, size_t idx, const Cone &cone) {
             if (idx >= map.size()) throw pybind11::index_error();
             map.at(idx) = cone;
           })
      .def(
          "__iter__",
          [](const Map<Cone> &map) { return pybind11::make_iterator(map.begin(), map.end()); },
          pybind11::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)

      .def("__getitem__",
           [](const Map<Cone> &map, pybind11::slice slice) -> Map<Cone> * {
             ssize_t start, stop, step, slicelength;
             if (!slice.compute(map.size(), &start, &stop, &step, &slicelength))
               throw pybind11::error_already_set();
             Map<Cone> *map_slice = new Map<Cone>();
             for (ssize_t i = 0; i < slicelength; ++i) {
               map_slice->Add(map.at(start));
               start += step;
             }
             return map_slice;
           })
      .def("__setitem__",
           [](Map<Cone> &map, pybind11::slice slice, const std::vector<Cone> &value) {
             size_t start, stop, step, slicelength;
             if (!slice.compute(map.size(), &start, &stop, &step, &slicelength))
               throw pybind11::error_already_set();
             if (slicelength != value.size())
               throw std::runtime_error(
                   "Left and right hand size of slice assignment have different sizes!");
             for (size_t i = 0; i < slicelength; ++i) {
               map.at(start) = value.at(i);
               start += step;
             }
           })

      .def("translate", [](std::shared_ptr<Map<Cone>> &map,
            const Eigen::Vector2d &translation, const bool inplace, const bool inverse) {
          auto res = inverse ?
            Transformation::Translate<Map<Cone>, true>
              (inplace ? *map : std::as_const(*map), translation)
            : Transformation::Translate<Map<Cone>, false>
              (inplace ? *map : std::as_const(*map), translation);
          return inplace ? map : std::make_shared<Map<Cone>>(res);
      }, pybind11::arg("translation"),
        pybind11::arg("inplace") = false,
        pybind11::arg("inverse") = false)

      .def("rotate", [](std::shared_ptr<Map<Cone>> &map,
            const Eigen::Matrix2d &mat, const bool inplace, const bool inverse) {
          Eigen::Isometry2d transform;
          transform.linear() = mat;

          auto res = inverse ?
            Transformation::Transform<Map<Cone>, true>
              (inplace ? *map : std::as_const(*map), transform)
            : Transformation::Transform<Map<Cone>, false>
              (inplace ? *map : std::as_const(*map), transform);
          return inplace ? map : std::make_shared<Map<Cone>>(res);
      }, pybind11::arg("rotation"),
        pybind11::arg("inplace") = false,
        pybind11::arg("inverse") = false)

      .def("rotate_yaw", [](std::shared_ptr<Map<Cone>> &map,
            const double yaw, const bool inplace, const bool inverse) {
          auto res = inverse ?
            Transformation::Rotate<Map<Cone>, true>
              (inplace ? *map : std::as_const(*map), yaw)
            : Transformation::Rotate<Map<Cone>, false>
              (inplace ? *map : std::as_const(*map), yaw);
          return inplace ? map : std::make_shared<Map<Cone>>(res);
      }, pybind11::arg("yaw"),
        pybind11::arg("inplace") = false,
        pybind11::arg("inverse") = false)

      .def("transform", [](std::shared_ptr<Map<Cone>> &map, const Eigen::Matrix2d &mat,
            const Eigen::Vector2d &translation, const bool inplace, const bool inverse) {
          Eigen::Isometry2d transform;
          transform.linear() = mat;
          transform.translation() = translation;

          auto res = inverse ?
            Transformation::Transform<Map<Cone>, true>
              (inplace ? *map : std::as_const(*map), transform)
            : Transformation::Transform<Map<Cone>, false>
              (inplace ? *map : std::as_const(*map), transform);
          return inplace ? map : std::make_shared<Map<Cone>>(res);
      }, pybind11::arg("rotation"),
        pybind11::arg("translation"),
        pybind11::arg("inplace") = false,
        pybind11::arg("inverse") = false)

      .def("transform_yaw", [](std::shared_ptr<Map<Cone>> &map, const double yaw,
            const Eigen::Vector2d &translation, const bool inplace, const bool inverse) {
          auto res = inverse ?
            Transformation::Transform<Map<Cone>, true>
              (inplace ? *map : std::as_const(*map), yaw, translation)
            : Transformation::Transform<Map<Cone>, false>
              (inplace ? *map : std::as_const(*map), yaw, translation);

          return inplace ? map : std::make_shared<Map<Cone>>(res);
      }, pybind11::arg("yaw"),
        pybind11::arg("translation"),
        pybind11::arg("inplace") = false,
        pybind11::arg("inverse") = false)

      .def_static(
          "from_csv", [](const std::string &path) { return Map<Cone>(io::LoadCones<Cone>(path)); },
          pybind11::arg("path"))
      .def(
          "to_csv",
          [](const Map<Cone> &map,
             const std::string &path) { return io::SaveCones<Cone>(map.ToCones(), path); },
          pybind11::arg("path"));
}
}  // namespace eufs::map::python
