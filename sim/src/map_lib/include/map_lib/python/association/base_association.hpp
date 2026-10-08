#ifndef INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_BASE_ASSOCIATION_HPP_
#define INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_BASE_ASSOCIATION_HPP_

#include <vector>

#include "map_lib/type/cone.hpp"
#include "map_lib/type/map.hpp"
#include "map_lib/association/type/association.hpp"

namespace eufs::map::association::python {

template<template <typename, typename> class AssociationType>
class BaseAssociation {
 public:
  /**
   * @brief Virtual Destructor
   */
  virtual ~BaseAssociation() = default;

  /**
   * @brief Match a list of observations to a list of predicted observations from the map.
   *
   * @param obs List of observations to associate
   * @param map The map of cones to match against
   * @return Association A structure containing matched pairs and unmatched observations
   */
  virtual Association Match(const std::vector<Cone> &obs, const Map<Cone> &map) = 0;

  /**
   * @brief Match a list of observations to a list of predicted observations from the map.
   *
   * @param obs List of observations to associate
   * @param map The map of cones to match against
   * @return Association A structure containing matched pairs and unmatched observations
   */
  virtual Association Match(const std::vector<Cone> &obs, const std::vector<Cone> &map) = 0;
};

}  // namespace eufs::map::association::python

#endif  // INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_BASE_ASSOCIATION_HPP_
