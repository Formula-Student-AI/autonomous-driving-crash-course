#ifndef INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_ASSOCIATOR_HPP_
#define INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_ASSOCIATOR_HPP_

#include <vector>

#include "map_lib/type/map.hpp"
#include "map_lib/type/cone.hpp"
#include "map_lib/association/type/association.hpp"
#include "map_lib/python/association/base_association.hpp"

namespace eufs::map::association::python {

template <template <typename, typename> class AssociationType,
  typename FilterType, typename MetricType>
class Associator : virtual public BaseAssociation<AssociationType> {
 public:
  /**
   * @brief Construct a new associator object
   *
   * @param filter_value Filter threshold
   */
  explicit Associator(double filter_value) : association_(filter_value) {}

  /**
   * @brief Destructor
   */
  ~Associator() override {}

  /**
   * @brief Match a list of observations to a list of predicted observations from the map.
   *
   * @param obs List of observations to associate
   * @param map The map of cones to match against
   * @return Association A structure containing matched pairs and unmatched observations
   */
  Association Match(const std::vector<Cone> &obs, const Map<Cone> &map) override {
    return association_.Match(obs, map);
  };

  /**
   * @brief Match a list of observations to a list of predicted observations from the map.
   *
   * @param obs List of observations to associate
   * @param map The map of cones to match against
   * @return Association A structure containing matched pairs and unmatched observations
   */
  Association Match(const std::vector<Cone> &obs, const std::vector<Cone> &map) override {
    return association_.Match(obs, map);
  };

 private:
  AssociationType<FilterType, MetricType> association_;
};

}  // namespace eufs::map::association::python

#endif  // INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_ASSOCIATOR_HPP_
