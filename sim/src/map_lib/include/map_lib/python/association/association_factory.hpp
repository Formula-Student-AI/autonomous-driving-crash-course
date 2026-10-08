#ifndef INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_ASSOCIATION_FACTORY_HPP_
#define INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_ASSOCIATION_FACTORY_HPP_

#include <memory>
#include <string>

#include "map_lib/association/cone_association.hpp"

#include "map_lib/python/association/associator.hpp"
#include "map_lib/python/association/base_association.hpp"

namespace eufs::map::association::python {

template <template <typename, typename> class AssociationType>
class AssociationFactory {
 public:
  /**
   * @brief Get a specific associator by specifying template types as strings.
   *
   * @param filter_value
   * @param filter_type string representation of filter type
   * @param metric_type string representation of metric type
   * @return Associator
   */
  static std::unique_ptr<BaseAssociation<AssociationType>> Create(
      double filter_value, const std::string &filter_type, const std::string &metric_type) {
    if (metric_type == "bhattacharyya") {
      return AssociationFactory<AssociationType>::Create<Bhattacharyya>(filter_value, filter_type);
    } else if (metric_type == "euclidean") {
      return AssociationFactory<AssociationType>::Create<Euclidean>(filter_value, filter_type);
    } else if (metric_type == "mahalanobis") {
      return AssociationFactory<AssociationType>::Create<Mahalanobis>(filter_value, filter_type);
    }

    return AssociationFactory<AssociationType>::Create
      <NegativeLogLikelihood>(filter_value, filter_type);
  }

  /**
   * @brief Get a specific associator by specifying template types as strings.
   *
   * @param filter_value
   * @param filter_type string representation of filter type
   * @return Associator
   */
  template<typename MetricType>
  static std::unique_ptr<BaseAssociation<AssociationType>>
  Create(double filter_value, const std::string &filter_type) {
    if (filter_type == "color_threshold") {
        return std::make_unique<
          Associator<AssociationType, ColorThreshold<MetricType>, MetricType>>(filter_value);
    }

    return std::make_unique<
      Associator<AssociationType, Threshold<MetricType>, MetricType>>(filter_value);
  }
};

}  // namespace eufs::map::association::python

#endif  // INCLUDE_MAP_LIB_PYTHON_ASSOCIATION_ASSOCIATION_FACTORY_HPP_
