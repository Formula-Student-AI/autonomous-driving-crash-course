#ifndef INCLUDE_MAP_LIB_ASSOCIATION_SINGLE_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_SINGLE_HPP_

#include <memory>
#include <numeric>
#include <iterator>
#include <algorithm>
#include <vector>

#include <map_lib/type/map.hpp>

#include "map_lib/association/type/association.hpp"
#include "map_lib/association/type/associable.hpp"

namespace eufs::map::association {

template <typename FilterType, typename MetricType>
class SingleAssociation {
 public:
  /**
   * @brief Construct a new Single Association object
   *
   * @param filter_value Filter threshold
   */
  explicit SingleAssociation(double filter_value) : filter_value_(filter_value) {}

  /**
   * @brief Match a list of observations to the map.
   *
   * @param obs List of observations to associate
   * @param map The map of cones to match against
   * @return Association structure containing matched pairs and unmatched observations
   */
  template <typename ConeType, Associable ConeContainer>
  Association Match(const std::vector<ConeType> &obs, const ConeContainer &map) const {
    Association result;

    // If map is empty, everything is unmatched
    if (map.size() == 0) {
      FillRange(result.unmatched, 0lu, obs.size());
      return result;
    }

    // At this point, there is at least one element in the map
    for (size_t obs_idx = 0; obs_idx < obs.size(); ++obs_idx) {
      double best_metric = 0;
      size_t best_map_idx = 0;
      bool found = false;

      // Search through map and find the closest cone within maximum distance
      for (size_t map_idx = 0; map_idx < map.size(); ++map_idx) {
        if (filter_(obs.at(obs_idx), map.at(map_idx), filter_value_)) {
          auto current_metric = metric_(obs.at(obs_idx), map.at(map_idx));
          if (!found || current_metric < best_metric) {
            best_metric = current_metric;
            best_map_idx = map_idx;
            found = true;
          }
        }
      }

      if (found) {
        result.matched.emplace(obs_idx, best_map_idx);
      } else {
        result.unmatched.emplace(obs_idx);
      }
    }

    return result;
  }

 private:
  double filter_value_;
  MetricType metric_;
  FilterType filter_;
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_SINGLE_HPP_
