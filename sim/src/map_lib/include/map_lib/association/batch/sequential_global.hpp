#ifndef INCLUDE_MAP_LIB_ASSOCIATION_BATCH_SEQUENTIAL_GLOBAL_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_BATCH_SEQUENTIAL_GLOBAL_HPP_

#include <algorithm>
#include <numeric>
#include <tuple>
#include <vector>

#include <map_lib/type/map.hpp>

#include "map_lib/association/type/association.hpp"
#include "map_lib/association/type/associable.hpp"

namespace eufs::map::association {

template <typename FilterType, typename MetricType>
struct MetricTable {
  using PairingTuple = std::tuple<size_t, size_t, double>;  // (obs_idx, map_idx, metric value)

  /**
   * @brief Construct a new Metric Table object
   *
   * @param obs The list of observations to associate
   * @param map The map
   * @param filter_value Filter threshold
   */
  template <typename ConeType, typename ConeContainer>
  MetricTable(const std::vector<ConeType> &obs,
              const ConeContainer &map,
              double filter_value) {
    for (size_t obs_idx = 0; obs_idx < obs.size(); ++obs_idx) {
      for (size_t map_idx = 0; map_idx < map.size(); ++map_idx) {
        if (filter_(obs.at(obs_idx), map.at(map_idx), filter_value)) {
          table_.push_back({obs_idx, map_idx, metric_(obs.at(obs_idx), map.at(map_idx))});
        }
      }
    }
  }

  /**
   * @brief Return pairing with lowest metric value and remove pairs with the observation or map
   * element from the table.
   */
  PairingTuple RemoveMinElement() {
    // Find pair with lowest metric
    auto best_pairing = *std::min_element(
        table_.cbegin(), table_.cend(),
        [](const auto &a, const auto &b) { return std::get<2>(a) < std::get<2>(b); });
    // Remove observation and map element from consideration
    std::erase_if(table_, [&](const auto &x) {
      return (std::get<0>(x) == std::get<0>(best_pairing)) ||
             (std::get<1>(x) == std::get<1>(best_pairing));
    });
    return best_pairing;
  }

  /**
   * @brief Return number of pairs in the table
   *
   * @return Size
   */
  size_t size() const { return table_.size(); }

 private:
  std::vector<PairingTuple> table_;
  FilterType filter_;
  MetricType metric_;
};

template <typename FilterType, typename MetricType>
class SequentialGlobalAssociation {
 public:
  /**
   * @brief Construct a new Sequential Global Association object
   *
   * @param filter_value Filter threshold
   */
  explicit SequentialGlobalAssociation(double filter_value) : filter_value_(filter_value) {}

  /**
   * @brief Match a list of observations to a list of predicted observations from the map.
   *
   * @param obs List of observations to associate
   * @param map The map of cones to match against
   * @return Association A structure containing matched pairs and unmatched observations
   */
  template <typename ConeType, Associable ConeContainer>
  Association Match(const std::vector<ConeType> &obs, const ConeContainer &map) const {
    Association result;

    // If map is empty, everything is unmatched
    if (map.size() == 0) {
      FillRange(result.unmatched, 0lu, obs.size());
      return result;
    }

    // Construct metric table for all (observation, map) pairs
    auto metrics = MetricTable<FilterType, MetricType>(obs, map, filter_value_);

    // Find best pairings sequentially from the table
    while (metrics.size() > 0) {
      auto best_pairing = metrics.RemoveMinElement();
      result.matched.emplace(std::get<0>(best_pairing), std::get<1>(best_pairing));
    }

    // Find set of unmatched observations
    for (size_t obs_idx = 0; obs_idx < obs.size(); ++obs_idx) {
      if (result.matched.count(obs_idx) == 0) {
        result.unmatched.emplace(obs_idx);
      }
    }

    return result;
  }

 private:
  double filter_value_;
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_BATCH_SEQUENTIAL_GLOBAL_HPP_
