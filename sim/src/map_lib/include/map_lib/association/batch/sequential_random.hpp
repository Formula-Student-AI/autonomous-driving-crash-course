#ifndef INCLUDE_MAP_LIB_ASSOCIATION_BATCH_SEQUENTIAL_RANDOM_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_BATCH_SEQUENTIAL_RANDOM_HPP_

#include <algorithm>
#include <mutex>  // NOLINT
#include <numeric>
#include <random>
#include <tuple>
#include <vector>

#include <map_lib/type/map.hpp>

#include "map_lib/association/type/association.hpp"
#include "map_lib/association/type/associable.hpp"

namespace eufs::map::association {

template <typename FilterType, typename MetricType>
class SequentialRandomAssociation {
 public:
  /**
   * @brief Construct a new Sequential Random Association object
   *
   * @param filter_value Filter threshold
   */
  explicit SequentialRandomAssociation(double filter_value,
                                       uint seed = std::random_device{}())  // NOLINT
      : filter_value_(filter_value), rng_(seed) {}

  /**
   * @brief Match a list of observations to the map.
   *
   * @param obs List of observations to associate
   * @param map The map of cones to match against
   * @return Association structure containing matched pairs and unmatched observations
   */
  template <typename ConeType, Associable ConeContainer>
  Association Match(const std::vector<ConeType> &obs, const ConeContainer &map) {
    Association result;

    // If map is empty, everything is unmatched
    if (map.size() == 0) {
      FillRange(result.unmatched, 0lu, obs.size());
      return result;
    }

    // Randomly shuffle the observations
    auto obs_copy = ShuffleVector(obs);

    std::vector<size_t> available_map_idxs(map.size());
    std::iota(available_map_idxs.begin(), available_map_idxs.end(), 0);

    for (size_t obs_idx = 0; obs_idx < obs_copy.size(); ++obs_idx) {
      double best_metric = 0;
      size_t best_map_idx = 0;
      bool found = false;

      // Search through map and find the closest cone within maximum distance
      for (const size_t map_idx : available_map_idxs) {
        if (filter_(obs_copy.at(obs_idx), map.at(map_idx), filter_value_)) {
          auto current_metric = metric_(obs_copy.at(obs_idx), map.at(map_idx));
          if (!found || current_metric < best_metric) {
            best_metric = current_metric;
            best_map_idx = map_idx;
            found = true;
          }
        }
      }

      if (found) {
        result.matched.emplace(obs_idx, best_map_idx);
        auto used_index =
            std::find(available_map_idxs.begin(), available_map_idxs.end(), best_map_idx);
        available_map_idxs.erase(used_index);
      } else {
        result.unmatched.emplace(obs_idx);
      }
    }

    return result;
  }

 private:
  /**
   * @brief Shuffle vector of observations in-place
   *
   * @param obs Vector of observations
   */
  template <typename ConeType>
  std::vector<ConeType> ShuffleVector(std::vector<ConeType> obs) {
    const std::lock_guard<std::mutex> lk(rng_mutex_);
    std::shuffle(obs.begin(), obs.end(), rng_);
    return obs;
  }

 private:
  double filter_value_;
  std::mt19937 rng_;
  std::mutex rng_mutex_;
  MetricType metric_;
  FilterType filter_;
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_BATCH_SEQUENTIAL_RANDOM_HPP_
