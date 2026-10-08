#ifndef INCLUDE_MAP_LIB_ASSOCIATION_TYPE_ASSOCIATION_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_TYPE_ASSOCIATION_HPP_

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <numeric>
#include <set>
#include <string>

#include <eufs_msgs/msg/cone_association_array.hpp>
#include <eufs_msgs/msg/cone_association.hpp>
#include <map_lib/type/map.hpp>
#include <map_lib/type/cone.hpp>
#include <map_lib/association/type/indexable.hpp>
#include "map_lib/constraint/in_fov_checker.hpp"
#include "map_lib/constraint/map.hpp"

namespace eufs::map::association {

struct Association {
  /**
   * @brief Matched observations to map landmarks
   */
  std::unordered_map<size_t, size_t> matched;

  /**
   * @brief Unmatched observations
   */
  std::unordered_set<size_t> unmatched;
};

/**
 * @brief Fill a container with [begin, end) with step size `step`.
 * Note that the contents of the container is cleared first.
 *
 * Main use case for this is to fill the `Association::unmatched`
 * attribute with the indices of the observations as std::iota
 * cannot be used in this case.
 *
 * @param set Container to fill with range
 * @param begin Start of the range
 * @param end End of the range
 * @param step Step size within [begin, end) (Default = 1)
 */
template <typename ContainerType, typename RangeType>
void FillRange(ContainerType &set, RangeType begin, RangeType end, RangeType step = 1) {  // NOLINT
  set.clear();  // Ensure the set is empty before we fill
  for (auto i = begin; i < end; i += step) {
    set.emplace(i);
  }
}

/**
 * @brief Convert associations to message
 *
 * @param associations Associations between the map and observations
 * @param observations The observations
 * @param map The map
 * @return eufs_msgs::msg::ConeAssociationArray message
 */
template <Indexable MapLike1, Indexable MapLike2>
eufs_msgs::msg::ConeAssociationArray ToMsg(const Association &associations,
                                           const MapLike1 &observations,
                                           const MapLike2 &map, double threshold,
                                           std::string association_type) {
  eufs_msgs::msg::ConeAssociationArray msg;
  msg.threshold = threshold;
  msg.type = association_type;
  for (const auto &[obs_idx, map_idx] : associations.matched) {
    eufs_msgs::msg::ConeAssociation pair_msg;
    pair_msg.first = eufs::map::ToMsg(observations.at(obs_idx));
    pair_msg.second = eufs::map::ToMsg(map.at(map_idx));
    msg.matched.push_back(pair_msg);
  }
  for (const auto &obs_idx : associations.unmatched) {
    msg.unmatched.push_back(ToMsg(observations.at(obs_idx)));
  }
  return msg;
}

/**
 * @brief Return a set of missed landmarks based on map and association information
 *
 * @param map Map centered at the pose of the vehicle
 * @param association Association information
 * @param fov Function which determines whether a cone is within detection range
*/
template <constraint::Map MapType, typename MissedPredicate>
requires constraint::InFOVChecker<MissedPredicate, typename MapType::value_type>
std::set<size_t> MissedLandmarks(const MapType &map,
                                    const Association &association,
                                    MissedPredicate pred) {
  std::vector<size_t> landmark_indices(map.size());
  std::iota(landmark_indices.begin(), landmark_indices.end(), 0);
  std::set<size_t> missed(landmark_indices.begin(), landmark_indices.end());
  for (const auto &[obs_idx, map_idx] : association.matched) {
    missed.erase(map_idx);
  }
  std::erase_if(missed, [&](const auto &idx){ return !pred(map.at(idx)); });
  return missed;
}

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_TYPE_ASSOCIATION_HPP_
