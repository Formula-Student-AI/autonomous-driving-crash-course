#ifndef INCLUDE_MAP_LIB_TRANSFORM_HPP_
#define INCLUDE_MAP_LIB_TRANSFORM_HPP_

#include <ranges>
#include <algorithm>
#include <iterator>

#include "eigen3/Eigen/Dense"

#include "map_lib/type/map.hpp"
#include "map_lib/type/add_iterator.hpp"
#include "map_lib/constraint/map.hpp"

namespace eufs::map {

class Transformation {
 public:
/**
 * Rotate cones in map by a given angle in place.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param yaw Angle to rotate by
 * @return Reference to map argument
 */
template <constraint::Map MapType, bool inverse = false>
static void Rotate(MapType &map, const double yaw) {
    if constexpr(inverse) {
      return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
        return Eigen::Rotation2D<double>(-yaw) * position;
      });
    }

    return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
      return Eigen::Rotation2D<double>(yaw) * position;
    });
}

/**
 * Rotate cones in map by a given angle.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param yaw Angle to rotate by
 * @return Transformed map
 */
template <constraint::Map MapType, bool inverse = false>
static MapType Rotate(const MapType &map, const double yaw) {
    if constexpr(inverse) {
      return CopyTransform(map, [&](const Eigen::Vector2d &position){
        return Eigen::Rotation2D<double>(-yaw) * position;
      });
    }

    return CopyTransform(map, [&](const Eigen::Vector2d &position){
      return Eigen::Rotation2D<double>(yaw) * position;
    });
}

/**
 * Translate cones in map by a given vector in place.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param v Translation vector
 * @return Reference to map argument
 */
template <constraint::Map MapType, bool inverse = false>
static void Translate(MapType &map, const Eigen::Vector2d &v) {
    if constexpr(inverse) {
      return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
        return position - v;
      });
    }

    return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
      return position + v;
    });
}

/**
 * Translate cones in map by a given vector.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param v Translation vector
 * @return Transformed map
 */
template <constraint::Map MapType, bool inverse = false>
static MapType Translate(const MapType &map, const Eigen::Vector2d &v) {
    if constexpr(inverse) {
      return CopyTransform(map, [&](const Eigen::Vector2d &position){
        return position - v;
      });
    }

    return CopyTransform(map, [&](const Eigen::Vector2d &position){
      return position + v;
    });
}

/**
 * Transform cones in map by a given angle and vector in place.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param yaw Angle to rotate by
 * @param v Translation vector
 * @return Reference to map argument
 */
template <constraint::Map MapType, bool inverse = false>
static void Transform(MapType &map, const double yaw, const Eigen::Vector2d &v) {
    if constexpr(inverse) {
      return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
        return Eigen::Vector2d{Eigen::Rotation2D<double>(-yaw) * (position - v)};
      });
    }

    return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
      return Eigen::Vector2d{Eigen::Rotation2D<double>(yaw) * position + v};
    });
}

/**
 * Transform cones in map by a given angle and vector.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param yaw Angle to rotate by
 * @param v Translation vector
 * @return Transformed map
 */
template <constraint::Map MapType, bool inverse = false>
static MapType Transform(const MapType &map, const double yaw, const Eigen::Vector2d &v) {
    if constexpr(inverse) {
      return CopyTransform(map, [&](const Eigen::Vector2d &position){
        return Eigen::Vector2d{Eigen::Rotation2D<double>(-yaw) * (position - v)};
      });
    }

    return CopyTransform(map, [&](const Eigen::Vector2d &position){
      return Eigen::Vector2d{Eigen::Rotation2D<double>(yaw) * position + v};
    });
}

/**
 * Transform cones in map in place.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param transform
 * @return Reference to map argument
 */
template <constraint::Map MapType, bool inverse = false>
static void Transform(MapType &map, const Eigen::Isometry2d &transform) {
    if constexpr(inverse) {
      return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
        return Eigen::Vector2d{transform.inverse() * position};
      });
    }

    return InPlaceTranform(map, [&](const Eigen::Vector2d &position){
      return Eigen::Vector2d{transform * position};
    });
}

/**
 * Transform cones in map.
 * 
 * @tparam MapType
 * @tparam Whether to invert the transform or not
 * @param map
 * @param transform
 * @return Transformed map
 */
template <constraint::Map MapType, bool inverse = false>
static MapType Transform(const MapType &map, const Eigen::Isometry2d &transform) {
    if constexpr(inverse) {
      return CopyTransform(map, [&](const Eigen::Vector2d &position){
        return Eigen::Vector2d{transform.inverse() * position};
      });
    }

    return CopyTransform(map, [&](const Eigen::Vector2d &position){
      return Eigen::Vector2d{transform * position};
    });
}

 private:
template <constraint::Map MapType>
static void InPlaceTranform(MapType &map,
  std::function<Eigen::Vector2d(const Eigen::Vector2d&)> func) {
    std::ranges::for_each(map, [&](typename MapType::value_type &cone) {
      cone.position = func(cone.position);
    });
}

template <constraint::Map MapType>
static MapType CopyTransform(const MapType &map,
  std::function<Eigen::Vector2d(const Eigen::Vector2d&)> func) {
    MapType updated_map;

    std::ranges::transform(map, AddIterator(updated_map), [&](typename MapType::value_type cone) {
      cone.position = func(cone.position);
      return cone;
    });

    return updated_map;
}
};
}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_TRANSFORM_HPP_
