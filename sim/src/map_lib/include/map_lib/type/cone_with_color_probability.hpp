#ifndef INCLUDE_MAP_LIB_TYPE_CONE_WITH_COLOR_PROBABILITY_HPP_
#define INCLUDE_MAP_LIB_TYPE_CONE_WITH_COLOR_PROBABILITY_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <vector>
#include <memory>

#include "map_lib/constraint/cone_with_color_probability.hpp"
#include "map_lib/type/color_probabilities.hpp"

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_color_probability.hpp>
#include <eufs_msgs/msg/cone_with_color_probability_array.hpp>

namespace eufs::map {

struct ConeWithColorProbability {
  typedef std::shared_ptr<ConeWithColorProbability> Ptr;
  typedef std::shared_ptr<const ConeWithColorProbability> ConstPtr;

  // (x,y) coordinate of the cone
  Eigen::Vector2d position;

  // Covariance in the position of the cone
  Eigen::Matrix2d covariance;

  /**
   * @brief Construct a new ConeWithColorProbability object
   */
  ConeWithColorProbability() = default;

  /**
   * @brief Construct a new ConeWithColorProbability object
   *
   * @param pos (x,y)-coordinate
   * @param color_probabilities Color probabilities of the cone
   */
  explicit ConeWithColorProbability(const Eigen::Vector2d &pos,
    const ColorProbabilities &color_probabilities = {
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.})
      : position(pos),
      blue_probability(color_probabilities.blue),
      yellow_probability(color_probabilities.yellow),
      orange_probability(color_probabilities.orange),
      big_orange_probability(color_probabilities.big_orange),
      unknown_probability(color_probabilities.unknown) {}

  /**
   * @brief Construct a new ConeWithColorProbability object
   *
   * @param pos (x,y)-coordinate
   * @param color_probabilities Color probabilities of the cone
   */
  explicit ConeWithColorProbability(const std::array<double, 2> &pos,
    const ColorProbabilities &color_probabilities = {
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.})
      : position(pos.data()),
      blue_probability(color_probabilities.blue),
      yellow_probability(color_probabilities.yellow),
      orange_probability(color_probabilities.orange),
      big_orange_probability(color_probabilities.big_orange),
      unknown_probability(color_probabilities.unknown) {}

  /**
   * @brief Construct a new ConeWithColorProbability object
   *
   * @param pos (x,y)-coordinate
   * @param cov Covariance of position
   * @param color_probabilities Color probabilities of the cone
   */
  ConeWithColorProbability(const Eigen::Vector2d &pos, const Eigen::Matrix2d &cov,
    const ColorProbabilities &color_probabilities = {
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.})
      : position(pos),
        covariance(cov),
        blue_probability(color_probabilities.blue),
        yellow_probability(color_probabilities.yellow),
        orange_probability(color_probabilities.orange),
        big_orange_probability(color_probabilities.big_orange),
        unknown_probability(color_probabilities.unknown) {}

  /**
   * @brief Construct a new ConeWithColorProbability object
   *
   * @param pos (x,y)-coordinate
   * @param cov Covariance of x and y as a flattened array of size 4 (row-wise)
   * @param color_probabilities Color probabilities of the cone
   */
  ConeWithColorProbability(const std::array<double, 2> &pos, const std::array<double, 4> &cov,
       const ColorProbabilities &color_probabilities = {
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.})
      : position(pos.data()),
      covariance(cov.data()),
      blue_probability(color_probabilities.blue),
      yellow_probability(color_probabilities.yellow),
      orange_probability(color_probabilities.orange),
      big_orange_probability(color_probabilities.big_orange),
      unknown_probability(color_probabilities.unknown) {}

  /**
   * @brief Construct a new ConeWithColorProbability object
   *
   * @param msg eufs_msgs::msg::ConeWithCovariance message
   * @param color_probabilities Color probabilities of the cone
   */
  explicit ConeWithColorProbability(const eufs_msgs::msg::ConeWithCovariance &msg,
      const ColorProbabilities &color_probabilities = {
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.})
      : position(msg.point.x, msg.point.y),
      covariance(msg.covariance.data()),
      blue_probability(color_probabilities.blue),
      yellow_probability(color_probabilities.yellow),
      orange_probability(color_probabilities.orange),
      big_orange_probability(color_probabilities.big_orange),
      unknown_probability(color_probabilities.unknown) {}

    /**
   * @brief Construct a new ConeWithColorProbability object
   *
   * @param msg eufs_msgs::msg::ConeWithColorProbability message
   */
  explicit ConeWithColorProbability(const eufs_msgs::msg::ConeWithColorProbability &msg)
      : position(msg.point.x, msg.point.y),
      covariance(msg.covariance.data()),
      blue_probability(msg.blue_prob),
      yellow_probability(msg.yellow_prob),
      orange_probability(msg.orange_prob),
      big_orange_probability(msg.big_orange_prob),
      unknown_probability(msg.unknown_prob) {}

  /**
   * @brief Return color probabilities of the cone
   * @return Color probabilities
   */
  ColorProbabilities GetColorProbabilities() const {
    return {
      .blue = blue_probability,
      .yellow = yellow_probability,
      .orange = orange_probability,
      .big_orange = big_orange_probability,
      .unknown = unknown_probability
    };
  }

  /**
   * @brief Return the cone ID
   * @return Cone ID
   */
  int GetId() const {
    return 0;
  }

  /**
  * @brief Print function used by GTest
  * @param cone Cone object
  * @param os Stream
  */
  friend void PrintTo(const ConeWithColorProbability& cone, std::ostream* os) {
    *os << "(" << cone.position << ")";
    *os << "(" << cone.covariance << ")";
    *os << "(" << cone.blue_probability << ")";
    *os << "(" << cone.yellow_probability << ")";
    *os << "(" << cone.orange_probability << ")";
    *os << "(" << cone.big_orange_probability << ")";
    *os << "(" << cone.unknown_probability << ")";
  }

 private:
  double blue_probability{.0};
  double yellow_probability{.0};
  double orange_probability{.0};
  double big_orange_probability{.0};
  double unknown_probability{1.};
};

template <constraint::ConeWithColorProbability ConeType>
std::vector<ConeType> FromMsg(const eufs_msgs::msg::ConeArrayWithCovariance &msg) {
  auto to_cone = [](const ColorProbabilities &color_probabilities) {
    return [color_probabilities](const eufs_msgs::msg::ConeWithCovariance &msg) {
      return ConeType(msg, color_probabilities);
    };
  };
  auto to_blue = to_cone({
    .blue = 1.,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  });
  auto to_yellow = to_cone({
    .blue = .0,
    .yellow = 1.,
    .orange = .0,
    .big_orange = .0,
    .unknown = .0
  });
  auto to_orange = to_cone({
    .blue = .0,
    .yellow = .0,
    .orange = 1.,
    .big_orange = .0,
    .unknown = .0
  });
  auto to_unknown = to_cone({
    .blue = .0,
    .yellow = .0,
    .orange = .0,
    .big_orange = .0,
    .unknown = 1.
  });

  std::vector<ConeType> cones;
  auto it = std::back_inserter(cones);
  std::transform(msg.blue_cones.cbegin(), msg.blue_cones.cend(), it, to_blue);
  std::transform(msg.yellow_cones.cbegin(), msg.yellow_cones.cend(), it, to_yellow);
  std::transform(msg.orange_cones.cbegin(), msg.orange_cones.cend(), it, to_orange);
  std::transform(msg.big_orange_cones.cbegin(), msg.big_orange_cones.cend(), it, to_orange);
  std::transform(msg.unknown_color_cones.cbegin(), msg.unknown_color_cones.cend(), it, to_unknown);

  return cones;
}

template <constraint::ConeWithColorProbability ConeType>
std::vector<ConeType> FromMsg(const eufs_msgs::msg::ConeWithColorProbabilityArray &msg) {
  auto to_cone = [](const eufs_msgs::msg::ConeWithColorProbability &msg) {
    return ConeType(msg);
  };

  std::vector<ConeType> cones;
  auto it = std::back_inserter(cones);
  std::transform(msg.cones.cbegin(), msg.cones.cend(), it, to_cone);

  return cones;
}

template <constraint::ConeWithColorProbability ConeType>
eufs_msgs::msg::ConeWithColorProbability ToMsg(const ConeType &cone) {
  eufs_msgs::msg::ConeWithColorProbability msg;
  ColorProbabilities color_probabilities = cone.GetColorProbabilities();
  msg.id = cone.GetId();
  msg.blue_prob = color_probabilities.blue;
  msg.yellow_prob = color_probabilities.yellow;
  msg.orange_prob = color_probabilities.orange;
  msg.unknown_prob = color_probabilities.unknown;
  msg.point.x = cone.position(0);
  msg.point.y = cone.position(1);
  msg.covariance = {
      cone.covariance(0, 0),
      cone.covariance(0, 1),
      cone.covariance(1, 0),
      cone.covariance(1, 1),
  };
  return msg;
}

template <constraint::ConeWithColorProbability ConeType>
eufs_msgs::msg::ConeWithColorProbabilityArray ToMsg(std::vector<ConeType> cones) {
  eufs_msgs::msg::ConeWithColorProbabilityArray msg;
  for (const auto &cone : cones) {
        msg.cones.push_back(ToMsg(cone));
    }
  return msg;
}

template <constraint::ConeWithColorProbability ConeType>
eufs_msgs::msg::ConeWithColorProbabilityArray ToMsg(const std::vector<ConeType> &cones,
                                                  const std_msgs::msg::Header &header) {
  auto msg = eufs::map::ToMsg(cones);
  msg.header = header;
  return msg;
}


}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_TYPE_CONE_WITH_COLOR_PROBABILITY_HPP_
