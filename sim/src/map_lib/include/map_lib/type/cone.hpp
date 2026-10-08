#ifndef INCLUDE_MAP_LIB_TYPE_CONE_HPP_
#define INCLUDE_MAP_LIB_TYPE_CONE_HPP_

#include "map_lib/constraint/cone.hpp"
#include "map_lib/constraint/cone_plus.hpp"

#include <Eigen/Dense>

#include <array>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_color_probability.hpp>
#include <eufs_msgs/msg/cone_with_color_probability_array.hpp>
#include <std_msgs/msg/header.hpp>

namespace eufs::map {

enum class Color {
  BLUE,
  YELLOW,
  ORANGE,
  UNKNOWN
};

struct Cone {
  typedef std::shared_ptr<Cone> Ptr;
  typedef std::shared_ptr<const Cone> ConstPtr;

  // (x,y) coordinate of the cone
  Eigen::Vector2d position;

  // Covariance in the position of the cone
  Eigen::Matrix2d covariance;

  /**
   * @brief Construct a new Cone object
   */
  Cone() = default;

  /**
   * @brief Construct a new Cone object
   *
   * @param pos (x,y)-coordinate
   * @param c Color of cone
   */
  explicit Cone(const Eigen::Vector2d &pos, const Color &c = Color::UNKNOWN)
      : position(pos), color(c) {}

  /**
   * @brief Construct a new Cone object
   *
   * @param pos (x,y)-coordinate
   * @param c Color of cone
   */
  explicit Cone(const std::array<double, 2> &pos, const Color &c = Color::UNKNOWN)
      : position(pos.data()), color(c) {}

  /**
   * @brief Construct a new Cone object
   *
   * @param pos (x,y)-coordinate
   * @param cov Covariance of position
   * @param c Color of cone
   */
  Cone(const Eigen::Vector2d &pos, const Eigen::Matrix2d &cov, const Color &c = Color::UNKNOWN)
      : position(pos), covariance(cov), color(c) {}

  /**
   * @brief Construct a new Cone object
   *
   * @param pos (x,y)-coordinate
   * @param cov Covariance of x and y as a flattened array of size 4 (row-wise)
   * @param c Color of cone
   */
  Cone(const std::array<double, 2> &pos, const std::array<double, 4> &cov,
       const Color &c = Color::UNKNOWN)
      : position(pos.data()), covariance(cov.data()), color(c) {}

  /**
   * @brief Construct a new Cone object
   *
   * @param msg eufs_msgs::msg::ConeWithCovariance message
   * @param c Color of cone
   */
  explicit Cone(const eufs_msgs::msg::ConeWithCovariance &msg, const Color &c = Color::UNKNOWN)
      : position(msg.point.x, msg.point.y), covariance(msg.covariance.data()), color(c) {}

    /**
   * @brief Construct a new Cone object
   *
   * @param msg eufs_msgs::msg::ConeWithColorProbability message
   */
  explicit Cone(const eufs_msgs::msg::ConeWithColorProbability &msg)
      : position(msg.point.x, msg.point.y), covariance(msg.covariance.data()), color(
        MaximumLikelihood(msg.blue_prob, msg.yellow_prob, msg.big_orange_prob, msg.orange_prob,
          msg.unknown_prob)), id(msg.id) {}

  /**
   * @brief Return color of cone
   * @return Color
   */
  Color GetColor() const { return color; }

  /**
   * @brief Return color of cone
   * @return Color
   */  
  Color &GetColor() { return color; }

  /**
   * @brief Set color of cone to given color
   * 
   * @param color Color (orange, yellow, blue)
   */
  void SetColor(Color Color) { color = Color; }

  /**
   * @brief Return colour probabilities of the cone
   *
   * @param discount_factor Discount factor used by richer cone types. Not used for base Cone.
   * @return Colour probabilities keyed by map::Color
   */
  std::unordered_map<Color, double> GetColorProbabilities(const double discount_factor) const {
    (void)discount_factor;
    return {
      {Color::BLUE, color == Color::BLUE ? 1.0 : 0.0},
      {Color::YELLOW, color == Color::YELLOW ? 1.0 : 0.0},
      {Color::ORANGE, color == Color::ORANGE ? 1.0 : 0.0},
      {Color::UNKNOWN, color == Color::UNKNOWN ? 1.0 : 0.0},
    };
  }

  /**
   * @brief Return the cone ID
   * @return Cone ID
   */
  int GetId() const {
    return id;
  }

  void SetId(const int cone_id) {
    id = cone_id;
  }

  /**
   * @brief Convert string to Color enum value
   *
   * @param color Color (blue, yellow, orange, big_orange), otherwise unknown color
   */
  static Color ToColor(const std::string color) {
    if (color == "blue")
      return Color::BLUE;
    else if (color == "yellow")
      return Color::YELLOW;
    else if (color == "orange" || color == "big_orange")
      return Color::ORANGE;
    else
      return Color::UNKNOWN;
  }

  /**
  * @brief Print function used by GTest
  * @param cone Cone object
  * @param os Stream
  */
  friend void PrintTo(const Cone& cone, std::ostream* os) {
    *os << "(" << cone.position << ")";
    *os << "(" << cone.covariance << ")";
  }

 private:
  Color MaximumLikelihood(
      const double blue_probability,
      const double yellow_probability,
      const double orange_probability,
      const double big_orange_probability,
      const double unknown_probability) {
    const double max_probability = std::max({
      blue_probability,
      yellow_probability,
      orange_probability,
      big_orange_probability,
      unknown_probability
    });

    if (blue_probability == max_probability) {
      return Color::BLUE;
    }
    if (yellow_probability == max_probability) {
      return Color::YELLOW;
    }
    if (orange_probability == max_probability || big_orange_probability == max_probability) {
      return Color::ORANGE;
    }

    return Color::UNKNOWN;
  }

 protected:
  // Color of the cone
  Color color = Color::UNKNOWN;

  // ID of the cone
  int id = 0;
};

template <constraint::Cone ConeType>
std::vector<ConeType> FromMsg(const eufs_msgs::msg::ConeArrayWithCovariance &msg) {
  auto to_cone = [](const Color &color) {
    return [color](const eufs_msgs::msg::ConeWithCovariance &msg) { return ConeType(msg, color); };
  };
  auto to_blue = to_cone(Color::BLUE);
  auto to_yellow = to_cone(Color::YELLOW);
  auto to_orange = to_cone(Color::ORANGE);
  auto to_unknown = to_cone(Color::UNKNOWN);

  std::vector<ConeType> cones;
  auto it = std::back_inserter(cones);
  std::transform(msg.blue_cones.cbegin(), msg.blue_cones.cend(), it, to_blue);
  std::transform(msg.yellow_cones.cbegin(), msg.yellow_cones.cend(), it, to_yellow);
  std::transform(msg.orange_cones.cbegin(), msg.orange_cones.cend(), it, to_orange);
  std::transform(msg.big_orange_cones.cbegin(), msg.big_orange_cones.cend(), it, to_orange);
  std::transform(msg.unknown_color_cones.cbegin(), msg.unknown_color_cones.cend(), it, to_unknown);

  return cones;
}

template <constraint::Cone ConeType>
std::vector<ConeType> FromMsg(const eufs_msgs::msg::ConeWithColorProbabilityArray &msg) {
  auto to_cone = [](const eufs_msgs::msg::ConeWithColorProbability &msg) {
    return ConeType(msg);
  };

  std::vector<ConeType> cones;
  auto it = std::back_inserter(cones);
  std::transform(msg.cones.cbegin(), msg.cones.cend(), it, to_cone);

  return cones;
}

template <constraint::Cone ConeType>
eufs_msgs::msg::ConeWithCovariance ToMsg(const ConeType &cone) {
  eufs_msgs::msg::ConeWithCovariance msg;
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

template <constraint::Cone ConeType>
eufs_msgs::msg::ConeArrayWithCovariance ToMsg(std::vector<ConeType> cones) {
  eufs_msgs::msg::ConeArrayWithCovariance msg;
  for (const auto &cone : cones) {
    switch (cone.GetColor()) {
      case Color::BLUE:
        msg.blue_cones.push_back(ToMsg(cone));
        break;
      case Color::YELLOW:
        msg.yellow_cones.push_back(ToMsg(cone));
        break;
      case Color::ORANGE:
        msg.orange_cones.push_back(ToMsg(cone));
        break;
      default:
        msg.unknown_color_cones.push_back(ToMsg(cone));
        break;
    }
  }
  return msg;
}

template <constraint::Cone ConeType>
eufs_msgs::msg::ConeArrayWithCovariance ToMsg(const std::vector<ConeType> &cones,
                                              const std_msgs::msg::Header &header) {
  auto msg = eufs::map::ToMsg(cones);
  msg.header = header;
  return msg;
}


template <constraint::ConePlus ConeType>
eufs_msgs::msg::ConeWithColorProbability ToMsg(const ConeType &cone,
                                             const double discount_factor) {
  eufs_msgs::msg::ConeWithColorProbability msg;
  std::unordered_map<map::Color, double> prob_map =
                                  cone.GetColorProbabilities(discount_factor);
  msg.id = cone.GetId();
  msg.blue_prob = prob_map[map::Color::BLUE];
  msg.yellow_prob = prob_map[map::Color::YELLOW];
  msg.orange_prob = prob_map[map::Color::ORANGE];
  msg.unknown_prob = prob_map[map::Color::UNKNOWN];
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

template <constraint::ConePlus ConeType>
eufs_msgs::msg::ConeWithColorProbabilityArray ToMsg(std::vector<ConeType> cones,
                                                  const double discount_factor) {
  eufs_msgs::msg::ConeWithColorProbabilityArray msg;
  for (const auto &cone : cones) {
        msg.cones.push_back(ToMsg(cone, discount_factor));
    }
  return msg;
}

template <constraint::ConePlus ConeType>
eufs_msgs::msg::ConeWithColorProbabilityArray ToMsg(const std::vector<ConeType> &cones,
                                                  const std_msgs::msg::Header &header,
                                                  const double discount_factor) {
  auto msg = eufs::map::ToMsg(cones, discount_factor);
  msg.header = header;
  return msg;
}

}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_TYPE_CONE_HPP_
