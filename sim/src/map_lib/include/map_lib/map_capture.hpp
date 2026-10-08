#ifndef INCLUDE_MAP_LIB_MAP_CAPTURE_HPP_
#define INCLUDE_MAP_LIB_MAP_CAPTURE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>

namespace eufs::map {

class MapCapture : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Map Capture node object
   */
  MapCapture();

  /**
   * @brief Receive map and save as csv
   * @param msg Map message
   */
  void MapCallback(const eufs_msgs::msg::ConeArrayWithCovariance &msg) const;

 private:
  std::string save_path_;
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr map_sub_;
};

}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_MAP_CAPTURE_HPP_
