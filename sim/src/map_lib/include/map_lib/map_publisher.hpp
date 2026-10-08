#ifndef INCLUDE_MAP_LIB_MAP_PUBLISHER_HPP_
#define INCLUDE_MAP_LIB_MAP_PUBLISHER_HPP_

#include <string>

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <rclcpp/rclcpp.hpp>

namespace eufs::map {

class MapPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Map Publisher Node object
   */
  MapPublisher();

  /**
   * @brief Publish map
   */
  void PublishMap();

 private:
  /**
   * @brief Get the path to the selected map in the setup.json when using the EUFS Testing App.
   *
   * @return std::string
   */
  std::string GetSelectedMapPath();

  // ROS Publishers
  rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr map_publisher_;

  // ROS Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Map to publish
  eufs_msgs::msg::ConeArrayWithCovariance map_msg_;
};

}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_MAP_PUBLISHER_HPP_
