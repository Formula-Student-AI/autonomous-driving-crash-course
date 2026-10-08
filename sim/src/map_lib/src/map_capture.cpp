#include "map_lib/map_capture.hpp"

#include <functional>

#include <eufs_logger/logger.hpp>

#include "map_lib/io/writer.hpp"
#include "map_lib/type/cone.hpp"

namespace eufs::map {

MapCapture::MapCapture() : rclcpp::Node("map_capture") {
  // ROS parameters
  save_path_ = declare_parameter<std::string>("save_path", "");
  map_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
      "/map", 1, std::bind(&MapCapture::MapCallback, this, std::placeholders::_1));
}

void MapCapture::MapCallback(const eufs_msgs::msg::ConeArrayWithCovariance &msg) const {
  auto path = save_path_ + "map_" + logging::GenerateTimeStamp() + ".csv";
  io::SaveCones(FromMsg<Cone>(msg), path);
  RCLCPP_INFO(get_logger(), "Saved map to %s", path.c_str());
}

}  // namespace eufs::map

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto map_capture = std::make_shared<eufs::map::MapCapture>();
  rclcpp::spin(map_capture);
  rclcpp::shutdown();
  return 0;
}
