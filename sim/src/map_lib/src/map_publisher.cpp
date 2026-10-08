#include "map_lib/map_publisher.hpp"

#include <jsoncpp/json/json.h>
#include <rmw/qos_profiles.h>

#include <chrono>  // NOLINT
#include <fstream>
#include <filesystem>
#include <string>

#include "map_lib/io/reader.hpp"
#include "map_lib/type/cone.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace eufs::map {

MapPublisher::MapPublisher() : rclcpp::Node("map_publisher") {
  // ROS parameters
  auto map_path = declare_parameter<std::string>("map_path", "");
  auto frame_id = declare_parameter<std::string>("frame_id", "map");
  auto frequency = declare_parameter<int>("frequency", 0);

  // Update map path with selected map if none given
  if (map_path.empty()) {
    map_path = GetSelectedMapPath();
  }

  if (map_path.empty()) {
    RCLCPP_FATAL(get_logger(), "No map selected");
    return;
  }

  // Load map
  auto cones = eufs::map::io::LoadCones<eufs::map::Cone>(map_path);
  RCLCPP_INFO(get_logger(), "Loaded map (%lu cones)", cones.size());

  // Header for map
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = frame_id;

  // Create map msg
  map_msg_ = eufs::map::ToMsg(cones, header);

  // ROS Publisher
  if (frequency == 0) {
    rclcpp::QoS qos = rclcpp::QoS(1).transient_local();
    map_publisher_ = create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>("/map", qos);
    map_publisher_->publish(map_msg_);
  } else {
    auto period = std::chrono::nanoseconds(static_cast<int>(1e9 / static_cast<double>(frequency)));
    timer_ = create_wall_timer(period, std::bind(&MapPublisher::PublishMap, this));
    map_publisher_ = create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>("/map", 1);
  }
}

std::string MapPublisher::GetSelectedMapPath() {
  std::filesystem::path eufs_master(std::getenv("EUFS_MASTER"));
  std::filesystem::path setup_file("data/current/setup.json");

  std::ifstream setup_file_stream(eufs_master / setup_file);

  if (!setup_file_stream) {
    RCLCPP_WARN(get_logger(), "Could not open setup.json");
    return "";
  }

  Json::Reader reader;
  Json::Value complete_json_data;
  reader.parse(setup_file_stream, complete_json_data);

  if (!complete_json_data.isMember("map")) {
    RCLCPP_WARN(get_logger(), "Could not find map in setup.json");
    return "";
  }

  std::string map_path = complete_json_data["map"].asString() + "/map_utm.csv";
  RCLCPP_INFO(get_logger(), "Selected map: %s", map_path.c_str());
  return map_path;
}

void MapPublisher::PublishMap() { map_publisher_->publish(map_msg_); }

}  // namespace eufs::map

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto map_publisher = std::make_shared<eufs::map::MapPublisher>();
  rclcpp::spin(map_publisher);
  rclcpp::shutdown();
  return 0;
}
