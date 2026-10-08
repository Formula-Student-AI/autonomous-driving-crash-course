#include "vehicle_models/ros.hpp"

#include <string>

namespace eufs::vehicle_models {

Param SetParamsFromNode(rclcpp::Node &node) {
  std::string inertia_prefix = "inertia";
  std::string kinematic_prefix = "kinematic";
  std::string tyre_prefix = "tyre";
  std::string powertrain_prefix = "powertrain";
  std::string steering_prefix = "steering";
  std::string aero_prefix = "aero";
  std::string input_ranges_prefix = "input_ranges";

  return {.inertia = {
                  node.declare_parameter<double>(inertia_prefix + ".m"),
                  node.declare_parameter<double>(inertia_prefix + ".g"),
                  node.declare_parameter<double>(inertia_prefix + ".I_z"),
              },
          .kinematic = {
                  node.declare_parameter<double>(kinematic_prefix + ".l"),
                  node.declare_parameter<double>(kinematic_prefix + ".w_front"),
                  node.declare_parameter<double>(kinematic_prefix + ".axle_width"),
              },
          .tyre = {
                  node.declare_parameter<double>(tyre_prefix + ".A"),
                  node.declare_parameter<double>(tyre_prefix + ".B"),
                  node.declare_parameter<double>(tyre_prefix + ".C"),
                  node.declare_parameter<double>(tyre_prefix + ".radius"),
                  node.declare_parameter<double>(tyre_prefix + ".rolling_resistance"),
              },
          .powertrain = {
                  node.declare_parameter<double>(powertrain_prefix + ".overdrive"),
              },
          .steering = {
                  node.declare_parameter<double>(steering_prefix + ".max_rate"),
                  node.declare_parameter<double>(steering_prefix + ".max_angle")
              },
          .aero = {
                  node.declare_parameter<double>(aero_prefix + ".C_down"),
                  node.declare_parameter<double>(aero_prefix + ".C_drag"),
              },
          .input_ranges = {
              .acc = {
                      node.declare_parameter<double>(input_ranges_prefix + ".acceleration.min"),
                      node.declare_parameter<double>(input_ranges_prefix + ".acceleration.max"),
                  },
              .vel = {
                      node.declare_parameter<double>(input_ranges_prefix + ".velocity.min"),
                      node.declare_parameter<double>(input_ranges_prefix + ".velocity.max"),
                  },
              .delta = {
                  node.declare_parameter<double>(input_ranges_prefix + ".steering.min"),
                  node.declare_parameter<double>(input_ranges_prefix + ".steering.max"),
              }}};
}

}  // namespace eufs::vehicle_models
