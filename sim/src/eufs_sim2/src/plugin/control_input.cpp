#include "eufs_sim2/plugin/control_input.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {

void ControlInputPlugin::SetCommand(type::ControlInput cmd) {
  cmd_ = cmd;
  new_cmd_ = true;
}

void ControlInputPlugin::PreUpdate(SimulationBase &sim) {
  // Deadman: if the controller has stopped publishing /cmd (crash/hang), do NOT
  // keep applying the last command — command a braked stop. This is the plant's
  // last line of defence, independent of any autonomy fault.
  if (node_ && cmd_ever_received_) {
    const double age = (node_->now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_s_) {
      sim.SetCommand(type::ControlInput{failsafe_brake_, 0.0});
      new_cmd_ = false;
      return;
    }
  }

  // Only update the sim command if a new command has actually been received.
  if (new_cmd_) {
    sim.SetCommand(cmd_);
    new_cmd_ = false;
  }
}

void ControlInputPlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);
  cmd_timeout_s_ = node_->declare_parameter<double>(plugin_namespace + "cmd_timeout_s", 0.3);
  last_cmd_time_ = node_->now();

  // Create ROS subscribers
  cmd_sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 1, std::bind(&ControlInputPlugin::CommandCallback, this, std::placeholders::_1));
}

void ControlInputPlugin::CommandCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg) {
  type::ControlInput cmd = type::ToControlInput(msg);
  // Reject non-finite commands (a single NaN would corrupt the vehicle state);
  // brake safely instead. Clamp finite commands to physical limits.
  if (!std::isfinite(cmd.acceleration) || !std::isfinite(cmd.steering_angle)) {
    cmd = type::ControlInput{failsafe_brake_, 0.0};
  } else {
    cmd.acceleration = std::clamp(cmd.acceleration, -max_accel_, max_accel_);
    cmd.steering_angle = std::clamp(cmd.steering_angle, -max_steering_, max_steering_);
  }
  last_cmd_time_ = node_->now();
  cmd_ever_received_ = true;
  SetCommand(cmd);
}

}  // namespace eufs::sim2::plugin
