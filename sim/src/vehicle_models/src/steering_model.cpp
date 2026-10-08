#include "vehicle_models/steering_model.hpp"

namespace eufs::vehicle_models {

SteeringModel::SteeringModel(const Param::Steering &steering_params)
  : steering_params_(steering_params) {}

double SteeringModel::GetSteeringAngle(
    const double prev_steering_angle,
    const Command &command, const rclcpp::Duration &dt) const {
  return std::clamp(
      NextSteeringAngle(prev_steering_angle, command, dt),
        -steering_params_.max_angle, steering_params_.max_angle);
}

double SteeringModel::NextSteeringAngle(
    const double prev_steering_angle,
    const Command &command, const rclcpp::Duration &dt) const {
  if (std::abs(prev_steering_angle - command.steering_angle) >
      dt.seconds() * steering_params_.max_rate) {
    if (prev_steering_angle > command.steering_angle) {
      return prev_steering_angle - dt.seconds() * steering_params_.max_rate;
    } else {
      return prev_steering_angle + dt.seconds() * steering_params_.max_rate;
    }
  }

  return command.steering_angle;
}

}  // namespace eufs::vehicle_models
