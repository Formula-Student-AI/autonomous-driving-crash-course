#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_STEERING_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_STEERING_MODEL_HPP_

#include <concepts>

#include <rclcpp/duration.hpp>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/types/command.hpp"

namespace eufs::vehicle_models::constraint {

template <typename T>
concept SteeringModel = requires(
    T t,
    const Param::Steering steering_params,
    const double prev_steering_angle,
    const Command command,
    const rclcpp::Duration dt) {
  T(steering_params);
  { t.GetSteeringAngle(prev_steering_angle, command, dt) } -> std::convertible_to<double>;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_STEERING_MODEL_HPP_
