#ifndef INCLUDE_VEHICLE_MODELS_STEERING_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_STEERING_MODEL_HPP_

#include <rclcpp/duration.hpp>

#include <state_lib/state_vector.hpp>

#include "vehicle_models/types/command.hpp"
#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models {

class SteeringModel {
 public:
  /**
   * @brief Initialise steering model
   *
   * @param param Parameters of the motion model
   */
  explicit SteeringModel(const Param::Steering &steering_params);

  /**
   * @brief Get next steering angle
   *
   * @param command Latest command
   * @return Vehicle steering angle
   */
  double GetSteeringAngle(
      const double prev_steering_angle,
      const Command &command, const rclcpp::Duration &dt) const;

 private:
  double NextSteeringAngle(const double prev_steering_angle,
                           const Command &command, const rclcpp::Duration &dt) const;

 protected:
  Param::Steering steering_params_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_STEERING_MODEL_HPP_
